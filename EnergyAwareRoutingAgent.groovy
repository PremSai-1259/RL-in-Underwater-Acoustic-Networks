
import org.arl.fjage.*
import org.arl.fjage.param.Parameter
import org.arl.unet.*
import org.arl.unet.link.LinkStatusNtf
import org.arl.unet.net.*
import org.arl.unet.nodeinfo.NodeInfoParam

class EAFTRAgent extends UnetAgent {

  enum Params implements Parameter {
    destination, energy, neighborInfo, qtable, nextHops, stats,
    failedLinks, failedNodes
  }

  static final String title = 'EAFTR Energy-Aware Routing'
  static final String description = 'Reinforcement learning based energy-aware fault-tolerant routing'

  static final int EAFTR_PROTOCOL = Protocol.USER + 1
  static final int DATA_PROTOCOL = Protocol.USER + 2

  static final double ALPHA = 0.15d
  static final double GAMMA = 0.90d
  static final double EPSILON = 0.05d

  static final double DATA_BYTES = 128d
  static final double DATA_RATE_BPS = 10000d
  static final double TX_POWER_W = 10d
  static final double RX_POWER_W = 2d
  static final double ROUTING_COST_J = 0.05d
  static final double RESERVE_J = 0.10d

  static final long ADVERTISE_MS = 10000L
  static final long REQUEST_MS = 15000L
  static final long ENERGY_TTL_MS = 60000L
  static final long DEBOUNCE_MS = 500L
  static final long ACK_TIMEOUT_MS = 8000L
  static final long SUSPECT_MS = 30000L

  static final int ADVERT_HOPS = 8
  static final int MAX_TTL = 10
  static final int MAX_RETRIES = 3

  static final Map<Integer,Double> INITIAL_ENERGY = [
    1:100d, 2:75d, 3:90d, 4:80d,
    5:45d, 6:85d, 7:70d, 8:100d
  ]

  final Map<Integer,List<Integer>> neighbors = [
    1:[2,3],
    2:[1,4,5],
    3:[1,5,6],
    4:[2,7],
    5:[2,3,7],
    6:[3,7],
    7:[4,5,6,8],
    8:[7]
  ]

  final Map<Integer,List<Double>> position = [
    1:[0d,0d,-10d],
    2:[100d,0d,-15d],
    3:[0d,100d,-20d],
    4:[200d,0d,-20d],
    5:[100d,100d,-25d],
    6:[0d,200d,-15d],
    7:[250d,100d,-20d],
    8:[350d,100d,-10d]
  ]

  int address = -1
  int destination = 8
  int sequence = 0

  double energyJ = 100d
  long energyVersion = 0L

  boolean training = false
  boolean publishScheduled = false

  Set<String> failedLinks = [] as Set<String>
  Set<Integer> failedNodes = [] as Set<Integer>

  private AgentID router
  private AgentID link

  private final Random rng = new Random(42)

  private Map<Integer,Map> neighbourState = [:]
  private Map<Integer,Map<Integer,Double>> q = [:]
  private Map<Integer,Integer> activeRoutes = [:]
  private Map<Integer,Double> observedLQ = [:]
  private Map<Integer,Long> suspectUntil = [:]
  private Map<String,Map> pending = [:]
  private Set<String> seen = new HashSet<String>()

  private Map<String,Number> stat = [
    generated:0,
    delivered:0,
    txData:0,
    txCtl:0,
    failedFwd:0,
    noRoute:0,
    duplicates:0,
    routeChanges:0,
    staleEvents:0
  ]

  private double totalDelayMs = 0d

  @Override
  List<Parameter> getParameterList() {
    allOf(Params)
  }

  boolean isReadOnly(Parameter p) {
    p == Params.neighborInfo ||
    p == Params.qtable ||
    p == Params.nextHops ||
    p == Params.stats
  }

  double getEnergy() {
    energyJ
  }

  void setEnergy(Object value) {
    if (!(value instanceof Number) ||
        !Double.isFinite(value as double) ||
        (value as double) < 0d) {
      throw new IllegalArgumentException(
        'Energy must be a non-negative number'
      )
    }

    energyJ = value as double
    energyVersion++
    updateOwnEnergy()
    refreshPolicy('energy manually changed')
    publishEnergy()
  }

  String getNeighborInfo() {
    long now = System.currentTimeMillis()

    neighbourState.keySet().sort().collect { int n ->
      Map s = neighbourState[n]
      long age = now - (s.receivedAt as long)

      "${n}: ${fmt(s.energy as double)} J, " +
      "version=${s.version}, age=${age}ms" +
      "${age > ENERGY_TTL_MS ? ' STALE' : ''}"
    }.join('\n')
  }

  String getQtable() {
    if (address < 1) return 'Agent is starting.'

    StringBuilder out =
      new StringBuilder("EAFTR Q-table at node ${address}\n")

    q.keySet().sort().each { int d ->
      out.append("Destination ${d}\n")

      (neighbors[address] ?: []).sort().each { int n ->
        out.append(
          "  Next hop ${n}: Q = ${fmt(q[d]?.get(n) ?: 0d)}\n"
        )
      }
    }

    out.toString()
  }

  String getNextHops() {
    activeRoutes.keySet().sort().collect { int d ->
      "destination=${d}, nextHop=${activeRoutes[d]}"
    }.join('\n')
  }

  String getStats() {
    "node=${address}, generated=${stat.generated}, " +
    "delivered=${stat.delivered}, txData=${stat.txData}, " +
    "failed=${stat.failedFwd}, noRoute=${stat.noRoute}, " +
    "energy=${fmt(energyJ)} J, avgDelayMs=" +
    "${stat.delivered ? fmt(totalDelayMs / (stat.delivered as double)) : 'n/a'}"
  }

  void setDestination(int d) {
    if (!neighbors.containsKey(d)) {
      throw new IllegalArgumentException('Unknown destination')
    }

    destination = d
    refreshPolicy('destination changed')
  }

  void failNode(int n) {
    failedNodes.add(n)
    refreshPolicy("node ${n} failed")
  }

  void restoreNode(int n) {
    failedNodes.remove(n)
    suspectUntil.remove(n)
    refreshPolicy("node ${n} restored")
  }

  void failLink(int a, int b) {
    failedLinks.add(edgeKey(a,b))
    refreshPolicy("link ${a}-${b} failed")
  }

  void restoreLink(int a, int b) {
    failedLinks.remove(edgeKey(a,b))
    refreshPolicy("link ${a}-${b} restored")
  }

  void sendTest(int count = 1) {
    count.times {
      sendData(destination)
    }
  }

  @Override
  void startup() {
    router = agentForService(Services.ROUTING)
    link = agentForService(Services.LINK)

    subscribe topic(Topics.DATAGRAM)
    if (link != null) subscribe topic(link)

    add new WakerBehavior(1000, {
      try {
        def nodeAgent = agentForService(Services.NODE_INFO)
        address = get(nodeAgent, NodeInfoParam.address) as int
        energyJ = INITIAL_ENERGY[address] ?: 100d

        seedEnergyEstimates()
        initialiseTables()
        updateOwnEnergy()

        log.info(
          "EAFTR START node=${address}, energy=${fmt(energyJ)}, " +
          "router=${router}, link=${link}"
        )

        refreshPolicy('startup')
        publishEnergy()
        sendControl(null, 'R')

        scheduleAdvertisement()
        scheduleEnergyRequests()

      } catch (Exception ex) {
        log.severe(
          "EAFTR STARTUP ERROR ${ex.class.simpleName}: ${ex.message}"
        )
      }
    })
  }

  @Override
  void processMessage(Message msg) {
    if (msg instanceof DatagramNtf) {
      if (msg.protocol == EAFTR_PROTOCOL ||
          msg.protocol == DATA_PROTOCOL) {
        handleDatagram(msg)
      }
      return
    }

    if (msg instanceof LinkStatusNtf &&
        msg.sender == link &&
        address > 0) {

      int peer = msg.to as int
      if (!(neighbors[address] ?: []).contains(peer)) return

      if (msg.up) {
        failedLinks.remove(edgeKey(address,peer))
      } else {
        failedLinks.add(edgeKey(address,peer))
      }

      refreshPolicy("link status changed: ${peer}")
    }
  }

  private void seedEnergyEstimates() {
    long now = System.currentTimeMillis()

    INITIAL_ENERGY.each { int n, double e ->
      neighbourState[n] = [
        energy:e,
        version:0L,
        receivedAt:now,
        sentAt:now,
        estimated:true
      ]
    }
  }

  private void initialiseTables() {
    neighbors.keySet().each { int d ->
      if (!q.containsKey(d)) q[d] = [:]

      (neighbors[address] ?: []).each { int n ->
        if (!q[d].containsKey(n)) q[d][n] = 0d
      }
    }
  }

  private void scheduleAdvertisement() {
    add new WakerBehavior(ADVERTISE_MS, {
      if (address > 0) {
        publishEnergy()
        scheduleAdvertisement()
      }
    })
  }

  private void scheduleEnergyRequests() {
    add new WakerBehavior(REQUEST_MS, {
      if (address > 0) {
        sendControl(null,'R')
        scheduleEnergyRequests()
      }
    })
  }

  private double txEnergy(double bytes) {
    TX_POWER_W * bytes * 8d / DATA_RATE_BPS
  }

  private double rxEnergy(double bytes) {
    RX_POWER_W * bytes * 8d / DATA_RATE_BPS
  }

  private void updateOwnEnergy() {
    long now = System.currentTimeMillis()

    neighbourState[address] = [
      energy:energyJ,
      version:energyVersion,
      receivedAt:now,
      sentAt:now,
      estimated:false
    ]
  }

  private void charge(double joules, boolean publishIt = false) {
    if (address < 1 || joules <= 0d) return

    energyJ = Math.max(0d,energyJ-joules)
    energyVersion++
    updateOwnEnergy()

    if (energyJ <= RESERVE_J) {
      log.warning(
        "EAFTR ENERGY_LOW node=${address}, energy=${fmt(energyJ)}"
      )
    }

    refreshPolicy('energy consumed')

    if (publishIt && !publishScheduled) {
      publishScheduled = true

      add new WakerBehavior(DEBOUNCE_MS, {
        publishScheduled = false
        publishEnergy()
      })
    }
  }

  private void publishEnergy() {
    if (address < 1) return

    updateOwnEnergy()

    String payload =
      "E|${address}|${fmt(energyJ)}|${energyVersion}|${ADVERT_HOPS}"

    sendControl(null,payload)
  }

  private void sendControl(Integer target, String payload) {
    List<Integer> targets = target == null
      ? (neighbors[address] ?: [])
      : [target]

    targets.each { int n ->
      if (!failedNodes.contains(n) &&
          !failedLinks.contains(edgeKey(address,n))) {
        transmit(n,EAFTR_PROTOCOL,payload,false)
      }
    }
  }

  private boolean transmit(
    int target,
    int protocol,
    String payload,
    boolean dataPacket
  ) {
    if (failedNodes.contains(address) ||
        failedNodes.contains(target) ||
        failedLinks.contains(edgeKey(address,target))) {
      return false
    }

    if (link == null) {
      log.warning('EAFTR: link agent is unavailable')
      return false
    }

    byte[] bytes = payload.getBytes('UTF-8')

    double cost = dataPacket
      ? txEnergy(DATA_BYTES)
      : txEnergy(bytes.length as double)

    if (energyJ < cost + RESERVE_J) {
      log.warning("EAFTR TX_SKIPPED node=${address}: insufficient energy")
      return false
    }

    charge(cost,dataPacket)

    try {
      send new DatagramReq(
        recipient:link,
        to:target,
        protocol:protocol,
        reliability:false,
        data:bytes
      )

      if (dataPacket) {
        stat.txData++
      } else {
        stat.txCtl++
      }

      if (protocol == EAFTR_PROTOCOL && payload.startsWith('E|')) {
        log.info("EAFTR ENERGY_TX node=${address} to=${target}")
      }

      return true

    } catch (Exception ex) {
      log.warning(
        "EAFTR TX_ERROR to=${target}: ${ex.class.simpleName}: ${ex.message}"
      )
      return false
    }
  }

  private void handleDatagram(DatagramNtf msg) {
    if (address < 1 || failedNodes.contains(address)) return

    int from = msg.from as int

    if (!(neighbors[address] ?: []).contains(from)) return
    if (failedLinks.contains(edgeKey(address,from))) return

    byte[] raw = msg.data as byte[]
    boolean dataPacket = msg.protocol == DATA_PROTOCOL

    double cost = dataPacket
      ? rxEnergy(DATA_BYTES)
      : rxEnergy((raw?.length ?: 0) as double)

    if (energyJ < cost + RESERVE_J) return

    charge(cost,dataPacket)
    charge(ROUTING_COST_J)

    String payload
    try {
      payload = new String(raw,'UTF-8').trim()
    } catch (Exception ex) {
      return
    }

    String[] p = payload.split('\\|',-1)

    try {
      if (dataPacket) {
        onData(from,p)
      } else if (p[0] == 'R') {
        sendOwnEnergy(from)
      } else if (p[0] == 'E' && p.length == 5) {
        onEnergy(from,p)
      } else if (p[0] == 'A' && p.length == 3) {
        onAck(from,p)
      }
    } catch (Exception ex) {
      log.warning(
        "EAFTR BAD_MESSAGE from=${from}: ${ex.class.simpleName}: ${ex.message}"
      )
    }
  }

  private void sendOwnEnergy(int target) {
    String payload =
      "E|${address}|${fmt(energyJ)}|${energyVersion}|1"

    sendControl(target,payload)
  }

  private void onEnergy(int from, String[] p) {
    int origin = p[1] as int
    double e = p[2] as double
    long version = p[3] as long
    int ttl = p[4] as int

    if (!INITIAL_ENERGY.containsKey(origin) || origin == address) return
    if (!Double.isFinite(e) || e < 0d || ttl < 1) return

    Map old = neighbourState[origin]

    if (old != null &&
        old.estimated != true &&
        version <= (old.version as long)) {
      stat.duplicates++
      return
    }

    neighbourState[origin] = [
      energy:e,
      version:version,
      receivedAt:System.currentTimeMillis(),
      sentAt:System.currentTimeMillis(),
      estimated:false
    ]

    log.info(
      "EAFTR ENERGY_UPDATED node=${address}, origin=${origin}, " +
      "energy=${fmt(e)}, version=${version}"
    )

    refreshPolicy("energy update from ${origin}")

    if (ttl > 1) {
      (neighbors[address] ?: []).findAll {
        it != from && it != origin
      }.each { int n ->
        sendControl(
          n,
          "E|${origin}|${fmt(e)}|${version}|${ttl-1}"
        )
      }
    }
  }

  private double freshEnergy(int n) {
    Map s = neighbourState[n]
    if (s == null) return -1d

    long age = System.currentTimeMillis() - (s.receivedAt as long)

    if (age > ENERGY_TTL_MS) return -1d

    s.energy as double
  }

  private boolean suspect(int n) {
    Long until = suspectUntil[n]
    until != null && until > System.currentTimeMillis()
  }

  private boolean canTransmit() {
    energyJ >= txEnergy(DATA_BYTES) + ROUTING_COST_J + RESERVE_J
  }

  private boolean feasible(int n, int d) {
    if (n == address ||
        failedNodes.contains(n) ||
        failedNodes.contains(address)) return false

    if (failedLinks.contains(edgeKey(address,n)) || suspect(n)) return false

    double e = freshEnergy(n)
    if (e < 0d) return false

    double required = n == d
      ? rxEnergy(DATA_BYTES) + RESERVE_J
      : rxEnergy(DATA_BYTES) + txEnergy(DATA_BYTES) +
        ROUTING_COST_J + RESERVE_J

    if (e < required) return false

    pathExists(n,d,[address] as Set<Integer>)
  }

  private boolean pathExists(
    int start,
    int target,
    Set<Integer> blocked
  ) {
    if (start == target) return true

    ArrayDeque<Integer> queue = new ArrayDeque<Integer>()
    Set<Integer> visited = new HashSet<Integer>(blocked)

    queue.add(start)
    visited.add(start)

    while (!queue.isEmpty()) {
      int u = queue.removeFirst()

      for (int v : (neighbors[u] ?: [])) {
        if (visited.contains(v) ||
            failedNodes.contains(v) ||
            failedLinks.contains(edgeKey(u,v))) continue

        double e = freshEnergy(v)

        if (v != target && e >= 0d &&
            e < rxEnergy(DATA_BYTES) + txEnergy(DATA_BYTES) +
                ROUTING_COST_J + RESERVE_J) {
          continue
        }

        if (v == target) return true

        visited.add(v)
        queue.add(v)
      }
    }

    return false
  }

  private double distance(int a, int b) {
    List<Double> x = position[a]
    List<Double> y = position[b]

    double sum = 0d

    for (int i = 0; i < 3; i++) {
      sum += Math.pow(x[i]-y[i],2d)
    }

    Math.sqrt(sum)
  }

  private double reward(int n, int d) {
    if (n == d) return 100d

    double e = freshEnergy(n)
    if (e < 0d) return -20d

    double energyBenefit =
      10d * Math.min(1d,Math.max(0d,e/(INITIAL_ENERGY[n] ?: 100d)))

    double lq = Math.exp(-distance(address,n)/500d) *
      (observedLQ.containsKey(n) ? observedLQ[n] : 1d)

    double progress =
      5d * (distance(address,d)-distance(n,d)) /
      Math.max(1d,distance(address,d))

    double cost =
      3d * (txEnergy(DATA_BYTES)+rxEnergy(DATA_BYTES)+ROUTING_COST_J)

    energyBenefit + 5d*lq + progress - cost - 1d
  }

  private List<Integer> feasibleActions(int d) {
    if (!canTransmit()) return []

    (neighbors[address] ?: []).findAll { int n ->
      feasible(n,d)
    }
  }

  private Integer chooseAction(int d, Collection<Integer> excluded) {
    List<Integer> actions =
      feasibleActions(d).findAll { !excluded.contains(it) }

    if (actions.isEmpty()) return null

    if (training && rng.nextDouble() < EPSILON) {
      return actions[rng.nextInt(actions.size())]
    }

    Map<Integer,Double> table = q[d] ?: [:]

    double best = actions.collect { int n ->
      (table[n] ?: 0d) + reward(n,d)
    }.max() as double

    List<Integer> ties = actions.findAll { int n ->
      Math.abs((table[n] ?: 0d)+reward(n,d)-best) < 0.000001d
    }

    ties[rng.nextInt(ties.size())]
  }

  private double bestQ(int d) {
    List<Integer> actions = feasibleActions(d)
    if (actions.isEmpty()) return 0d

    actions.collect { int n ->
      q[d]?.get(n) ?: 0d
    }.max() as double
  }

  private void updateQ(int d, int action, double r, double futureQ) {
    if (!q.containsKey(d)) q[d] = [:]

    double old = q[d][action] ?: 0d

    q[d][action] =
      old + ALPHA * (r + GAMMA*futureQ - old)
  }

  private void refreshPolicy(String reason) {
    if (address < 1) return

    initialiseTables()

    q.keySet().each { int d ->
      (neighbors[address] ?: []).each { int n ->
        double old = q[d][n] ?: 0d
        double target = feasible(n,d) ? reward(n,d) : -20d

        q[d][n] = old + ALPHA*(target-old)
      }
    }

    Map<Integer,Integer> updated = [:]

    neighbors.keySet().each { int d ->
      if (d != address) {
        Integer nh = chooseAction(d,[])
        if (nh != null) updated[d] = nh
      }
    }

    if (updated != activeRoutes) {
      stat.routeChanges++
    }

    activeRoutes = updated

    log.info(
      "EAFTR QTABLE_RECALCULATED node=${address}, reason=${reason}, " +
      "nextHops=${activeRoutes}"
    )

    syncRouterRoutes()
  }

  private void syncRouterRoutes() {
    if (router == null || address < 1 || failedNodes.contains(address)) return

    activeRoutes.each { int d, int nh ->
      if (d == address ||
          failedNodes.contains(d) ||
          failedNodes.contains(nh) ||
          failedLinks.contains(edgeKey(address,nh))) {
        return
      }

      try {
        def req = EditRouteReq.newRoute()
        req.to = d
        req.nextHop = nh
        req.link = link?.name ?: 'uwlink'
        req.hops = 1
        req.reliability = true
        req.metric = (1000d - (q[d]?.get(nh) ?: 0d)) as float
        req.auto = false
        req.recipient = router

        send req

        log.info(
          "EAFTR ROUTE_REQUESTED node=${address}, destination=${d}, nextHop=${nh}"
        )
      } catch (Exception ex) {
        log.warning(
          "EAFTR ROUTE_SYNC_ERROR destination=${d}: " +
          "${ex.class.simpleName}: ${ex.message}"
        )
      }
    }
  }

  private void sendData(int dst) {
    if (address < 1 || failedNodes.contains(address)) return

    String id = "${address}-${sequence++}"
    stat.generated++

    forward(
      id,address,dst,MAX_TTL,[address],
      System.currentTimeMillis(),0,[]
    )
  }

  private void forward(
    String id,
    int src,
    int dst,
    int ttl,
    List<Integer> path,
    long started,
    int attempt,
    List<Integer> tried
  ) {
    charge(ROUTING_COST_J,true)

    if (ttl <= 0) {
      stat.failedFwd++
      log.warning("EAFTR TTL_EXPIRED id=${id}")
      return
    }

    Integer nh = chooseAction(dst,path+tried)

    if (nh == null) {
      stat.noRoute++

      log.warning(
        "EAFTR NO_FEASIBLE_ROUTE node=${address}, destination=${dst}"
      )

      sendControl(null,'R')

      if (attempt == 0) {
        stat.staleEvents++

        add new WakerBehavior(2000, {
          forward(
            id,src,dst,ttl,path,started,1,tried
          )
        })
      }

      return
    }

    String payload =
      "D|${id}|${src}|${dst}|${ttl}|${path.join(',')}|${started}"

    boolean sent = transmit(nh,DATA_PROTOCOL,payload,true)

    if (!sent) {
      stat.failedFwd++
      updateQ(dst,nh,-20d,0d)
      refreshPolicy('data transmission failed')
      return
    }

    Map ev = [
      nh:nh,
      src:src,
      dst:dst,
      ttl:ttl,
      path:path,
      started:started,
      tried:tried
    ]

    pending[id] = ev

    add new WakerBehavior(ACK_TIMEOUT_MS, {
      onTimeout(id,ev)
    })
  }

  private void onData(int from, String[] p) {
    if (p.length != 7) return

    String id = p[1]
    int src = p[2] as int
    int dst = p[3] as int
    int ttl = p[4] as int
    List<Integer> path = p[5].split(',').collect { it as int }
    long started = p[6] as long

    sendControl(from,"A|${id}|${fmt(bestQ(dst))}")

    if (!seen.add(id+'@'+address)) {
      stat.duplicates++
      return
    }

    if (path.contains(address)) {
      log.warning("EAFTR LOOP_DETECTED id=${id}")
      return
    }

    if (dst == address) {
      stat.delivered++
      totalDelayMs += System.currentTimeMillis()-started

      log.info(
        "EAFTR PACKET_DELIVERED id=${id}, path=${path+[address]}"
      )
      return
    }

    forward(
      id,src,dst,ttl-1,path+[address],
      started,0,[]
    )
  }

  private void onAck(int from, String[] p) {
    Map ev = pending.remove(p[1])
    if (ev == null) return

    int nh = ev.nh as int
    int d = ev.dst as int

    updateQ(d,nh,reward(nh,d),p[2] as double)

    double old = observedLQ.containsKey(nh) ? observedLQ[nh] : 1d
    observedLQ[nh] = 0.7d*old + 0.3d

    suspectUntil.remove(nh)
    refreshPolicy("ACK from ${from}")
  }

  private void onTimeout(String id, Map ev) {
    if (!pending.containsKey(id) || !pending[id].is(ev)) return

    pending.remove(id)

    int nh = ev.nh as int
    int d = ev.dst as int

    updateQ(d,nh,-20d,0d)

    double old = observedLQ.containsKey(nh) ? observedLQ[nh] : 1d
    observedLQ[nh] = 0.7d*old

    suspectUntil[nh] = System.currentTimeMillis()+SUSPECT_MS
    stat.failedFwd++

    refreshPolicy("ACK timeout at ${nh}")

    List<Integer> tried = (ev.tried as List<Integer>)+[nh]

    if (tried.size() <= MAX_RETRIES) {
      forward(
        id,
        ev.src as int,
        ev.dst as int,
        ev.ttl as int,
        ev.path as List<Integer>,
        ev.started as long,
        1,
        tried
      )
    } else {
      log.warning("EAFTR PACKET_LOST id=${id}")
    }
  }

  private static String fmt(double value) {
    String.format('%.3f',value)
  }

  private static String edgeKey(int a, int b) {
    "${Math.min(a,b)}-${Math.max(a,b)}"
  }
}
