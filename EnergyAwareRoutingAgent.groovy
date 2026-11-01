
import org.arl.fjage.*
import org.arl.fjage.param.Parameter
import org.arl.unet.*
import org.arl.unet.link.LinkStatusNtf
import org.arl.unet.net.*
import org.arl.unet.nodeinfo.NodeInfoParam

class EnergyAwareRoutingAgent extends UnetAgent {

  enum Params implements Parameter {
    destination, energy, failedLinks, failedNodes, episodes,
    neighborInfo, qtable, routes, traceRoute, debug
  }

  static final String title = 'EAFTR Q-learning routing'
  static final String description = 'Energy-aware fault-tolerant routing'
  static final int energyProtocol = Protocol.USER + 1

  static final double packetBytes = 128d
  static final double dataRate = 10000d
  static final double txPower = 14.65d
  static final double rxPower = 2d
  static final double routingCost = 0.05d
  static final double energyReserve = 0.10d

  static final Map<Integer, Double> INITIAL_ENERGY = [
    1: 100d, 2: 80d, 3: 85d, 4: 0d,
    5: 65d, 6: 70d, 7: 90d, 8: 100d
  ]

  // Keep this false for the current demonstration. Energy is still checked
  // before selecting a relay, but forwarding does not silently drain nodes.
  // Set it to true only when you want dynamic battery depletion.
  static final boolean DEBIT_DATA_ENERGY = false

  double alpha = 0.15d
  double gamma = 0.90d
  int episodes = 800
  int destination = 8

  private double initialEnergy = 100d
  private double residualEnergy = 100d

  Set<String> failedLinks = [] as Set<String>
  Set<Integer> failedNodes = [] as Set<Integer>

  final Map<Integer, List<Integer>> neighbors = [
    1: [2, 3],
    2: [1, 4, 5],
    3: [1, 5, 6],
    4: [2, 7],
    5: [2, 3, 7],
    6: [3, 7],
    7: [4, 5, 6, 8],
    8: [7]
  ]

  final Map<Integer, List<Double>> position = [
    1: [0d, 0d, -10d],
    2: [100d, 0d, -15d],
    3: [0d, 100d, -20d],
    4: [200d, 0d, -20d],
    5: [100d, 100d, -25d],
    6: [0d, 200d, -15d],
    7: [250d, 100d, -20d],
    8: [350d, 100d, -10d]
  ]

  private AgentID router
  private AgentID link
  private int address = -1

  // A state is the canonical set of permanent failed nodes and links. Thus
  // [failedNodes:[4], failedLinks:["2-5"]] always addresses the same table,
  // regardless of assignment order or input formatting.
  private Map<String, Map<Integer, Map<Integer, Double>>> qTablesByState = [:]
  private Map<String, Map<Integer, Set<Integer>>> bootstrappedByState = [:]
  private Map<Integer, Map> neighborCache = [:]

  private Map<Integer, Integer> installedNextHops = [:]
  private Map<Integer, Long> lastEnergyRequestAt = [:]
  private Map<Integer, List<Double>> pendingControlTxCost = [:]

  private long localAdvertisementSequence = 0L
  private long lastEnergyAdvertisementAt = 0L
  private double lastAdvertisedEnergy = Double.NaN
  private boolean refreshQueued = false

  private String debugStatus = 'Agent initialized; waiting for startup.'
  private String routesResult = 'Set rlroute.routes to a destination.'
  private String traceRouteResult = 'Set rlroute.traceRoute to a destination.'

  static final long NEIGHBOR_TTL_MS = 90000L
  // The topology has configured initial energy and the demonstration does not
  // deplete batteries. Keep the last reported energy usable after its age
  // exceeds NEIGHBOR_TTL_MS; otherwise every route disappears after 90 s if a
  // control advertisement is missed.
  static final boolean REQUIRE_FRESH_NEIGHBOR_ENERGY = false
  static final long NEIGHBOR_QUERY_MS = 5000L
  static final long ADVERTISEMENT_INTERVAL_MS = 5000L
  static final long MIN_ADVERT_GAP_MS = 2000L
  static final double ENERGY_CHANGE_THRESHOLD_J = 1.0d

  @Override
  List<Parameter> getParameterList() {
    allOf(Params)
  }

  boolean isReadOnly(Parameter p) {
    p == Params.qtable || p == Params.neighborInfo || p == Params.debug
  }

  private void setDebugStatus(String message) {
    debugStatus = "${new Date()} | ${message}"
  }

  String getDebug() {
    debugStatus
  }

  double getEnergy() {
    residualEnergy
  }

  void setEnergy(Object value) {
    if (!(value instanceof Number) ||
        !Double.isFinite(value as double) ||
        (value as double) < 0d) {
      throw new IllegalArgumentException(
        'energy must be a non-negative number of joules'
      )
    }

    residualEnergy = value as double

    setDebugStatus(
      "LOCAL ENERGY SET: node=${address}, energy=${residualEnergy} J"
    )

    log.info("EAFTR: node ${address} energy set to ${residualEnergy} J")

    refreshQValues()
    queueRefresh()

    lastAdvertisedEnergy = Double.NaN
    lastEnergyAdvertisementAt = 0L

    advertiseEnergyIfChanged(true)
  }

  String getNeighborInfo() {
    if (neighborCache.isEmpty()) {
      return 'No neighbour energy information available.'
    }

    neighborCache.keySet().sort().collect { int node ->
      Map n = neighborCache[node]
      long age = System.currentTimeMillis() - (n.time as long)

      "${node}: energy=${String.format('%.3f', n.energy as double)} J, age=${age} ms"
    }.join('\n')
  }

  String getQtable() {
    if (address < 1) return 'Agent is still starting.'

    Map<Integer, Map<Integer, Double>> qTables = currentQTables()
    StringBuilder out = new StringBuilder()
    out.append("EAFTR Q-table at node ${address}; state=${networkStateKey()}\n")
    out.append('Q[destination][nextHop]\n')

    qTables.keySet().sort().each { int target ->
      out.append("\nDestination ${target}\n")

      qTables[target].keySet().sort().each { int hop ->
        double value = qTables[target][hop] ?: 0d
        out.append(
          "  NextHop ${hop}: Q = ${String.format('%.2f', value)}\n"
        )
      }
    }

    out.toString()
  }

  String getRoutes() {
    routesResult
  }

  void setRoutes(int target) {
    routesResult = calculateRoute(target)
    log.info(routesResult)
  }

  String getTraceRoute() {
    traceRouteResult
  }

  void setTraceRoute(int target) {
    traceRouteResult = calculateTraceRoute(target)
    log.info(traceRouteResult)
  }

  private String calculateRoute(int target) {
    if (address < 1) return 'Agent is still starting.'
    if (!neighbors.containsKey(target)) return "Invalid destination ${target}."
    if (target == address) return "Already at destination ${target}."

    Map<Integer, Double> values = localTable(target)
    recalculateDestinationQValues(target, values)

    Integer next = bestLocalAction(target, values)

    if (next == null) {
      return "No feasible next hop from ${address} to ${target}."
    }

    "Node ${address} -> destination ${target}: nextHop=${next}, Q=${String.format('%.2f', values[next])}"
  }

  private String calculateTraceRoute(int target) {
    if (address < 1) return 'Agent is still starting.'
    if (!neighbors.containsKey(target)) return "Invalid destination ${target}."
    if (target == address) return "Route: ${address}"

    List<Integer> path = [address]
    Set<Integer> visited = [address] as Set<Integer>
    int current = address

    while (current != target && path.size() <= neighbors.size()) {
      Integer next

      if (current == address) {
        Map<Integer, Double> values = localTable(target)
        recalculateDestinationQValues(target, values)
        next = bestLocalAction(target, values, visited)
      } else {
        List<Integer> options = (neighbors[current] ?: []).findAll { int n ->
          !visited.contains(n) &&
          !failedNodes.contains(n) &&
          !failedLinks.contains(key(current, n)) &&
          relayEnergyFeasible(n, target) &&
          pathExists(n, visited, target)
        }

        if (options.isEmpty()) {
          return "No complete route. Partial route: ${path.join(' -> ')}"
        }

        options.sort { int a, int b ->
          int da = shortestDistance(a, target)
          int db = shortestDistance(b, target)
          da <=> db ?: a <=> b
        }

        next = options[0]
      }

      if (next == null || visited.contains(next)) {
        return "No complete route. Partial route: ${path.join(' -> ')}"
      }

      path.add(next)
      visited.add(next)
      current = next
    }

    if (current != target) {
      return "No complete route. Partial route: ${path.join(' -> ')}"
    }

    "Predicted route: ${path.join(' -> ')}"
  }

  @Override
  void startup() {
    router = agentForService(Services.ROUTING)
    link = agentForService(Services.LINK)

    if (link != null) subscribe topic(link)

    subscribe topic(Topics.DATAGRAM)

    add new WakerBehavior(1000, {
      def info = agentForService(Services.NODE_INFO)
      address = get(info, NodeInfoParam.address) as int

      setDebugStatus("STARTUP COMPLETE: node=${address}")

      // Do not use Groovy's ?: here: 0d is a valid configured energy, but
      // ?: treats zero as false and would incorrectly replace it with 100 J.
      initialEnergy = INITIAL_ENERGY.containsKey(address) ?
        INITIAL_ENERGY[address] : 100d
      residualEnergy = initialEnergy

      long now = System.currentTimeMillis()

      (neighbors[address] ?: []).each { int node ->
        neighborCache[node] = [
          energy: INITIAL_ENERGY.containsKey(node) ? INITIAL_ENERGY[node] : 100d,
          sequence: 0L,
          time: now,
          advertisedAt: now
        ]
      }

      initializeTables()
      refreshQValues()
      refreshPolicy()

      log.info(
        "EAFTR started at node ${address}, energy=${residualEnergy} J"
      )

      requestNeighborEnergy()
      advertiseEnergyIfChanged(true)

      scheduleNeighborRequests()
      scheduleAdvertisements()
    })
  }

  @Override
  void processMessage(Message msg) {
    if (address < 1) return

    if (msg instanceof DatagramNtf) {
      // Ignore datagrams overheard for other nodes.
      // Only process datagrams addressed to this node or broadcast.
      if (msg.to != address && msg.to != 0) return

      setDebugStatus(
        "DATAGRAM RECEIVED: node=${address}, from=${msg.from}, protocol=${msg.protocol}"
      )

      if (msg.protocol == energyProtocol) {
        handleEnergyDatagram(msg)
        return
      }

      double bytes = msg.data?.length ?: 0d

      debitDataEnergy(
        rxEnergy(bytes) + routingCost,
        'data reception and processing',
        true
      )

      log.info(
        "EAFTR: data received at node ${address} from ${msg.from}, bytes=${bytes}"
      )

      return
    }

    if (msg.sender != link) return

    if (msg instanceof DatagramTransmissionNtf) {
      List<Double> costs = pendingControlTxCost[msg.to]
      boolean isControl = costs != null && !costs.isEmpty()

      if (isControl) {
        costs.remove(0)

        if (costs.isEmpty()) {
          pendingControlTxCost.remove(msg.to)
        }

        log.info(
          "EAFTR: energy-control transmission to ${msg.to}; battery unchanged"
        )
      } else {
        debitDataEnergy(
          txEnergy(packetBytes) + routingCost,
          'data transmission and routing decision',
          true
        )

        log.info(
          "EAFTR: data transmission from node ${address} to ${msg.to}"
        )
      }

      return
    }

    if (msg instanceof LinkStatusNtf) {
      if ((neighbors[address] ?: []).contains(msg.to)) {
        log.info(
          "EAFTR: physical link notification ${address}-${msg.to}, up=${msg.up}; manual failure mode unchanged"
        )
      }

      return
    }
  }

  void setFailedLinks(Object values) {
    Collection items

    if (values == null) {
      items = []
    } else if (values instanceof Collection) {
      items = values as Collection
    } else {
      items = [values]
    }

    Set<String> configured = items.collect { item ->
      String edge = item.toString().trim().replaceAll('\\s+', '')
      def match = edge =~ /^(\d+)[-:,](\d+)$/

      if (match.matches()) {
        key(match[0][1] as int, match[0][2] as int)
      } else {
        throw new IllegalArgumentException("Invalid link '${item}'; use node-node, e.g. 2-5")
      }
    }.toSet() as Set<String>

    Set<String> unknown = configured.findAll { String edge ->
      List<String> endpoints = edge.split('-') as List<String>
      int left = endpoints[0] as int
      int right = endpoints[1] as int
      !(neighbors[left] ?: []).contains(right)
    } as Set<String>
    if (!unknown.isEmpty()) {
      throw new IllegalArgumentException("Not a topology link: ${unknown.sort()}")
    }

    failedLinks = configured

    log.info("EAFTR: manually configured failed links = ${failedLinks}")

    refreshQValues()
    queueRefresh()
  }

  void setFailedNodes(Object values) {
    Collection items = values == null ? [] :
      (values instanceof Collection ? values as Collection : [values])

    Set<Integer> configured = items.collect { it as int } as Set<Integer>
    Set<Integer> unknown = configured.findAll { !neighbors.containsKey(it) } as Set<Integer>
    if (!unknown.isEmpty()) {
      throw new IllegalArgumentException("Unknown node address(es): ${unknown.sort()}")
    }

    failedNodes = configured

    log.info("EAFTR: manually configured failed nodes = ${failedNodes}")

    refreshQValues()
    queueRefresh()
  }

  void setDestination(int value) {
    if (!neighbors.containsKey(value)) {
      throw new IllegalArgumentException('Unknown destination.')
    }

    destination = value
    queueRefresh()
  }

  void setEpisodes(int value) {
    if (value < 1) {
      throw new IllegalArgumentException('episodes must be positive')
    }

    episodes = value
    refreshQValues()
    queueRefresh()
  }

  private void initializeTables() {
    if (address < 1) return

    neighbors.keySet().each { int target ->
      if (target != address) localTable(target)
    }
  }

  private Map<Integer, Double> localTable(int target) {
    Map<Integer, Map<Integer, Double>> qTables = currentQTables()
    if (!qTables.containsKey(target)) {
      qTables[target] = (neighbors[address] ?: []).collectEntries {
        int hop -> [(hop): 0d]
      }
    }

    qTables[target]
  }

  private void refreshQValues() {
    if (address < 1) return

    initializeTables()

    Map<Integer, Map<Integer, Double>> qTables = currentQTables()
    qTables.keySet().each { int target ->
      recalculateDestinationQValues(target, qTables[target])
    }
  }

  private String networkStateKey() {
    String nodes = failedNodes.toList().sort().join(',')
    String links = failedLinks.toList().sort().join(',')
    "nodes=[${nodes}];links=[${links}]"
  }

  private Map<Integer, Map<Integer, Double>> currentQTables() {
    String state = networkStateKey()
    if (!qTablesByState.containsKey(state)) qTablesByState[state] = [:]
    qTablesByState[state]
  }

  private Map<Integer, Set<Integer>> currentBootstrappedActions() {
    String state = networkStateKey()
    if (!bootstrappedByState.containsKey(state)) bootstrappedByState[state] = [:]
    bootstrappedByState[state]
  }

  private void recalculateDestinationQValues(
    int target,
    Map<Integer, Double> values
  ) {
    Map<Integer, Set<Integer>> bootstrappedActions = currentBootstrappedActions()
    Set<Integer> trained = bootstrappedActions[target]

    if (trained == null) {
      trained = [] as Set<Integer>
      bootstrappedActions[target] = trained
    }

    (neighbors[address] ?: []).each { int hop ->
      if (!isActionFeasible(hop, target)) {
        values[hop] = -10d
        return
      }

      double r = reward(address, hop, target)

      // A hop may have been marked -10 while a failure/low-energy condition
      // was active. When that condition clears, bootstrap it again instead of
      // only making one tiny alpha update from the failure penalty.
      if (!trained.contains(hop) || (values[hop] ?: 0d) <= -10d) {
        values[hop] = 0d
        episodes.times {
          values[hop] += alpha * (r - values[hop])
        }

        trained.add(hop)
      } else {
        values[hop] += alpha * (r - values[hop])
      }
    }
  }

  private Integer bestLocalAction(
    int target,
    Map<Integer, Double> values,
    Set<Integer> visited = [] as Set<Integer>
  ) {
    List<Integer> options = actions(target, visited)

    if (options.isEmpty()) return null

    double best = options.collect {
      values[it] ?: 0d
    }.max() as double

    List<Integer> ties = options.findAll {
      Math.abs((values[it] ?: 0d) - best) < 1e-9
    }

    ties.sort { int a, int b ->
      double ra = reward(address, a, target)
      double rb = reward(address, b, target)

      rb <=> ra ?: a <=> b
    }

    ties[0]
  }

  private List<Integer> actions(
    int target,
    Set<Integer> visited
  ) {
    if (failedNodes.contains(address) || !canTransmitLocal()) {
      return []
    }

    (neighbors[address] ?: []).findAll { int next ->
      !visited.contains(next) &&
      isActionFeasible(next, target) &&
      pathExists(next, visited, target)
    }
  }

  private boolean isActionFeasible(int node, int target) {
    // This method is used both by route selection and Q-value refresh. Keep
    // the local transmit test here so a zero-energy source cannot retain old
    // positive Q-values while actions() correctly reports no route.
    if (failedNodes.contains(address) || !canTransmitLocal()) return false
    if (failedNodes.contains(node)) return false
    if (failedNodes.contains(target)) return false
    if (failedLinks.contains(key(address, node))) return false
    if (!(neighbors[address] ?: []).contains(node)) return false

    int currentDistance = shortestDistance(address, target)
    int nextDistance = shortestDistance(node, target)

    if (currentDistance == Integer.MAX_VALUE ||
        nextDistance == Integer.MAX_VALUE ||
        nextDistance >= currentDistance) {
      return false
    }

    double energy = freshNeighborEnergy(node)
    if (energy < 0d) return false

    double required = node == target
      ? rxEnergy() + energyReserve
      : rxEnergy() + txEnergy() + routingCost + energyReserve

    energy >= required
  }

  private double reward(int from, int to, int target) {
    if (to == target) return 100d

    double battery = 10d * freshNeighborEnergy(to) /
      Math.max(1d, INITIAL_ENERGY[to] ?: 100d)

    double linkQuality = 5d * Math.exp(-distance(from, to) / 500d)

    double progress =
      (distance(from, target) - distance(to, target)) / 100d

    battery + linkQuality + progress -
      2d * (txEnergy() + rxEnergy() + routingCost) - 1d
  }

  private boolean pathExists(
    int start,
    Set<Integer> visited,
    int target
  ) {
    def queue = new ArrayDeque<Integer>()
    Set<Integer> seen = [] as Set<Integer>

    // This is a modelled end-to-end feasibility search. A node must have
    // enough energy to receive the current packet and, unless it is the
    // destination, make one forwarding transmission. This makes an initial
    // energy of 0 on an intermediate node remove it from predicted routes.
    if (!relayEnergyFeasible(start, target)) return false

    queue.add(start)
    seen.add(start)

    while (!queue.isEmpty()) {
      int current = queue.removeFirst()

      if (current == target) return true

      (neighbors[current] ?: []).each { int next ->
        if (!seen.contains(next) &&
            !visited.contains(next) &&
            !failedNodes.contains(next) &&
            !failedLinks.contains(key(current, next)) &&
            relayEnergyFeasible(next, target)) {
          seen.add(next)
          queue.add(next)
        }
      }
    }

    false
  }

  private boolean relayEnergyFeasible(int node, int target) {
    if (failedNodes.contains(node)) return false

    double energy = node == address ? residualEnergy : freshNeighborEnergy(node)
    if (energy < 0d) return false

    double required = node == target
      ? rxEnergy() + energyReserve
      : rxEnergy() + txEnergy() + routingCost + energyReserve

    energy >= required
  }

  private int shortestDistance(int start, int target) {
    if (start == target) return 0

    def queue = new ArrayDeque<List<Integer>>()
    Set<Integer> seen = [] as Set<Integer>

    queue.add([start, 0])
    seen.add(start)

    while (!queue.isEmpty()) {
      List<Integer> item = queue.removeFirst()
      int current = item[0] as int
      int hops = item[1] as int

      if (current == target) return hops

      (neighbors[current] ?: []).each { int next ->
        if (!seen.contains(next) &&
            !failedNodes.contains(next) &&
            !failedLinks.contains(key(current, next))) {
          seen.add(next)
          queue.add([next, hops + 1])
        }
      }
    }

    Integer.MAX_VALUE
  }

  private double freshNeighborEnergy(int node) {
    Map snapshot = neighborCache[node]

    if (snapshot == null) {
      return INITIAL_ENERGY.containsKey(node) ? INITIAL_ENERGY[node] : -1d
    }

    if (REQUIRE_FRESH_NEIGHBOR_ENERGY &&
        System.currentTimeMillis() - (snapshot.time as long) > NEIGHBOR_TTL_MS) {
      return -1d
    }

    snapshot.energy as double
  }

  private double distance(int a, int b) {
    double total = 0d

    for (int i = 0; i < 3; i++) {
      double d = position[a][i] - position[b][i]
      total += d * d
    }

    Math.sqrt(total)
  }

  private static double packetDuration(double bytes = packetBytes) {
    bytes * 8d / dataRate
  }

  private static double txEnergy(double bytes = packetBytes) {
    txPower * packetDuration(bytes)
  }

  private static double rxEnergy(double bytes = packetBytes) {
    rxPower * packetDuration(bytes)
  }

  private boolean canTransmitLocal() {
    residualEnergy >= routingCost + txEnergy() + energyReserve
  }

  private void debitEnergy(
    double amount,
    String reason,
    boolean advertise = false
  ) {
    if (amount <= 0d) return

    residualEnergy = Math.max(0d, residualEnergy - amount)

    log.info(
      "EAFTR: node ${address} energy=${String.format('%.3f', residualEnergy)} J after ${reason}"
    )

    refreshQValues()
    queueRefresh()

    if (advertise) advertiseEnergyIfChanged()
  }

  private void debitDataEnergy(double amount, String reason, boolean advertise = false) {
    if (!DEBIT_DATA_ENERGY) {
      log.info("EAFTR: node ${address} would use ${String.format('%.3f', amount)} J for ${reason}; accounting disabled")
      return
    }

    debitEnergy(amount, reason, advertise)
  }

  private void recordNeighborAdvertisement(
    int node,
    double joules,
    long sequence,
    long timestamp
  ) {
    if (!(neighbors[address] ?: []).contains(node)) {
      setDebugStatus("REJECTED ENERGY UPDATE: ${node} is not a neighbour")
      log.warning("EAFTR: energy update from non-neighbour ${node}")
      return
    }

    if (joules < 0d || !Double.isFinite(joules)) {
      setDebugStatus("REJECTED ENERGY UPDATE: invalid value from ${node}")
      log.warning("EAFTR: invalid energy value from node ${node}")
      return
    }

    long receivedAt = System.currentTimeMillis()

    neighborCache[node] = [
      energy: joules,
      sequence: sequence,
      time: receivedAt,
      advertisedAt: timestamp
    ]

    setDebugStatus(
      "NEIGHBOUR UPDATED: node=${address}, neighbour=${node}, energy=${joules} J"
    )

    log.info(
      "EAFTR: UPDATED neighbour ${node} energy=${String.format('%.3f', joules)} J"
    )

    refreshQValues()
    queueRefresh()
  }

  private void handleEnergyDatagram(DatagramNtf msg) {
    String payload

    try {
      payload = new String(msg.data as byte[], 'UTF-8').trim()
    } catch (Exception ignored) {
      setDebugStatus("ERROR: could not decode energy message from ${msg.from}")
      log.warning('EAFTR: could not decode energy message')
      return
    }

    setDebugStatus(
      "ENERGY PAYLOAD RECEIVED: from=${msg.from}, payload=${payload}"
    )

    log.info(
      "EAFTR: energy payload received from ${msg.from}: ${payload}"
    )

    String[] parts = payload.split(':')

    if (parts.size() == 3 &&
        parts[0] == 'EAFTR' &&
        parts[1] == 'ENERGY_REQ') {

      int sender = msg.from as int

      if ((neighbors[address] ?: []).contains(sender)) {
        long sequence = ++localAdvertisementSequence

        setDebugStatus(
          "ENERGY REQUEST RECEIVED from ${sender}; replying with ${residualEnergy} J"
        )

        sendEnergyControl(
          sender,
          "EAFTR:ENERGY_RSP:${sequence}:${residualEnergy}:${System.currentTimeMillis()}",
          msg.sender as AgentID
        )
      }

      return
    }

    if (parts.size() == 5 &&
        parts[0] == 'EAFTR' &&
        parts[1] == 'ENERGY_RSP') {

      try {
        recordNeighborAdvertisement(
          msg.from as int,
          parts[3] as double,
          parts[2] as long,
          parts[4] as long
        )
      } catch (Exception ignored) {
        setDebugStatus("ERROR: invalid energy response from ${msg.from}")
        log.warning(
          "EAFTR: invalid energy response from ${msg.from}"
        )
      }

      return
    }

    setDebugStatus(
      "UNKNOWN ENERGY MESSAGE from ${msg.from}: ${payload}"
    )

    log.warning(
      "EAFTR: unknown energy message from ${msg.from}"
    )
  }

  private void sendEnergyControl(
    int target,
    String payload,
    AgentID preferredProvider = null
  ) {
    def datagram = preferredProvider ?: agentForService(Services.DATAGRAM)

    if (datagram == null) {
      setDebugStatus("SEND FAILED: DATAGRAM service unavailable")
      log.warning('EAFTR: DATAGRAM service unavailable')
      return
    }

    byte[] data = payload.getBytes('UTF-8')

    List<Double> costs = pendingControlTxCost[target]

    if (costs == null) {
      costs = []
      pendingControlTxCost[target] = costs
    }

    costs.add(0d)

    setDebugStatus(
      "SENDING ENERGY CONTROL: node=${address}, to=${target}, payload=${payload}"
    )

    send new DatagramReq(
      recipient: datagram,
      to: target,
      protocol: energyProtocol,
      reliability: false,
      data: data
    )

    log.info(
      "EAFTR: sending energy-control datagram to ${target}; battery unchanged"
    )
  }

  private void requestNeighborEnergy() {
    if (link == null || address < 1) return

    long now = System.currentTimeMillis()
    long requestId = ++localAdvertisementSequence

    (neighbors[address] ?: []).each { int node ->
      Long previous = lastEnergyRequestAt[node]

      if (!failedNodes.contains(node) &&
          !failedLinks.contains(key(address, node)) &&
          (previous == null ||
           now - previous >= NEIGHBOR_QUERY_MS)) {

        lastEnergyRequestAt[node] = now

        log.info(
          "EAFTR: requesting energy from node ${node}"
        )

        sendEnergyControl(
          node,
          "EAFTR:ENERGY_REQ:${requestId}"
        )
      }
    }
  }

  private void advertiseEnergyIfChanged(boolean force = false) {
    if (link == null || address < 1) {
      setDebugStatus(
        "ADVERTISEMENT SKIPPED: link unavailable or address not initialized"
      )
      return
    }

    long now = System.currentTimeMillis()

    boolean enoughTime =
      now - lastEnergyAdvertisementAt >= MIN_ADVERT_GAP_MS

    boolean enoughChange =
      Double.isNaN(lastAdvertisedEnergy) ||
      Math.abs(lastAdvertisedEnergy - residualEnergy) >=
      ENERGY_CHANGE_THRESHOLD_J

    if (!force && (!enoughTime || !enoughChange)) return

    lastEnergyAdvertisementAt = now
    lastAdvertisedEnergy = residualEnergy

    long sequence = ++localAdvertisementSequence

    (neighbors[address] ?: []).each { int node ->
      if (!failedNodes.contains(node) &&
          !failedLinks.contains(key(address, node))) {

        log.info(
          "EAFTR: advertising energy=${residualEnergy} J to node ${node}"
        )

        sendEnergyControl(
          node,
          "EAFTR:ENERGY_RSP:${sequence}:${residualEnergy}:${now}"
        )
      }
    }
  }

  private void scheduleNeighborRequests() {
    add new WakerBehavior(NEIGHBOR_QUERY_MS, {
      if (address > 0 && !failedNodes.contains(address)) {
        requestNeighborEnergy()
      }

      scheduleNeighborRequests()
    })
  }

  private void scheduleAdvertisements() {
    add new WakerBehavior(ADVERTISEMENT_INTERVAL_MS, {
      if (address > 0 && !failedNodes.contains(address)) {
        advertiseEnergyIfChanged(true)
      }

      scheduleAdvertisements()
    })
  }

  private void queueRefresh() {
    if (address < 1 || refreshQueued) return

    refreshQueued = true

    add new WakerBehavior(250, {
      refreshQueued = false
      refreshPolicy()
    })
  }

  void refreshPolicy() {
    if (router == null || address < 1) return

    installedNextHops.keySet().findAll {
      failedNodes.contains(it) || failedNodes.contains(address)
    }.toList().each { int target ->
      removeInstalledRoute(target)
    }

    if (failedNodes.contains(address)) return

    neighbors.keySet().findAll {
      it != address && !failedNodes.contains(it)
    }.sort().each { int target ->
      installRoute(target)
    }
  }

  private void installRoute(int target) {
    Map<Integer, Double> values = localTable(target)

    recalculateDestinationQValues(target, values)

    Integer next = bestLocalAction(target, values)

    if (next == null) {
      removeInstalledRoute(target)

      log.warning(
        "EAFTR: no feasible route from ${address} to ${target}"
      )

      return
    }

    if (installedNextHops[target] == next) return

    if (installedNextHops.containsKey(target)) {
      removeInstalledRoute(target)
    }

    def request = EditRouteReq.newRoute()

    request.to = target
    request.nextHop = next
    request.link = link

    int routeHops = shortestDistance(address, target)
    request.hops = routeHops == Integer.MAX_VALUE ? 1 : routeHops

    request.reliability = true
    request.metric = (1000d + values[next]) as float
    request.auto = false
    request.recipient = router

    send(request)

    installedNextHops[target] = next

    log.info(
      "EAFTR: route ${address} -> ${target} via ${next}"
    )
  }

  private void removeInstalledRoute(int target) {
    Integer oldNext = installedNextHops.remove(target)

    if (oldNext == null) return

    def request = EditRouteReq.deleteRoute()

    request.to = target
    request.nextHop = oldNext
    request.link = link
    request.recipient = router

    send(request)
  }

  private static String key(int a, int b) {
    "${Math.min(a, b)}-${Math.max(a, b)}"
  }
}
