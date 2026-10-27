import org.arl.fjage.*
import org.arl.fjage.param.Parameter
import org.arl.unet.*
import org.arl.unet.link.LinkStatusNtf
import org.arl.unet.net.*
import org.arl.unet.nodeinfo.NodeInfoParam
import java.util.Random

/**
 * Live UnetStack implementation of EAFTR. One instance runs at every node,
 * learns a local Q-learning policy and adds the selected next hop to Router.
 */
class EnergyAwareRoutingAgent extends UnetAgent {
  enum Params implements Parameter { source, destination, energy, failedLinks, failedNodes, episodes, qtable }
  static final String title = 'EAFTR Q-learning routing'
  static final String description = 'Energy-aware, fault-tolerant routing policy'
  double alpha=0.15d, gamma=0.90d, epsilonStart=0.30d, epsilonEnd=0.03d
  // Keep the live policy's feasibility and reward model identical to the
  // offline experiment: 128 bytes at 10 kbps, 10 W TX and 2 W RX.
  static final double packetBytes=128d, dataRate=10000d, txPower=10d, rxPower=2d
  static final double routingCost=0.05d, energyReserve=0.10d
  // 800 episodes keeps a full 8 x 8 route-table refresh responsive in
  // real-time while still providing epsilon-greedy Q-learning convergence.
  int episodes=800, source=1, destination=8
  List<Double> energy=[100d,40d,90d,70d,20d,35d,60d,100d]
  Set<String> failedLinks=[] as Set<String>
  Set<Integer> failedNodes=[] as Set<Integer>

  final Map<Integer,List<Integer>> neighbors=[
    1:[2,3], 2:[1,4,5], 3:[1,5,6], 4:[2,7],
    5:[2,3,7], 6:[3,7], 7:[4,5,6,8], 8:[7]]
  final Map<Integer,List<Double>> position=[
    1:[0d,0d,-10d], 2:[100d,0d,-15d], 3:[0d,100d,-20d],
    4:[200d,0d,-20d], 5:[100d,100d,-25d], 6:[0d,200d,-15d],
    7:[250d,100d,-20d], 8:[350d,100d,-10d]]
  private AgentID router, link
  private int address=-1
  private Map q=[:]
  // One learned action-value map per destination, exposed by rlroute.qtable.
  private Map<Integer,Map> qTables=[:]
  private Map<Integer,Integer> installedNextHops=[:]
  private boolean refreshQueued=false

  @Override List<Parameter> getParameterList() { allOf(Params) }
  // Unet 7.0.0 does not declare this hook on UnetAgent, so this intentionally
  // has no @Override annotation. Newer shells still discover it as a property
  // metadata method.
  boolean isReadOnly(Parameter parameter) {
    parameter == Params.qtable
  }
  String getQtable() {
    if (address < 1 || qTables.isEmpty()) return 'Q-tables are still training. Wait a few seconds and try again.'
    StringBuilder out=new StringBuilder("EAFTR Q-tables at node ${address}\n")
    qTables.keySet().sort().each { int target ->
      out.append("\nDestination ${target}\n")
      neighbors.keySet().sort().each { int from ->
        List<String> values=neighbors[from].collect { int to ->
          "${to}:${String.format('%.2f', qTables[target][from][to])}"
        }
        out.append("  ${from} -> ").append(values.join(', ')).append('\n')
      }
    }
    out.toString()
  }
  @Override void startup() {
    router=agentForService(Services.ROUTING); link=agentForService(Services.LINK)
    if (link != null) subscribe topic(link)
    add new WakerBehavior(1000, {
      def info=agentForService(Services.NODE_INFO)
      address=get(info,NodeInfoParam.address) as int
      refreshPolicy()
    })
  }
  @Override void processMessage(Message msg) {
    if (address<1 || msg.sender != link) return
    // ECLink publishes one notification for every datagram it actually sends,
    // including retransmissions. Account for the routing decision and the
    // acoustic transmission at the local node before refreshing the policy.
    if (msg instanceof DatagramTransmissionNtf) {
      debitLocalEnergy(routingCost+txEnergy())
      return
    }
    if (!(msg instanceof LinkStatusNtf)) return
    if (!neighbors[address]?.contains(msg.to)) return
    String edge=key(address,msg.to)
    if (msg.up) failedLinks.remove(edge); else failedLinks.add(edge)
    queueRefresh()
  }
  void setEnergy(List values) {
    if (values==null || values.size()!=8 || values.any { it==null || (it as double)<0d })
      throw new IllegalArgumentException('energy must contain 8 non-negative joule values')
    this.@energy=values.collect { it as double }; queueRefresh()
  }
  void setFailedLinks(Collection values) {
    this.@failedLinks=(values ?: []).collect { it.toString() } as Set<String>; queueRefresh()
  }
  void setFailedNodes(Collection values) {
    this.@failedNodes=(values ?: []).collect { it as int } as Set<Integer>; queueRefresh()
  }
  void setDestination(int value) {
    if (!neighbors.containsKey(value)) throw new IllegalArgumentException('unknown destination')
    this.@destination=value; queueRefresh()
  }
  void setEpisodes(int value) {
    if (value<1) throw new IllegalArgumentException('episodes must be positive')
    this.@episodes=value; queueRefresh()
  }
  private void queueRefresh() {
    if (address<1 || refreshQueued) return
    refreshQueued=true
    add new WakerBehavior(250, { refreshQueued=false; refreshPolicy() })
  }
  private void debitLocalEnergy(double amount) {
    int index=address-1
    if (index<0 || index>=energy.size()) return
    energy[index]=Math.max(0d,energy[index]-amount)
    log.info(String.format('EAFTR: node %d energy %.3f J',address,energy[index]))
    queueRefresh()
  }

  /**
   * Installs a route to every other address in the known topology. This makes
   * the agent a real Router policy, not a N0-to-N7-only demonstration.
   */
  void refreshPolicy() {
    if (router==null || address<1) return
    // Remove routes that now point at a declared failed destination. If this
    // node itself is failed, it must not retain any EAFTR forwarding policy.
    installedNextHops.keySet().findAll { failedNodes.contains(it) || failedNodes.contains(address) }
      .toList().each { int target -> removeInstalledRoute(target) }
    if (failedNodes.contains(address)) return
    neighbors.keySet().findAll { it!=address && !failedNodes.contains(it) }
      .sort().each { int target -> installRoute(target) }
  }
  private void installRoute(int target) {
    q=train(target)
    qTables[target]=q
    Integer next=bestLocalAction(target)
    if (next==null) {
      // A route learned before a failure/energy change must not survive as a
      // stale Router entry. RDP routes are untouched.
      removeInstalledRoute(target)
      log.warning("EAFTR: no feasible path from ${address} to ${target}")
      return
    }
    // Delete previous policy route for this destination but retain RDP fallback routes.
    removeInstalledRoute(target)
    def request=EditRouteReq.newRoute()
    request.to=target; request.nextHop=next; request.link=link; request.hops=1
    request.reliability=true; request.metric=(1000d+q[address][next]) as float
    request.auto=false; request.recipient=router; send(request)
    installedNextHops[target]=next
    log.info("EAFTR: route ${address} -> ${target} via ${next}")
  }
  private void removeInstalledRoute(int target) {
    Integer oldNext=installedNextHops.remove(target)
    if (oldNext == null) return
    def remove=EditRouteReq.deleteRoute()
    remove.to=target; remove.nextHop=oldNext; remove.link=link
    remove.recipient=router; send(remove)
  }

  private Map train(int target) {
    def values=[:]
    neighbors.each { int node, List<Integer> list ->
      values[node]=[:]; list.each { int next -> values[node][next]=0d }
    }
    // Every node uses the same training sequence for a target, so the network
    // forms one consistent destination-rooted policy rather than independent
    // local policies that could point back to one another.
    Random random=new Random(10000L+target)
    List<Integer> starts=neighbors.keySet().findAll { !failedNodes.contains(it) }.sort()
    episodes.times { int episode ->
      int current=starts[episode % starts.size()]
      def visited=[current] as Set<Integer>
      double epsilon=epsilonStart-(epsilonStart-epsilonEnd)*episode/Math.max(1d,episodes-1d)
      for (int hop=0; hop<7 && current!=target; hop++) {
        List<Integer> options=actions(current,visited,target)
        if (!options) break
        int next=random.nextDouble()<epsilon ? options[random.nextInt(options.size())] : maxQ(current,options,values)
        def after=new LinkedHashSet<Integer>(visited); after.add(next)
        List<Integer> future=next==target ? [] : actions(next,after,target)
        double bestFuture=future ? future.collect { values[next][it] }.max() as double : 0d
        values[current][next]+=alpha*(reward(current,next,target)+gamma*bestFuture-values[current][next])
        current=next; visited.add(next)
      }
    }
    values
  }
  private Integer bestLocalAction(int target) {
    List<Integer> options=actions(address,[] as Set<Integer>,target)
    options ? maxQ(address,options,q) : null
  }
  private int maxQ(int from, List<Integer> options, Map values) {
    double high=options.collect { values[from][it] }.max() as double
    options.find { Math.abs(values[from][it]-high)<1e-9 }
  }
  // Filters failures, energy-ineligible relays, loops and routing voids.
  private List<Integer> actions(int from, Set<Integer> visited, int target) {
    if (failedNodes.contains(from) || !canTransmit(from)) return []
    int currentHops=hopDistance(from,target)
    if (currentHops == Integer.MAX_VALUE) return []
    neighbors[from].findAll { int to ->
      !visited.contains(to) && !failedNodes.contains(to) && !failedLinks.contains(key(from,to)) &&
        canReceive(to,target) && pathExists(to,visited,target) &&
        hopDistance(to,target) < currentHops
    }
  }
  // Restrict forwarding to a neighbour with fewer feasible hops to the
  // destination. This preserves Q-learning's choice among candidates while
  // guaranteeing that the installed Router policy cannot form a loop.
  private int hopDistance(int start, int target) {
    if (start == target) return 0
    def queue=new ArrayDeque<Integer>(); def distance=[:]
    queue.add(start); distance[start]=0
    while (!queue.isEmpty()) {
      int current=queue.removeFirst()
      if (current == target) return distance[current] as int
      if (!canTransmit(current)) continue
      neighbors[current].each { int next ->
        if (distance[next] == null && !failedNodes.contains(next) &&
            !failedLinks.contains(key(current,next)) && canReceive(next,target)) {
          distance[next]=(distance[current] as int)+1
          queue.add(next)
        }
      }
    }
    Integer.MAX_VALUE
  }
  private boolean pathExists(int start, Set<Integer> visited, int target) {
    def queue=new ArrayDeque<Integer>(); def seen=new LinkedHashSet<Integer>()
    queue.add(start); seen.add(start)
    while (!queue.isEmpty()) {
      int current=queue.removeFirst()
      if (current==target) return true
      if (!canTransmit(current)) continue
      neighbors[current].each { int next ->
        if (!seen.contains(next) && !visited.contains(next) && !failedNodes.contains(next) &&
            !failedLinks.contains(key(current,next)) && canReceive(next,target)) { seen.add(next); queue.add(next) }
      }
    }
    false
  }
  private double reward(int from, int to, int target) {
    if (to==target) return 100d
    double battery=10d*energyAt(to)/100d
    double quality=5d*Math.exp(-distance(from,to)/500d)
    double progress=(distance(from,target)-distance(to,target))/100d
    battery+quality+progress-2d*(txEnergy()+rxEnergy()+routingCost)-1d
  }
  private boolean canTransmit(int node) { energyAt(node)>=routingCost+txEnergy() }
  private boolean canReceive(int node, int target) {
    energyAt(node)>=(node==target ? rxEnergy()+energyReserve : rxEnergy()+routingCost+txEnergy()+energyReserve)
  }
  private static double packetDuration() { packetBytes*8d/dataRate }
  private static double txEnergy() { txPower*packetDuration() }
  private static double rxEnergy() { rxPower*packetDuration() }
  private double energyAt(int node) { node>=1 && node<=energy.size() ? energy[node-1] : 0d }
  private double distance(int a, int b) {
    Math.sqrt((0..<3).collect { int i -> double d=position[a][i]-position[b][i]; d*d }.sum() as double)
  }
  private static String key(int a, int b) { "${Math.min(a,b)}-${Math.max(a,b)}" }
}
