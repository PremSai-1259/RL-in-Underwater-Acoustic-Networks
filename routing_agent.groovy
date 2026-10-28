#!/usr/bin/env groovy
/*
 * EAFTR: Energy-Aware Fault-Tolerant Routing for Underwater Acoustic Networks.
 * Self-contained implementation of Progress-215_233_245.pdf.
 * Run: groovy routing_agent.groovy [packets]
 */
import java.util.Random

class Network {
  final int size = 8, source = 0, destination = 7
  final List<List<Integer>> neighbours = [
    [1,2], [0,3,4], [0,4,5], [1,6], [1,2,6], [2,6], [3,4,5,7], [6]]
  final List<List<Double>> position = [
    [0d,0d,-10d], [100d,0d,-15d], [0d,100d,-20d], [200d,0d,-20d],
    [100d,100d,-25d], [0d,200d,-15d], [250d,100d,-20d], [350d,100d,-10d]]
  final List<Double> initialEnergy = [100d,40d,90d,70d,20d,35d,60d,100d]
  String edge(int a, int b) { "${Math.min(a,b)}-${Math.max(a,b)}" }
  double distance(int a, int b) {
    Math.sqrt((0..<3).collect { int i -> double x=position[a][i]-position[b][i]; x*x }.sum() as double)
  }
  double quality(int a, int b) { Math.exp(-distance(a,b)/500d) }
}

class Parameters {
  final double capacity=100d, dataRate=10000d, txPower=10d, rxPower=2d
  final int bytes=128, episodes=800
  final double routingCost=0.05d, margin=0.10d, soundSpeed=1500d
  final double alpha=0.15d, gamma=0.90d, epsilonStart=0.30d, epsilonEnd=0.03d
  double getDuration() { bytes*8d/dataRate }
  double getTx() { txPower*duration }
  double getRx() { rxPower*duration }
  double getRelayThreshold() { rx+routingCost+tx+margin }
}

class Scenario {
  String name; Set<String> links; Set<Integer> nodes; List<Double> energy
  Scenario(String n, Set<String> l, Set<Integer> f, List<Double> e) {
    name=n; links=new LinkedHashSet(l); nodes=new LinkedHashSet(f); energy=e.collect { it as double }
  }
}

class Metrics {
  int offered, delivered, failedHops, decisions, dataHops
  int lifetimePackets = -1
  // elapsed includes failed attempts for throughput; deliveryDelay contains
  // only packets that actually reached the destination.
  double elapsed, deliveryDelay, energy
  List<Integer> hops=[]
  double pdr() { offered ? 100d*delivered/offered : 0d }
  double loss() { offered ? 100d*(offered-delivered)/offered : 0d }
  double meanDelay() { delivered ? deliveryDelay/delivered : 0d }
  double throughput(int bytes) { elapsed ? delivered*bytes*8d/elapsed : 0d }
  double meanHops() { hops ? hops.sum()/hops.size() : 0d }
}

class Policy {
  String name
  Network net; Parameters p
  List<List<Double>> q
  Policy(String n, Network network, Parameters parameters) { name=n; net=network; p=parameters }
  List<List<Double>> table() { (0..<net.size).collect { (0..<net.size).collect { 0d } } }
  boolean receiveOK(int node, List<Double> e, Set<Integer> failed) {
    if (node < 0 || node >= net.size || failed.contains(node)) return false
    e[node] >= (node == net.destination ? p.rx+p.margin : p.relayThreshold)
  }
  boolean transmitOK(int node, List<Double> e, Set<Integer> failed) {
    !failed.contains(node) && e[node] >= p.routingCost+p.tx
  }
  boolean pathExists(int start, Set<Integer> visited, List<Double> e, Set<String> failedLinks,
                     Set<String> blocked, Set<Integer> failedNodes) {
    if (failedNodes.contains(start)) return false
    def queue=new ArrayDeque<Integer>(); def seen=new LinkedHashSet<Integer>()
    queue.add(start); seen.add(start)
    while (!queue.isEmpty()) {
      int current=queue.removeFirst()
      if (current == net.destination) return true
      if (!transmitOK(current,e,failedNodes)) continue
      net.neighbours[current].each { int next ->
        String edge=net.edge(current,next)
        if (!seen.contains(next) && !visited.contains(next) && !failedNodes.contains(next) &&
            !failedLinks.contains(edge) && !blocked.contains(edge) && receiveOK(next,e,failedNodes)) {
          seen.add(next); queue.add(next)
        }
      }
    }
    false
  }
  List<Integer> actions(int current, Set<Integer> visited, List<Double> e, Set<String> failedLinks,
                        Set<String> blocked, Set<Integer> failedNodes) {
    if (!transmitOK(current,e,failedNodes)) return []
    net.neighbours[current].findAll { int next ->
      String edge=net.edge(current,next)
      !visited.contains(next) && !failedNodes.contains(next) && !failedLinks.contains(edge) &&
        !blocked.contains(edge) && receiveOK(next,e,failedNodes) &&
        pathExists(next,visited,e,failedLinks,blocked,failedNodes)
    }
  }
  double reward(int from, int to, List<Double> e) {
    if (to == net.destination) return 100d
    double battery=10d*e[to]/p.capacity
    double reliability=5d*net.quality(from,to)
    double progress=(net.distance(from,net.destination)-net.distance(to,net.destination))/100d
    battery+reliability+progress-2d*(p.tx+p.rx+p.routingCost)-1d
  }
  int best(int current, List<Integer> options) {
    double high=options.collect { q[current][it] }.max() as double
    options.find { Math.abs(q[current][it]-high) < 1e-10 }
  }
  void train(List<Double> e, Set<String> failedLinks, Set<Integer> failedNodes) {
    q=table(); Random random=new Random(42L)
    p.episodes.times { int episode ->
      int current=net.source; def visited=[current] as Set<Integer>
      double epsilon=p.epsilonStart-(p.epsilonStart-p.epsilonEnd)*episode/Math.max(1d,p.episodes-1d)
      for (int hop=0; hop<net.size-1 && current!=net.destination; hop++) {
        List<Integer> options=actions(current,visited,e,failedLinks,[] as Set<String>,failedNodes)
        if (!options) break
        int next=random.nextDouble()<epsilon ? options[random.nextInt(options.size())] : best(current,options)
        def after=new LinkedHashSet<Integer>(visited); after.add(next)
        List<Integer> future=next==net.destination ? [] : actions(next,after,e,failedLinks,[] as Set<String>,failedNodes)
        double futureValue=future ? future.collect { q[next][it] }.max() as double : 0d
        q[current][next] += p.alpha*(reward(current,next,e)+p.gamma*futureValue-q[current][next])
        current=next; visited.add(next)
      }
    }
  }
  int choose(int current, List<Integer> options, List<Double> e) { best(current,options) }
}

class EnergyGreedy extends Policy {
  EnergyGreedy(Network n, Parameters p) { super('Energy-greedy baseline',n,p) }
  void train(List<Double> e, Set<String> l, Set<Integer> n) {}
  int choose(int current, List<Integer> options, List<Double> e) {
    options.max { int a, int b ->
      int value=Double.compare(e[a],e[b])
      value ? value : Double.compare(net.quality(current,a),net.quality(current,b))
    }
  }
}
class Geographic extends Policy {
  Geographic(Network n, Parameters p) { super('Geographic baseline',n,p) }
  void train(List<Double> e, Set<String> l, Set<Integer> n) {}
  int choose(int current, List<Integer> options, List<Double> e) {
    options.min { int a, int b -> Double.compare(net.distance(a,net.destination),net.distance(b,net.destination)) }
  }
}

class Simulation {
  final Network net=new Network(); final Parameters p=new Parameters()
  Metrics execute(Policy policy, Scenario scenario, int count, long seed) {
    List<Double> e=scenario.energy.collect { it as double }
    policy.train(e,scenario.links,scenario.nodes)
    Metrics m=new Metrics(); Random channel=new Random(seed)
    count.times {
      m.offered++; int current=net.source; int hops=0; double packetDelay=0d
      def visited=[current] as Set<Integer>; def blocked=[] as Set<String>
      if (m.lifetimePackets < 0 && !policy.pathExists(net.source, visited, e,
          scenario.links, blocked, scenario.nodes)) {
        m.lifetimePackets=m.offered-1
      }
      while (current!=net.destination && hops<net.size-1) {
        List<Integer> choices=policy.actions(current,visited,e,scenario.links,blocked,scenario.nodes)
        if (!choices) break
        int next=policy.choose(current,choices,e); String edge=net.edge(current,next); m.decisions++
        if (!policy.transmitOK(current,e,scenario.nodes)) break
        e[current]-=p.routingCost+p.tx; m.energy+=p.routingCost+p.tx
        double hopDelay=p.duration+net.distance(current,next)/p.soundSpeed
        // The medium access and propagation time is spent whether the packet
        // is received, corrupted on the channel, or reaches an exhausted node.
        packetDelay+=hopDelay
        if (channel.nextDouble()>net.quality(current,next)) {
          blocked.add(edge); m.failedHops++; continue
        }
        if (e[next]<p.rx) break
        e[next]-=p.rx; m.energy+=p.rx
        current=next; visited.add(next); hops++; m.dataHops++
      }
      m.elapsed+=packetDelay
      if (current==net.destination) {
        m.delivered++; m.hops << hops; m.deliveryDelay+=packetDelay
      }
    }
    if (m.lifetimePackets < 0) m.lifetimePackets=count
    m
  }
  void printMetrics(String scenario, Policy policy, Metrics m) {
    println String.format('%-20s %-24s PDR=%6.2f%% loss=%6.2f%% delay=%7.4fs throughput=%8.1f bps energy=%7.3f J hops=%4.2f life=%3d pkts overhead=%4.2f',
      scenario,policy.name,m.pdr(),m.loss(),m.meanDelay(),m.throughput(p.bytes),m.energy,m.meanHops(),m.lifetimePackets,m.offered ? m.decisions/(double)m.offered : 0d)
  }
  void run(int count) {
    if (count<1) throw new IllegalArgumentException('Packet count must be at least 1.')
    println 'EAFTR — Reinforcement Learning Energy-Aware Fault-Tolerant Routing'
    println String.format('Packets/experiment: %d | training episodes: %d | packet duration: %.4f s | TX: %.4f J | RX: %.4f J',
      count,p.episodes,p.duration,p.tx,p.rx)
    List<Scenario> scenarios=[
      new Scenario('Normal',[] as Set<String>,[] as Set<Integer>,net.initialEnergy),
      new Scenario('Link N3-N6 failed',[net.edge(3,6)] as Set<String>,[] as Set<Integer>,net.initialEnergy),
      new Scenario('Node N3 failed',[] as Set<String>,[3] as Set<Integer>,net.initialEnergy),
      // Deliberately energy constrained, but still connected. The old setup
      // made N6 ineligible as a relay and therefore had no source-destination
      // path before the first packet was offered.
      new Scenario('Low battery',[] as Set<String>,[] as Set<Integer>,[5d,4d,4d,2d,1.5d,3d,4d,5d])]
    File csv=new File('results.csv')
    csv.text='scenario,algorithm,offered,delivered,pdr_percent,loss_percent,average_delay_s,throughput_bps,energy_j,average_hops,lifetime_packets,routing_decisions,failed_hops\n'
    scenarios.eachWithIndex { Scenario s, int si ->
      println "\n--- ${s.name}; failed links: ${s.links ?: 'none'}; failed nodes: ${s.nodes ?: 'none'} ---"
      List<Policy> policies=[new Policy('EAFTR Q-learning',net,p),new EnergyGreedy(net,p),new Geographic(net,p)]
      policies.each { Policy policy ->
        // Common random numbers make every policy face the same reproducible channel stream.
        Metrics m=execute(policy,s,count,10000L+si*100L); printMetrics(s.name,policy,m)
        csv << [s.name,policy.name,m.offered,m.delivered,m.pdr(),m.loss(),m.meanDelay(),m.throughput(p.bytes),m.energy,m.meanHops(),m.lifetimePackets,m.decisions,m.failedHops].collect {
          it instanceof String ? '"'+it.replace('"','""')+'"' : it
        }.join(',')+'\n'
      }
    }
    println "\nSaved machine-readable results to ${csv.canonicalPath}"
  }
}

int packets=args ? Integer.parseInt(args[0]) : 100
new Simulation().run(packets)
