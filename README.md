# EAFTR: RL-based Energy-Aware Fault-Tolerant Routing

This project implements the proposed methodology from `Progress-215_233_245.pdf` as a reproducible Groovy simulation. It compares EAFTR Q-learning against energy-greedy and geographic baselines under normal, link-failure, node-failure, and low-battery conditions.

## Run

Install a Java-compatible Groovy runtime, then run from this directory:

```bash
groovy routing_agent.groovy
groovy routing_agent.groovy 250
```

The optional argument is the packet count per experiment. The program prints results and creates `results.csv`.

## Implementation

- Q-learning with epsilon-greedy training and greedy evaluation.
- The report's reward components: residual-energy benefit, link reliability, destination progress, energy cost, hop penalty, and destination reward.
- Feasibility checks that exclude visited nodes, failed nodes/links, links that fail during a packet, low-energy relays, and next hops without a viable onward path.
- Failure recovery that blocks a failed acoustic link for the current packet and selects an alternative feasible route. In the live stack, obsolete EAFTR routes are removed when a failure or energy change leaves no feasible path; RDP fallback entries are retained.
- Distance-based acoustic link quality, 1500 m/s propagation delay, packet airtime, transmission, reception, and routing-decision energy. Both implementations use 128-byte packets at 10 kbps, 10 W transmission, 2 W reception, 0.05 J routing cost, and a 0.10 J reserve.
- Evaluation of PDR, packet loss, mean delay, throughput, total energy, average hops, failed hops, routing-decision overhead, and network lifetime. Lifetime is reported as packets offered before no energy-feasible source-to-destination path remains.

## Configuration

The eight-node topology, locations, node energy, physical constants, Q-learning values, and scenarios are in [routing_agent.groovy](routing_agent.groovy). Nodes are `N0` through `N7`; `N0` is the source and `N7` is the destination.

`8-node-network.groovy` runs the matching real-time UnetStack topology. It loads `EnergyAwareRoutingAgent.groovy` on every node; each agent trains a local policy and installs a loop-free selected next-hop route to every other node into the standard UnetStack Router.

## Real-time UnetStack demo

Copy the agent to the installed UnetStack classes directory and start the topology:

```bash
cd ~/Desktop/CN_project/RL-in-Underwater-Acoustic-Networks
cp EnergyAwareRoutingAgent.groovy ~/Downloads/unet-7.0.0/classes/
~/Downloads/unet-7.0.0/bin/unet 8-node-network.groovy
```

After about five seconds, open http://localhost:8081/ for N0 and run `routes 8` in its shell. It should show the EAFTR-selected next hop. Send a reliable packet with:

```groovy
dtx 8, [1,2,3,4], reliability: true
```

To demonstrate fault adaptation, update the policy agent from a node shell, for example:

```groovy
rlroute.failedLinks = ['4-7']
```

The agent retrains and replaces its own preferred route while preserving normal Router/RDP fallback routes.

The random seeds are fixed, and all policies in a scenario start with the same channel stream, so repeated runs with the same packet count are reproducible and comparable. For a report-quality statistical comparison, run multiple seeds and report confidence intervals.
