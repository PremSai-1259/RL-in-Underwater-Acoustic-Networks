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

`8-node-network.groovy` runs the matching real-time UnetStack topology. It loads `EnergyAwareRoutingAgent.groovy` on every node. Each agent owns only its own residual-energy model and local `Q[destination][nextHop]` values, then installs its selected next hop into the standard UnetStack Router. The centralized energy lists in `routing_agent.groovy` are deliberately limited to the offline experiment; they are not used by the live agent.

## Real-time UnetStack demo

Copy the agent to the installed UnetStack classes directory and start the topology:

```bash
cd ~/Desktop/CN_project/RL-in-Underwater-Acoustic-Networks
cp EnergyAwareRoutingAgent.groovy ~/Downloads/unet-7.0.0/classes/
~/Downloads/unet-7.0.0/bin/unet 8-node-network.groovy
```

After neighbour-energy advertisements have been received (or injected for the shell demo below), open http://localhost:8081/ for N0 and run `routes 8` in its shell. It should show the EAFTR-selected next hop. Send a reliable packet with:

```groovy
dtx 8, [1,2,3,4], reliability: true
```

To demonstrate fault adaptation, update the policy agent from a node shell, for example:

```groovy
rlroute.failedLinks = ['4-7']
```

The agent retrains and replaces its own preferred route while preserving normal Router/RDP fallback routes.

## Live-agent energy and neighbour information

Set a **single** local energy value on each node; lists are rejected so a node cannot configure or inspect every node's battery:

```groovy
rlroute.energy = 40.0
```

The live agent uses the same model as the simulation: 128-byte packets at 10 kbps give 0.1024 s airtime, 1.024 J transmit energy, 0.2048 J receive energy, 0.05 J per routing decision, and a 0.10 J reserve. A local `DatagramTransmissionNtf` charges the decision plus transmit cost exactly once for each acoustic send attempt (including a real retry). A `DatagramNtf` confirms successful local delivery and charges reception once. Residual energy is clamped at zero and triggers a route refresh.

Every node automatically requests each direct neighbour's energy estimate at startup and then at most once per minute. The request and reply use a dedicated EAFTR application datagram protocol over one-hop ECLink, so a node receives the estimate from its neighbour rather than reading a remote variable. Replies carry the neighbour's residual energy, sequence number and timestamp; each node keeps only its own cache, and snapshots expire after 90 seconds. Missing or stale energy makes that next hop infeasible. The control datagrams use their actual small payload length for TX/RX energy accounting, while data forwarding keeps the 128-byte model. `neighborInfo` is read-only and displays the received local cache.

Q-values are initialized only on first use and persist across failure and energy refreshes. The built-in bootstrap is a terminal, one-hop modelled transition because the current stack has no verified end-to-end ACK/delivery feedback; it never reads another node's Q-table. `recordForwardingFeedback()` is provided for an ACK/delivery adapter to apply the normal TD continuation update with real feedback. This is therefore a distributed policy with modelled bootstrap, not a claim of end-to-end online RL from the stock stack alone.

The random seeds are fixed, and all policies in a scenario start with the same channel stream, so repeated runs with the same packet count are reproducible and comparable. For a report-quality statistical comparison, run multiple seeds and report confidence intervals.
