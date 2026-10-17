import org.arl.fjage.*

/*
 * Real-time 8-node UnetStack testbed for EnergyAwareRoutingAgent.
 *
 * Before running this file, install the project-local agent source as:
 *   <UNET_HOME>/classes/EnergyAwareRoutingAgent.groovy
 */

println '''
RL energy-aware routing -- real-time 8-node underwater network

N0 (address 1): tcp://localhost:1101, http://localhost:8081/
N1 (address 2): tcp://localhost:1102, http://localhost:8082/
N2 (address 3): tcp://localhost:1103, http://localhost:8083/
N3 (address 4): tcp://localhost:1104, http://localhost:8084/
N4 (address 5): tcp://localhost:1105, http://localhost:8085/
N5 (address 6): tcp://localhost:1106, http://localhost:8086/
N6 (address 7): tcp://localhost:1107, http://localhost:8087/
N7 (address 8): tcp://localhost:1108, http://localhost:8088/
'''

platform = RealTimePlatform

// This is a complete forwarding stack. The policy agent is loaded after the
// standard Router and RouteDiscoveryProtocol services it depends on.
def routingStack = {
  container.add 'arp',       new org.arl.unet.addr.AddressResolution()
  container.add 'ranging',   new org.arl.unet.localization.Ranging()
  container.add 'mac',       new org.arl.unet.mac.CSMA()
  container.add 'uwlink',    new org.arl.unet.link.ECLink()
  container.add 'router',    new org.arl.unet.net.Router()
  container.add 'rdp',       new org.arl.unet.net.RouteDiscoveryProtocol()
  container.add 'transport', new org.arl.unet.transport.CaddyLite()
  container.add 'scheduler', new org.arl.unet.scheduler.Scheduler()
  container.add 'rlroute',   new EnergyAwareRoutingAgent()
}

simulate {
  node 'N0', address: 1, location: [0.m,   0.m,   -10.m], web: 8081, api: 1101, stack: routingStack
  node 'N1', address: 2, location: [100.m, 0.m,   -15.m], web: 8082, api: 1102, stack: routingStack
  node 'N2', address: 3, location: [0.m,   100.m, -20.m], web: 8083, api: 1103, stack: routingStack
  node 'N3', address: 4, location: [200.m, 0.m,   -20.m], web: 8084, api: 1104, stack: routingStack
  node 'N4', address: 5, location: [100.m, 100.m, -25.m], web: 8085, api: 1105, stack: routingStack
  node 'N5', address: 6, location: [0.m,   200.m, -15.m], web: 8086, api: 1106, stack: routingStack
  node 'N6', address: 7, location: [250.m, 100.m, -20.m], web: 8087, api: 1107, stack: routingStack
  node 'N7', address: 8, location: [350.m, 100.m, -10.m], web: 8088, api: 1108, stack: routingStack
}
