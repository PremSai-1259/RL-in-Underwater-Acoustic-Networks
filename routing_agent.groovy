import org.arl.fjage.*
import org.arl.unet.*

/*
 * Reinforcement-learning-based energy-aware, fault-tolerant routing.
 *
 * State  : node currently holding the packet
 * Action : a reachable neighbour
 * Reward : residual energy - acoustic hop cost, with a large delivery reward
 *
 * A failed link/node is removed from the action set. Therefore the same
 * learner can be retrained after a fault without ever selecting that path.
 */

final int NODE_COUNT = 8
final int SOURCE = 0
final int DESTINATION = 7
final int EPISODES = 10_000
final int MAX_HOPS = NODE_COUNT - 1       // prevents routing loops in an episode

// Q-learning parameters from the proposal's RL design.
final double ALPHA = 0.15                 // learning rate
final double GAMMA = 0.90                 // future-reward discount
final double EPSILON_START = 0.30          // exploration probability
final double EPSILON_END = 0.03
final double MINIMUM_ENERGY = 5.0          // a depleted relay is unavailable

// Undirected connectivity graph for N0 ... N7.
final List<List<Integer>> NEIGHBORS = [
    [1, 2],
    [0, 3, 4],
    [0, 4, 5],
    [1, 6],
    [1, 2, 6],
    [2, 6],
    [3, 4, 5, 7],
    [6]
]

// Coordinates are used to approximate the cost of an acoustic hop (metres).
final List<List<Double>> POSITION = [
    [0.0,   0.0,   -10.0],
    [100.0, 0.0,   -15.0],
    [0.0,   100.0, -20.0],
    [200.0, 0.0,   -20.0],
    [100.0, 100.0, -25.0],
    [0.0,   200.0, -15.0],
    [250.0, 100.0, -20.0],
    [350.0, 100.0, -10.0]
]

// Percentage of battery remaining at each node. Change values to test balancing.
final List<Double> ENERGY = [100.0, 55.0, 90.0, 70.0, 60.0, 85.0, 65.0, 100.0]

def linkKey = { int a, int b -> "${Math.min(a, b)}-${Math.max(a, b)}" }

def distance = { int a, int b ->
    Math.sqrt((0..<3).collect { axis ->
        double difference = POSITION[a][axis] - POSITION[b][axis]
        difference * difference
    }.sum() as double)
}

def newQTable = {
    (0..<NODE_COUNT).collect { (0..<NODE_COUNT).collect { 0.0d } }
}

def routeForScenario = { String title, Set<String> failedLinks, Set<Integer> failedNodes ->
    def q = newQTable()
    def random = new Random(42L)           // reproducible demonstrations/results

    def isAvailable = { int from, int to ->
        !failedNodes.contains(from) &&
        !failedNodes.contains(to) &&
        ENERGY[to] >= MINIMUM_ENERGY &&
        !failedLinks.contains(linkKey(from, to))
    }

    def actions = { int node, Set<Integer> visited ->
        NEIGHBORS[node].findAll { next -> isAvailable(node, next) && !visited.contains(next) }
    }

    // Higher residual energy is preferred; longer underwater hops cost more.
    def reward = { int from, int to ->
        if (to == DESTINATION) return 100.0d
        double energyBenefit = ENERGY[to] / 10.0d
        double acousticCost = distance(from, to) / 100.0d
        energyBenefit - acousticCost - 1.0d
    }

    EPISODES.times { episode ->
        int current = SOURCE
        def visited = [SOURCE] as Set<Integer>
        double epsilon = EPSILON_START -
            (EPSILON_START - EPSILON_END) * episode / (EPISODES - 1.0d)

        for (int hop = 0; hop < MAX_HOPS && current != DESTINATION; hop++) {
            List<Integer> choices = actions(current, visited)
            if (choices.isEmpty()) break

            int next = random.nextDouble() < epsilon
                ? choices[random.nextInt(choices.size())]
                : choices.max { candidate -> q[current][candidate] }

            double immediateReward = reward(current, next)
            visited.add(next)
            List<Integer> futureActions = next == DESTINATION ? [] : actions(next, visited)
            double bestFuture = futureActions.isEmpty()
                ? 0.0d
                : futureActions.collect { candidate -> q[next][candidate] }.max() as double

            q[current][next] += ALPHA *
                (immediateReward + GAMMA * bestFuture - q[current][next])
            current = next
        }
    }

    int current = SOURCE
    def route = [SOURCE]
    def visited = [SOURCE] as Set<Integer>
    def hopRewards = []

    for (int hop = 0; hop < MAX_HOPS && current != DESTINATION; hop++) {
        List<Integer> choices = actions(current, visited)
        if (choices.isEmpty()) break
        int next = choices.max { candidate -> q[current][candidate] }
        hopRewards << String.format("N%d -> N%d  (Q=%.2f, reward=%.2f)",
            current, next, q[current][next], reward(current, next))
        route << next
        visited.add(next)
        current = next
    }

    println "\n${'=' * 62}"
    println title
    println "${'=' * 62}"
    println "Failed links: " + (failedLinks ? failedLinks.join(', ') : 'none')
    println "Failed nodes: " + (failedNodes ? failedNodes.collect { "N${it}" }.join(', ') : 'none')
    println "Energy (%): " + ENERGY.withIndex().collect { value, index -> "N${index}=${value.intValue()}" }.join(', ')
    println "Selected route: " + route.collect { "N${it}" }.join(' -> ')
    println current == DESTINATION ? 'Destination reached.' : 'No fault-tolerant route found.'
    hopRewards.each { println "  ${it}" }
    println "\nQ-table (only valid links acquire values):"
    (0..<NODE_COUNT).each { state ->
        println "N${state}: " + q[state].collect { String.format('%7.2f', it) }.join(' ')
    }
}

// Baseline, then re-learning with the N5--N6 acoustic link unavailable.
routeForScenario('SCENARIO 1: NORMAL NETWORK', [] as Set<String>, [] as Set<Integer>)
routeForScenario('SCENARIO 2: N5 <-> N6 LINK FAILURE', [linkKey(5, 6)] as Set<String>, [] as Set<Integer>)