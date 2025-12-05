/**
 * Handles entering a node during DFS: updates path, visited set, and traversal order.
 * Returns the potentially incremented visitCounter.
 */
function enterNode(
    nodeId: string,
    visited: Set<string>,
    currentPath: string[],
    traversalOrder: Map<string, number>,
    visitCounter: number
): number {
    currentPath.push(nodeId);
    visited.add(nodeId);
    if (!traversalOrder.has(nodeId)) {
        traversalOrder.set(nodeId, visitCounter++);
    }
    return visitCounter;
}

/**
 * Handles backtracking from a node during DFS: removes from path and visited set.
 */
function exitNode(
    nodeId: string,
    visited: Set<string>,
    currentPath: string[]
): void {
    currentPath.pop();
    visited.delete(nodeId);
}

/**
 * Records a detected cycle during DFS by copying the current path and appending the start node.
 *
 * @param {string[]} currentPath - Current traversal path (will not be mutated).
 * @param {string} startNodeId - The starting node to close the cycle.
 * @param {string[][]} cycles - Accumulator to push the completed cycle onto.
 * @private
 */
function recordCycle(currentPath: string[], startNodeId: string, cycles: string[][]): void {
    const cycle = [...currentPath, startNodeId];
    cycles.push(cycle);
}

/**
 * Recursive depth-first search helper that explores all paths and detects cycles.
 *
 * This function makes the recursion explicit via self-calls to depthFirstSearchRecursive
 * and iterates neighbors with modern Array iteration (forEach) instead of a for..of loop.
 *
 * ASCII Diagram:
 *     current
 *        |
 *        v
 *     [N1, N2, N3] --- forEach(neighbor) --> (recursive call)
 *
 * @param {string} currentNodeId - The current node being visited.
 * @param {string} startNodeId - The original starting node (used to detect cycles back to start).
 * @param adjacencyList
 * @param {Set<string>} visited - Set of nodes visited in the current path.
 * @param {string[]} currentPath - The current path being explored.
 * @param {string[][]} cycles - Accumulator for all found cycles.
 * @param {number} remainingDepth - Remaining depth allowed for exploration.
 * @param {Map<string, number>} traversalOrder - Map tracking visit order of nodes.
 * @param {number} visitCounter - Counter for tracking visit order.
 */
function depthFirstSearchRecursive(
    currentNodeId: string,
    startNodeId: string,
    adjacencyList: Map<string, string[]>,
    visited: Set<string>,
    currentPath: string[],
    cycles: string[][],
    remainingDepth: number,
    traversalOrder: Map<string, number>,
    visitCounter: number
): number {
    if (remainingDepth <= 0) return visitCounter;

    visitCounter = enterNode(currentNodeId, visited, currentPath, traversalOrder, visitCounter);

    const neighbors = adjacencyList.get(currentNodeId) ?? [];

    neighbors.forEach((neighborId) => {
        const cycleFound = currentPath.length > 2 && neighborId === startNodeId

        if (cycleFound) {
            recordCycle(currentPath, startNodeId, cycles);
            return;
        }
        // Continue exploring if neighbor hasn't been visited in current path
        if (!visited.has(neighborId)) {
            visitCounter = depthFirstSearchRecursive(
                neighborId,
                startNodeId,
                adjacencyList,
                visited,
                currentPath,
                cycles,
                remainingDepth - 1,
                traversalOrder,
                visitCounter
            );
        }
    });

    exitNode(currentNodeId, visited, currentPath);

    return visitCounter;
}

/**
 * Performs a depth-first search starting from a given node to find all cycles in the graph.
 * Only returns paths that form complete cycles (end node connects back to start node).
 *
 * ASCII Diagram:
 *     A ----> B
 *     ^       |
 *     |       v
 *     D <---- C
 *
 * Starting from A: visits A -> B -> C -> D -> A (cycle found!)
 *
 * @param {string} startNodeId - The ID of the node to start the search from.
 * @param adjacencyList
 * @param {number} maxDepth - Maximum depth to explore (prevents infinite recursion).
 * @return {{cycles: string[][], traversalOrder: Map<string, number>}} An object with cycles and traversal order.
 */
export function depthFirstSearch(startNodeId: string, adjacencyList: Map<string, string[]>, maxDepth: number = 600): {
    cycles: string[][],
    traversalOrder: Map<string, number>
} {
    const cycles: string[][] = [];
    const visited = new Set<string>();
    const currentPath: string[] = [];
    const traversalOrder = new Map<string, number>();
    let visitCounter = 0;

    console.log(`Starting DFS from node ${startNodeId} with maxDepth ${maxDepth}`);

    depthFirstSearchRecursive(startNodeId, startNodeId, adjacencyList, visited, currentPath, cycles, maxDepth, traversalOrder, visitCounter);

    console.log(`DFS completed: visited ${traversalOrder.size} nodes, found ${cycles.length} cycles`);

    return {cycles: cycles, traversalOrder};
}