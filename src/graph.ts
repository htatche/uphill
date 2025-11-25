import type {Coordinate} from "@/types/map_types";
import type {OSMNode, OSMWay} from "@/types/osm_types";
import type {GraphEdge, GraphNode, PriorityQueueItem} from "@/types/graph_types";
import L from "leaflet";

export class Graph {
    public nodes = new Map<string, GraphNode>();
    public edges = new Map<string, GraphEdge>();
    public adjacencyList = new Map<string, string[]>();

    /**
     * Calculates the distance between two geographic coordinates using the Haversine formula.
     *
     * @param {number} lat1 The latitude of the first point in degrees.
     * @param {number} lon1 The longitude of the first point in degrees.
     * @param {number} lat2 The latitude of the second point in degrees.
     * @param {number} lon2 The longitude of the second point in degrees.
     * @return {number} The distance between the two points in meters.
     */
    static calculateDistance(lat1: number, lon1: number, lat2: number, lon2: number): number {
        const R = 6371e3; // Earth's radius in meters
        const φ1 = (lat1 * Math.PI) / 180;
        const φ2 = (lat2 * Math.PI) / 180;
        const Δφ = ((lat2 - lat1) * Math.PI) / 180;
        const Δλ = ((lon2 - lon1) * Math.PI) / 180;

        const a =
            Math.sin(Δφ / 2) * Math.sin(Δφ / 2) +
            Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) * Math.sin(Δλ / 2);
        const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

        return R * c;
    }

    /**
     * Finds the nearest node to the given coordinate.
     *
     * @param {Coordinate} coordinate - The geographic coordinate (latitude and longitude) used to find the nearest node.
     * @return {string} The ID of the nearest node to the provided coordinate.
     * @throws {Error} If there are no nodes in the graph.
     */
    public findNearestNode(coordinate: Coordinate): string {
        let nearestId: string | null = null;
        let minDist = Infinity;

        this.nodes.forEach((node) => {
            const dist = Graph.calculateDistance(
                coordinate.lat,
                coordinate.lng,
                node.lat,
                node.lon
            );

            if (dist < minDist) {
                minDist = dist;
                nearestId = node.id;
            }
        });

        if (!nearestId) throw new Error("No nodes in graph");

        return nearestId;
    }

    /**
     * Translates the given coordinates to their corresponding notes (node identifiers).
     *
     * @param {Coordinate} coordFrom - The starting coordinate to be translated.
     * @param {Coordinate} coordTo - The ending coordinate to be translated.
     * @return {string[]} An array containing the identifiers of the nodes corresponding to the coordinates.
     */
    public translateCoordsToNodes(coordFrom: Coordinate, coordTo: Coordinate): string[] {
        const startNodeId: string = this.findNearestNode(coordFrom);
        const endNodeId: string = this.findNearestNode(coordTo);

        return [startNodeId, endNodeId];
    }

    /**
     * Converts an array of graph node IDs into Leaflet LatLngExpressions.
     *
     * @param {string[]} ids - The node IDs to convert.
     * @return {Array<L.LatLngExpression>} Coordinates for the provided IDs.
     */
    private nodeIdsToCoords(ids: string[]): Array<L.LatLngExpression> {
        return ids.map((id) => {
            const node = this.nodes.get(id)!;
            return [node.lat, node.lon] as L.LatLngExpression;
        });
    }

    private addNode(osmNode: OSMNode): void {
        if (!this.nodes.has(osmNode.id.toString())) {
            const graphNode: GraphNode = {
                id: osmNode.id.toString(),
                lat: osmNode.lat,
                lon: osmNode.lon,
                type: 'waypoint', // Default type
                elevation: 0, // TODO: fetch elevation data
            };
            this.nodes.set(graphNode.id, graphNode);
            this.adjacencyList.set(graphNode.id, []);
        }
    }

    private addEdge(fromId: string, toId: string, distance: number): void {
        const edgeId = `${fromId}-${toId}`;
        const edge: GraphEdge = {
            id: edgeId,
            from: fromId,
            to: toId,
            weight: distance,
            distance,
            elevationGain: 0, // TODO: calculate from elevation data
            difficulty: 1, // Default difficulty
        };

        this.edges.set(edgeId, edge);
        this.adjacencyList.get(fromId)?.push(toId);
    }

    /**
     * Builds a graph structure using the provided OSM (OpenStreetMap) nodes and ways.
     * The graph consists of nodes (representing geographical points) and edges
     * (representing connections between those points) with distances as weights.
     *
     * @param {OSMNode[]} osmNodes - The array of OSMNode objects, each representing a geographical point.
     * @param {OSMWay[]} osmWays - The array of OSMWay objects, each representing a series of connected OSMNode IDs.
     * @return {void} This method does not return a value but constructs the graph by populating internal data structures.
     */
    public build(osmNodes: OSMNode[], osmWays: OSMWay[]): void {
        const nodeMap = new Map<number, OSMNode>();

        // Create a lookup map for OSM nodes
        osmNodes.forEach(node => nodeMap.set(node.id, node));
        // Build graph nodes and edges from ways
        osmWays.forEach(way => {
            const wayNodes = way.nodes
                .map(nodeId => nodeMap.get(nodeId))
                .filter((node): node is OSMNode => node !== undefined);

            if (wayNodes.length < 2) return;

            // Create graph nodes for each OSM node in the way
            wayNodes.forEach(osmNode => {
                this.addNode(osmNode);
            });

            // Create edges between consecutive nodes in the way
            for (let i = 0; i < wayNodes.length - 1; i++) {
                const fromNode = wayNodes[i];
                const toNode = wayNodes[i + 1];
                const fromId = fromNode.id.toString();
                const toId = toNode.id.toString();

                const distance = Graph.calculateDistance(
                    fromNode.lat,
                    fromNode.lon,
                    toNode.lat,
                    toNode.lon
                );

                this.addEdge(fromId, toId, distance);
                // Create a reverse edge (trails are bidirectional)
                this.addEdge(toId, fromId, distance);
            }
        });

        console.log(`Built graph with ${this.nodes.size} nodes and ${this.edges.size} edges`);
    }

    /**
     * Calculates the shortest path between two nodes in a graph and returns the corresponding coordinates.
     *
     * @param {string} nodeFrom - The identifier of the starting node.
     * @param {string} nodeTo - The identifier of the destination node.
     * @return {Array<L.LatLngExpression> | null} An array of coordinates representing the shortest path, or null if no path is found.
     */
    public findShortestPath(nodeFrom: string, nodeTo: string) {
        const shortestPath: string[] | null = this.calculateShortestPath(
            nodeFrom,
            nodeTo
        );

        if (shortestPath && shortestPath.length > 0) {
            const pathCoords = this.nodeIdsToCoords(shortestPath);
            return pathCoords;
        } else {
            console.log("No shortest path found");

            return null;
        }
    }

    /**
     * Calculates the shortest path between two nodes in a graph using Dijkstra's algorithm.
     *
     * @param {string} startNodeId - The ID of the starting node.
     * @param {string} endNodeId - The ID of the target node.
     * @return {string[] | null} The shortest path as an array of node IDs if a path is found, or null if no path exists.
     */
    private calculateShortestPath(startNodeId: string, endNodeId: string): string[] | null {
        const visited = new Set<string>();
        const pq: PriorityQueueItem[] = [
            {nodeId: startNodeId, cost: 0, path: [startNodeId]},
        ];

        while (pq.length > 0) {
            // Get the node with the smallest cost
            pq.sort((a, b) => a.cost - b.cost);

            const current = pq.shift()!;

            if (current.nodeId === endNodeId) return current.path;
            if (visited.has(current.nodeId)) continue;

            visited.add(current.nodeId);

            const neighbors = this.adjacencyList.get(current.nodeId) ?? [];

            neighbors.forEach((neighborId) => {
                if (visited.has(neighborId)) return;

                const edgeId = `${current.nodeId}-${neighborId}`;
                const edge = this.edges.get(edgeId)!;

                pq.push({
                    nodeId: neighborId,
                    cost: current.cost + edge.weight,
                    path: [...current.path, neighborId],
                });
            });
        }

        return null; // no path found
    }

    /**
     * Handles entering a node during DFS: updates path, visited set, and traversal order.
     * Returns the potentially incremented visitCounter.
     */
    private enterNode(
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
    private exitNode(
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
    private recordCycle(currentPath: string[], startNodeId: string, cycles: string[][]): void {
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
     * @param {Set<string>} visited - Set of nodes visited in the current path.
     * @param {string[]} currentPath - The current path being explored.
     * @param {string[][]} cycles - Accumulator for all found cycles.
     * @param {number} remainingDepth - Remaining depth allowed for exploration.
     * @param {Map<string, number>} traversalOrder - Map tracking visit order of nodes.
     * @param {number} visitCounter - Counter for tracking visit order.
     */
    private depthFirstSearchRecursive(
        currentNodeId: string,
        startNodeId: string,
        visited: Set<string>,
        currentPath: string[],
        cycles: string[][],
        remainingDepth: number,
        traversalOrder: Map<string, number>,
        visitCounter: number
    ): number {
        if (remainingDepth <= 0) return visitCounter;

        visitCounter = this.enterNode(currentNodeId, visited, currentPath, traversalOrder, visitCounter);

        const neighbors = this.adjacencyList.get(currentNodeId) ?? [];

        neighbors.forEach((neighborId) => {
            const cycleFound = currentPath.length > 2 && neighborId === startNodeId

            if (cycleFound) {
                this.recordCycle(currentPath, startNodeId, cycles);
                return;
            }
            // Continue exploring if neighbor hasn't been visited in current path
            if (!visited.has(neighborId)) {
                visitCounter = this.depthFirstSearchRecursive(
                    neighborId,
                    startNodeId,
                    visited,
                    currentPath,
                    cycles,
                    remainingDepth - 1,
                    traversalOrder,
                    visitCounter
                );
            }
        });

        this.exitNode(currentNodeId, visited, currentPath);

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
     * @param {number} maxDepth - Maximum depth to explore (prevents infinite recursion).
     * @return {{cycles: string[][], traversalOrder: Map<string, number>}} An object with cycles and traversal order.
     */
    private depthFirstSearch(startNodeId: string, maxDepth: number = 600): {
        cycles: string[][],
        traversalOrder: Map<string, number>
    } {
        const cycles: string[][] = [];
        const visited = new Set<string>();
        const currentPath: string[] = [];
        const traversalOrder = new Map<string, number>();
        let visitCounter = 0;

        console.log(`Starting DFS from node ${startNodeId} with maxDepth ${maxDepth}`);

        this.depthFirstSearchRecursive(startNodeId, startNodeId, visited, currentPath, cycles, maxDepth, traversalOrder, visitCounter);

        console.log(`DFS completed: visited ${traversalOrder.size} nodes, found ${cycles.length} cycles`);

        return {cycles, traversalOrder};
    }

    /**
     * Finds all cycles starting from a given node and returns them as coordinate arrays.
     * Only returns cycles where the path loops back to the starting node.
     *
     * ASCII Diagram:
     *     nodeId (start)
     *        |
     *        v
     *     [DFS Search] ---> finds cycles
     *        |
     *        v
     *     [[A,B,C,A], [A,D,E,A]] ---> converts to coordinates
     *        |
     *        v
     *     [[[lat1,lon1], [lat2,lon2], ...], ...]
     *
     * @param {string} nodeId - The ID of the node to start searching from.
     * @return {{cycles: L.LatLngExpression[][] | null, traversalOrder: Map<string, number>}} An object with cycles and traversal order.
     */
    public findCycles(nodeId: string): {
        cycles: L.LatLngExpression[][] | null,
        traversalOrder: Map<string, number>
    } {
        const {cycles, traversalOrder} = this.depthFirstSearch(nodeId);

        if (cycles.length === 0) {
            console.log("No cycles found");
            return {cycles: null, traversalOrder};
        }

        const cycleCoords = cycles.map((cycle) => this.nodeIdsToCoords(cycle));

        return {cycles: cycleCoords, traversalOrder};
    }
}