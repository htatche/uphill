import type { OSMNode, OSMWay, OverpassResponse } from "@/types/overpass";
import * as GraphTypes from "./types/graph";

/**
 * Calculate distance between two coordinates using Haversine formula
 * Returns distance in meters
 */
export function calculateDistance(lat1: number, lon1: number, lat2: number, lon2: number): number {
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

export function findNearestNode(
  coordinate: Coordinate,
  graph: GraphTypes.TrailGraph
): string {
  let nearestId: string | null = null;
  let minDist = Infinity;

  graph.nodes.forEach((node) => {
    const dist = calculateDistance(
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

// Uses Dijkstra to calculate shortest path between two nodes
export function calculateShortestPath(
  graph: GraphTypes.TrailGraph,
  startNodeId: string,
  endNodeId: string
): string[] | null {
  const visited = new Set<string>();
  const pq: GraphTypes.PriorityQueueItem[] = [
    { nodeId: startNodeId, cost: 0, path: [startNodeId] },
  ];

  while (pq.length > 0) {
    // Get the node with smallest cost
    pq.sort((a, b) => a.cost - b.cost);

    const current = pq.shift()!;

    if (current.nodeId === endNodeId) return current.path;
    if (visited.has(current.nodeId)) continue;

    visited.add(current.nodeId);

    const neighbors = graph.adjacencyList.get(current.nodeId) ?? [];

    neighbors.forEach((neighborId) => {
      if (visited.has(neighborId)) return;

      const edgeId = `${current.nodeId}-${neighborId}`;
      const edge = graph.edges.get(edgeId)!;

      pq.push({
        nodeId: neighborId,
        cost: current.cost + edge.weight,
        path: [...current.path, neighborId],
      });
    });
  }

  return null; // no path found
}

export function buildGraph(osmNodes: OSMNode[], osmWays: OSMWay[]): GraphTypes.TrailGraph {
  const graph: GraphTypes.TrailGraph = {
    nodes: new Map(),
    edges: new Map(),
    adjacencyList: new Map(),
  };

  // Create a lookup map for OSM nodes
  const nodeMap = new Map<number, OSMNode>();
  osmNodes.forEach(node => nodeMap.set(node.id, node));

  // Build graph nodes and edges from ways
  osmWays.forEach(way => {
    const wayNodes = way.nodes
      .map(nodeId => nodeMap.get(nodeId))
      .filter((node): node is OSMNode => node !== undefined);

    if (wayNodes.length < 2) return;

    // Create graph nodes for each OSM node in the way
    wayNodes.forEach(osmNode => {
      if (!graph.nodes.has(osmNode.id.toString())) {
        const graphNode: GraphTypes.GraphNode = {
          id: osmNode.id.toString(),
          lat: osmNode.lat,
          lon: osmNode.lon,
          type: 'waypoint', // Default type
          elevation: 0, // TODO: fetch elevation data
        };
        graph.nodes.set(graphNode.id, graphNode);
        graph.adjacencyList.set(graphNode.id, []);
      }
    });

    // Create edges between consecutive nodes in the way
    for (let i = 0; i < wayNodes.length - 1; i++) {
      const fromNode = wayNodes[i];
      const toNode = wayNodes[i + 1];
      const fromId = fromNode.id.toString();
      const toId = toNode.id.toString();

      const distance = calculateDistance(
        fromNode.lat,
        fromNode.lon,
        toNode.lat,
        toNode.lon
      );

      // Create edge from -> to
      const edgeId = `${fromId}-${toId}`;
      const edge: GraphTypes.GraphEdge = {
        id: edgeId,
        from: fromId,
        to: toId,
        weight: distance,
        distance,
        elevationGain: 0, // TODO: calculate from elevation data
        difficulty: 1, // Default difficulty
      };

      graph.edges.set(edgeId, edge);
      graph.adjacencyList.get(fromId)?.push(toId);

      // Create reverse edge (trails are bidirectional)
      const reverseEdgeId = `${toId}-${fromId}`;
      const reverseEdge: GraphTypes.GraphEdge = {
        id: reverseEdgeId,
        from: toId,
        to: fromId,
        weight: distance,
        distance,
        elevationGain: 0,
        difficulty: 1,
      };

      graph.edges.set(reverseEdgeId, reverseEdge);
      graph.adjacencyList.get(toId)?.push(fromId);
    }
  });

  console.log(`Built graph with ${graph.nodes.size} nodes and ${graph.edges.size} edges`);

  return graph;
}