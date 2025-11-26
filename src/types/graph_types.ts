// A node represents a point of interest or trail intersection
export interface GraphNode {
  id: string;
  lat: number;
  lon: number;
  elevation: number; // meters above sea level
  name?: string; // optional name for landmarks
  type: 'intersection' | 'trailhead' | 'peak' | 'waypoint';
}

// Trail surface types that affect running difficulty
export type TrailSurface = 'paved' | 'dirt' | 'gravel' | 'rock' | 'sand' | 'mud';

// An edge represents a trail segment connecting two nodes
export interface GraphEdge {
  id: string;
  from: string;
  to: string;
  weight: number; // Primary weight for pathfinding

  // OSM way identifier this edge belongs to (same on both directions)
  wayId?: number;

  // Physical properties
  distance: number; // meters
  elevationGain: number; // meters (positive = uphill)
  elevationLoss?: number; // meters (positive = downhill)

  // Trail characteristics
  surface?: TrailSurface;
  difficulty: 1 | 2 | 3 | 4 | 5; // 1=easy, 5=very difficult

  // Calculated weights for pathfinding algorithms
  weights?: {
    distance: number;
    elevation: number;
    difficulty: number;
    time: number; // estimated time in minutes
  };
}

// Dijkstra
export interface PriorityQueueItem {
  nodeId: string;
  cost: number;
  path: string[];
}
