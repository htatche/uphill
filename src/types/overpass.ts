// OpenStreetMap Overpass API response for trail data
export interface OSMNode {
  type: 'node';
  id: number;
  lat: number;
  lon: number;
  tags?: Record<string, string>;
}

export interface OSMWay {
  type: 'way';
  id: number;
  nodes: number[]; // array of node IDs
  tags?: Record<string, string>;
}

export interface OSMRelation {
  type: 'relation';
  id: number;
  members: Array<{
    type: 'node' | 'way' | 'relation';
    ref: number;
    role: string;
  }>;
  tags?: Record<string, string>;
}

export type OSMElement = OSMNode | OSMWay | OSMRelation;

export interface OverpassResponse {
  version: number;
  generator: string;
  elements: OSMElement[];
}