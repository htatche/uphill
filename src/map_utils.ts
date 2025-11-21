import L from "leaflet";
import type { Coordinate, BoundingBox } from "./types/map_types";
import type { OSMNode, OSMWay } from "@/types/osm_types";
import { buildNetworkQuery, fetchBoundingBoxNetwork } from "./api";
import { Graph } from "./graph";

export const MAP_PROVIDER = {
  name: "OpenTopoMap",
  url: "https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png",
  attribution: "© OpenStreetMap contributors, SRTM | OpenTopoMap (CC-BY-SA)",
  maxZoom: 17,
} as const;

export class MapUtils {
  public map: L.Map;
  private currentLayer: L.TileLayer;
  private coordinates: Coordinate[] = [];
  private graph: Graph = {
    nodes: new Map(),
    edges: new Map(),
    adjacencyList: new Map(),
  };

  constructor() {
    this.map = L.map("map").setView([42.5, 1.6], 13);
    this.currentLayer = L.tileLayer(MAP_PROVIDER.url, {
      attribution: MAP_PROVIDER.attribution,
      maxZoom: MAP_PROVIDER.maxZoom,
    });

    this.initializeMap();
  }

  private initializeMap(): void {
    this.currentLayer.addTo(this.map);

    this.map.on("click", (e: L.LeafletMouseEvent) => {
      this.handleMapClick(e);
    });
  }

  private createBoundingBox(
    coordinate: Coordinate,
    coordinate2: Coordinate
  ): BoundingBox {
    const padding: number = 0.05; // 1 km
    // const padding: number = 0; // 1 km

    return {
      south: Math.min(coordinate.lat, coordinate2.lat) - padding,
      west: Math.min(coordinate.lng, coordinate2.lng) - padding,
      north: Math.max(coordinate.lat, coordinate2.lat) + padding,
      east: Math.max(coordinate.lng, coordinate2.lng) + padding,
    };
  }

  private drawBoundingBox(bounding_box: BoundingBox): void {
    const { south, west, north, east } = bounding_box;

    const corners: L.LatLngExpression[] = [
      [south, west], // SW
      [south, east], // SE
      [north, east], // NE
      [north, west], // NW
      [south, west], // close polygon
    ];

    const polygon = L.polygon(corners, {
      color: "red",
      weight: 2,
      fill: false,
    });

    polygon.addTo(this.map);
  }

  private displayNodes() {
    // TODO
  }

  private drawWays(nodes: OSMNode[], ways: OSMWay[]) {
    // Create k->v store
    //   node ID -> node object
    const nodeMap: Record<number, L.LatLngExpression> = {};
    nodes.forEach((n) => (nodeMap[n.id] = [n.lat, n.lon]));

    const colors: string[] = ["pink", "blue", "purple", "orange", "green"];

    ways.forEach((way, idx) => {
      const coords = way.nodes.map((id) => nodeMap[id]).filter(Boolean);

      if (coords.length > 1) {
        L.polyline(coords, {
          color: colors[idx % colors.length],
          weight: 4,
          opacity: 0.8,
        }).addTo(this.map);
      }
    });
  }

  private drawShortestPath(nodes: string[]): void {
    const pathCoords = nodes.map((id) => {
      const node = this.graph.nodes.get(id)!;
      return [node.lat, node.lon] as L.LatLngExpression;
    });

    L.polyline(pathCoords, { color: "red", weight: 4 }).addTo(this.map);
  }

  private async handleMapClick(e: L.LeafletMouseEvent): Promise<void> {
    const { lat, lng } = e.latlng;
    let bounding_box: BoundingBox;
    let query: string = "";

    console.log(`Clicked at: ${lat.toFixed(6)}, ${lng.toFixed(6)}`);

    this.coordinates.push({
      lat: e.latlng.lat,
      lng: e.latlng.lng,
      marker: L.marker(e.latlng).addTo(this.map),
    });

    const [first, second] = this.coordinates.slice(-2);

    if (first && second) {
      bounding_box = this.createBoundingBox(first, second);
      query = buildNetworkQuery(bounding_box);

      console.log(`Generated bounding box query: ${query}`);

      const { nodes, ways } = await fetchBoundingBoxNetwork(bounding_box);

      this.drawBoundingBox(bounding_box);
      this.graph = new Graph();
      this.graph.build(nodes, ways);
      this.drawWays(nodes, ways);

      const startNodeId: string = this.graph.findNearestNode(first);
      const endNodeId: string = this.graph.findNearestNode(second);
      const shortestPath: string[] | null = this.graph.calculateShortestPath(
        startNodeId,
        endNodeId
      );

      if (shortestPath && shortestPath.length > 0) {
        const pathCoords = shortestPath.map((id) => {
          const node = this.graph.nodes.get(id)!;
          return [node.lat, node.lon] as L.LatLngExpression;
        });

        L.polyline(pathCoords, { color: "red", weight: 4 }).addTo(this.map);
      } else {
        console.log("No shortest path found");
      }
    }
  }
}
