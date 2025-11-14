import L from "leaflet";
import { buildNetworkQuery, fetchBoundingBoxNetwork } from "./api";
import type { Coordinate, BoundingBox } from "./types/graph";

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
    // const padding: number = 0.01; // 1 km
    const padding: number = 0; // 1 km

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

      console.log(nodes);
      console.log(ways);
    }
  }
}
