import L from "leaflet";

export class MapUtils {
  public map: L.Map;
  private currentLayer: L.TileLayer;
  private waypoint?: L.Marker;
  private waypointCoord?: { lat: number; lng: number };
  static readonly MAP_PROVIDER = {
    name: "OpenStreetMap",
    url: "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png",
    attribution: "© OpenStreetMap contributors",
    maxZoom: 18,
    description: "Standard street map with excellent global trail coverage",
  };

  constructor() {
    this.map = L.map("map").setView([42.5, 1.6], 13);
    this.currentLayer = L.tileLayer(MapUtils.MAP_PROVIDER.url, {
      attribution: MapUtils.MAP_PROVIDER.attribution,
      maxZoom: MapUtils.MAP_PROVIDER.maxZoom,
    });
  }

  public initializeMap(): void {
    this.currentLayer.addTo(this.map);

    this.map.on("click", (e: L.LeafletMouseEvent) => {
      this.handleMapClick(e);
    });
  }

  private handleMapClick(e: L.LeafletMouseEvent): void {
    const { lat, lng } = e.latlng;
    
    console.log(`Clicked at: ${lat.toFixed(6)}, ${lng.toFixed(6)}`);

    this.waypoint = L.marker([lat, lng]).addTo(this.map);
    this.waypointCoord = { lat, lng };
  }
}
