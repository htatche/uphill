import L from 'leaflet';

class Main {
  private map!: L.Map;
  private currentLayer?: L.TileLayer;

  constructor() {
    this.initializeMap();
  }

  MAP_PROVIDER = {
    name: 'OpenStreetMap',
    url: 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
    attribution: '© OpenStreetMap contributors',
    maxZoom: 18,
    description: 'Standard street map with excellent global trail coverage'
  };

  private switchProvider(): void {
    const provider = this.MAP_PROVIDER;

    // Add new layer
    this.currentLayer = L.tileLayer(provider.url, {
      attribution: provider.attribution,
      maxZoom: provider.maxZoom,
    });

    this.currentLayer.addTo(this.map);
  }

  private initializeMap(): void {
    // Initialize the map
    this.map = L.map('map').setView([42.5000, 1.6000], 13);

    this.switchProvider();
  }
}

document.addEventListener('DOMContentLoaded', () => {
  new Main();
});