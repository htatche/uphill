import L from "leaflet";
import {AutoExploreFeature} from "@/features/autoExploreFeature";
import {DrawPathFeature} from "@/features/drawPathFeature";
import {MapActions} from "@/features/mapActions";

export const MAP_PROVIDER = {
    name: "OpenTopoMap",
    url: "https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png",
    attribution: "© OpenStreetMap contributors, SRTM | OpenTopoMap (CC-BY-SA)",
    maxZoom: 17,
} as const;

type MapMode = "draw" | "auto";

export class MapUI {
    public map: L.Map;
    private currentLayer: L.TileLayer;
    private mode: MapMode = "draw";
    private abortController = new AbortController();
    private readonly mapActions: MapActions;
    private drawPathFeature: DrawPathFeature;
    private autoExploreFeature: AutoExploreFeature;

    constructor() {
        this.map = L.map("map").setView([42.5, 1.6], 13);
        this.currentLayer = L.tileLayer(MAP_PROVIDER.url, {
            attribution: MAP_PROVIDER.attribution,
            maxZoom: MAP_PROVIDER.maxZoom,
        });
        this.mapActions = new MapActions(this.map);

        this.drawPathFeature = new DrawPathFeature({
            map: this.map,
            addMarker: this.mapActions.addMarker.bind(this.mapActions),
            createBoundingBox: this.mapActions.createBoundingBox.bind(this.mapActions),
            drawBoundingBox: this.mapActions.drawBoundingBox.bind(this.mapActions),
        });

        this.autoExploreFeature = new AutoExploreFeature({
            map: this.map,
            addMarker: this.mapActions.addMarker.bind(this.mapActions),
            createBoundingBox: this.mapActions.createBoundingBox.bind(this.mapActions),
        });

        this.initializeMap();
        this.initializeModeSelector();
    }

    private initializeMap(): void {
        this.currentLayer.addTo(this.map);

        this.map.on("click", async (e: L.LeafletMouseEvent) => {
            await this.handleMapClick(e);
        });
    }

    private initializeModeSelector(): void {
        const radios = document.querySelectorAll('input[name="mode"]');

        radios.forEach((radio) => {
            radio.addEventListener("change", (e) => {
                const target = e.target as HTMLInputElement;

                this.mode = target.value as MapMode;
                this.resetMap();
            }, {signal: this.abortController.signal});
        });
    }

    private resetMap(): void {
        this.map.eachLayer((layer) => {
            if (layer !== this.currentLayer) {
                this.map.removeLayer(layer);
            }
        });

        this.mapActions.reset();
        this.drawPathFeature.reset();
        this.autoExploreFeature.reset();
    }

    private async handleMapClick(e: L.LeafletMouseEvent): Promise<void> {
        if (this.mode === "draw") {
            await this.drawPathFeature.onMapClick(e.latlng);
            return;
        }

        await this.autoExploreFeature.onMapClick(e.latlng);
    }
}
