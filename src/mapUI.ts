import L, {LatLng, Polygon} from "leaflet";
import type {Coordinate, BoundingBox} from "@/types/map_types";
import type {OSMNode, OSMWay} from "@/types/osm_types";
import {fetchBoundingBoxNetwork} from "./api";
import {Graph} from "./graph";

export const MAP_PROVIDER = {
    name: "OpenTopoMap",
    url: "https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png",
    attribution: "© OpenStreetMap contributors, SRTM | OpenTopoMap (CC-BY-SA)",
    maxZoom: 17,
} as const;

export class MapUI {
    public map: L.Map;
    private currentLayer: L.TileLayer;
    private coordinates: Coordinate[] = [];
    private currentBoundingBoxPolygon?: Polygon;
    private currentPathNodes: String[] = [];
    private currentPathCoordinates: L.LatLngExpression[] = [];

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
        coordFrom: Coordinate,
        coordTo: Coordinate
    ): BoundingBox {
        const padding: number = 0.05; // 1 km
        // const padding: number = 0; // 1 km

        return {
            south: Math.min(coordFrom.lat, coordTo.lat) - padding,
            west: Math.min(coordFrom.lng, coordTo.lng) - padding,
            north: Math.max(coordFrom.lat, coordTo.lat) + padding,
            east: Math.max(coordFrom.lng, coordTo.lng) + padding,
        };
    }

    private drawBoundingBox(bounding_box: BoundingBox): void {
        const {south, west, north, east} = bounding_box;
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

        this.currentBoundingBoxPolygon?.remove();
        this.currentBoundingBoxPolygon = polygon.addTo(this.map);
    }

    private drawPaths(nodes: OSMNode[], ways: OSMWay[]) {
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

    private addMarker(latLng: LatLng): void {
        console.log(`Clicked at: ${latLng.lat.toFixed(6)}, ${latLng.lng.toFixed(6)}`);

        this.coordinates.push({
            lat: latLng.lat,
            lng: latLng.lng
        });

        L.marker(latLng).addTo(this.map);
    }

    private async exploreBoundingBox(coordFrom: Coordinate, coordTo: Coordinate): Promise<{
        nodes: OSMNode[],
        ways: OSMWay[]
    }> {
        const bounding_box: BoundingBox = this.createBoundingBox(coordFrom, coordTo);
        const {nodes, ways} = await fetchBoundingBoxNetwork(bounding_box);

        this.drawBoundingBox(bounding_box); // if
        this.drawPaths(nodes, ways); // if

        return {nodes, ways}
    }

    private async traceShortestPath(coordFrom: Coordinate, coordTo: Coordinate): Promise<void> {
        const {nodes, ways} = await this.exploreBoundingBox(coordFrom, coordTo);
        const graph = new Graph();

        graph.build(nodes, ways);

        const [nodeFrom, nodeTo]: String[] = graph.translateCoordsToNodes(coordFrom, coordTo);
        const pathCoordinates = graph.findShortestPath(nodeFrom, nodeTo);

        if (pathCoordinates) {
            this.currentPathCoordinates = [...(this.currentPathCoordinates), ...pathCoordinates];

            L.polyline(this.currentPathCoordinates, {color: "red", weight: 4}).addTo(this.map);
        }
    }

    private async handleMapClick(e: L.LeafletMouseEvent): Promise<void> {
        this.addMarker(e.latlng);

        const [coordFrom, coordTo] = this.coordinates.slice(-2);

        if (coordFrom && coordTo) {
            await this.traceShortestPath(coordFrom, coordTo);
        }
    }
}
