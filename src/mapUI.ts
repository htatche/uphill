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
    private currentBoundingBoxPolygon?: Polygon | undefined;
    private currentPathCoordinates: L.LatLngExpression[] = [];
    private mode: 'draw' | 'auto' = 'draw';
    private abortController = new AbortController();
    private cyclePolylines: Map<number, L.Polyline> = new Map();
    private lastCycles: L.LatLngExpression[][] = [];

    constructor() {
        this.map = L.map("map").setView([42.5, 1.6], 13);
        this.currentLayer = L.tileLayer(MAP_PROVIDER.url, {
            attribution: MAP_PROVIDER.attribution,
            maxZoom: MAP_PROVIDER.maxZoom,
        });

        this.initializeMap();
        this.initializeModeSelector();
    }

    private initializeMap(): void {
        this.currentLayer.addTo(this.map);

        this.map.on("click", (e: L.LeafletMouseEvent) => {
            this.handleMapClick(e);
        });
    }

    private initializeModeSelector(): void {
        const radios = document.querySelectorAll('input[name="mode"]');

        radios.forEach(radio => {
            radio.addEventListener('change', (e) => {
                const target = e.target as HTMLInputElement;

                this.mode = target.value as 'draw' | 'auto';

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

        this.coordinates = [];
        this.currentBoundingBoxPolygon = undefined;
        this.currentPathCoordinates = [];
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

        const [nodeFrom, nodeTo]: string[] = graph.translateCoordsToNodes(coordFrom, coordTo);
        const pathCoordinates = graph.findShortestPath(nodeFrom, nodeTo);

        if (pathCoordinates) {
            this.currentPathCoordinates = [...(this.currentPathCoordinates), ...pathCoordinates];

            L.polyline(this.currentPathCoordinates, {color: "red", weight: 4}).addTo(this.map);
        }
    }

    private async traceLoopPath(coord: Coordinate): Promise<void> {
        const bounding_box: BoundingBox = this.createBoundingBox(coord, coord);
        const {nodes, ways} = await fetchBoundingBoxNetwork(bounding_box);
        const graph = new Graph();
        
        graph.build(nodes, ways);

        const startNodeId = graph.findNearestNode(coord);
        const cyclesResult = graph.findCycles(startNodeId);
        const cycles = cyclesResult.cycles;

        if (cycles && cycles.length > 0) {
            const sortedCycles = this.sortCyclesByDistance(cycles);
            this.lastCycles = sortedCycles;
            // Do not draw cycles automatically; only render the list for user selection
            this.renderCyclesList(sortedCycles);
        }
    }

    // Helper to assign colors consistently per cycle index
    private getCycleColor(index: number): string {
        const cycleColors: string[] = [
            "red", "blue", "green", "orange", "purple", "darkred", "darkblue", "darkgreen",
            "darkorange", "magenta", "cyan", "lime", "pink", "brown", "navy"
        ];

        return cycleColors[index % cycleColors.length];
    }

    // Draw cycles on the map and keep references for interactivity
    private drawCyclesOnMap(cycles: L.LatLngExpression[][]): void {
        // Clear previous cycles
        this.cyclePolylines.forEach(polyline => polyline.remove());
        this.cyclePolylines.clear();

        cycles.forEach((cycle, index) => {
            const color = this.getCycleColor(index);
            const polyline = L.polyline(cycle, {
                color: color,
                weight: 6,
                opacity: 0.9,
            }).addTo(this.map);

            this.cyclePolylines.set(index, polyline);
        });
    }

    // Build the UI list of cycles; draw only when user clicks an item
    private renderCyclesList(cycles: L.LatLngExpression[][]): void {
        const cyclesContainer = document.getElementById('cycles-container');
        const cyclesList = document.getElementById('cycles-list');
        const diffPanel = document.getElementById('cycle-diff');

        if (cyclesContainer) {
            cyclesContainer.innerHTML = '';
        }

        if (cycles && cycles.length > 0 && cyclesContainer) {
            // Clear any previously drawn cycle polylines from the map
            this.cyclePolylines.forEach(polyline => polyline.remove());
            this.cyclePolylines.clear();

            cycles.forEach((cycle, index) => {
                const color = this.getCycleColor(index);
                const distance = this.calculateCycleDistance(cycle);

                const cycleItem = document.createElement('div');
                cycleItem.className = 'cycle-item';
                cycleItem.innerHTML = `
                            <span class="cycle-color-box" style="background-color: ${color};"></span>
                            <span>Cycle ${index + 1} (${distance.toFixed(2)} km)</span>
                        `;

                // On click: show diff with next cycle in list (wrap-around)
                cycleItem.addEventListener('click', () => {
                    // Clear any existing displayed cycle(s)
                    this.cyclePolylines.forEach(polyline => polyline.remove());
                    this.cyclePolylines.clear();

                    // Draw only the selected cycle in red
                    const selected = cycles[index];
                    const polyline = L.polyline(selected, {
                        color: 'red',
                        weight: 6,
                        opacity: 0.9,
                    }).addTo(this.map);
                    this.cyclePolylines.set(index, polyline);

                    if (!this.lastCycles || this.lastCycles.length === 0 || !diffPanel) return;
                    const a = this.lastCycles[index];
                    const b = this.lastCycles[(index + 1) % this.lastCycles.length];
                    const html = this.buildCycleDiffHtml(a, b, index, (index + 1) % this.lastCycles.length);
                    diffPanel.innerHTML = html;
                    diffPanel.classList.add('visible');
                });

                cyclesContainer.appendChild(cycleItem);
            });

            if (cyclesList) cyclesList.classList.add('visible');
        }
    }

    // Convert a LatLngExpression to a compact string for comparison
    private coordKey(exp: L.LatLngExpression): string {
        const [lat, lon] = exp as [number, number];
        return `${lat.toFixed(5)},${lon.toFixed(5)}`;
    }

    private buildCycleDiffHtml(a: L.LatLngExpression[], b: L.LatLngExpression[], idxA: number, idxB: number): string {
        const aKeys = a.map(x => this.coordKey(x));
        const bKeys = b.map(x => this.coordKey(x));

        // index-by-index mismatches (up to min length)
        const minLen = Math.min(aKeys.length, bKeys.length);
        const mismatches: {i:number, a?:string, b?:string}[] = [];
        for (let i = 0; i < minLen; i++) {
            if (aKeys[i] !== bKeys[i]) {
                mismatches.push({i, a: aKeys[i], b: bKeys[i]});
            }
        }
        // Unique elements
        const setA = new Set(aKeys);
        const setB = new Set(bKeys);
        const onlyA: string[] = [];
        const onlyB: string[] = [];
        setA.forEach(k => { if (!setB.has(k)) onlyA.push(k); });
        setB.forEach(k => { if (!setA.has(k)) onlyB.push(k); });
        const commonCount = aKeys.filter(k => setB.has(k)).length;

        const previewLen = 8;
        const previewA = aKeys.slice(0, previewLen).join(' -> ');
        const previewB = bKeys.slice(0, previewLen).join(' -> ');

        const mismatchPreview = mismatches.slice(0, 6).map(m => `#${m.i}: <code>${m.a}</code> vs <code>${m.b}</code>`).join('<br/>');

        return `
            <h4>Traversal diff: Cycle ${idxA + 1} vs Cycle ${idxB + 1}</h4>
            <div class="diff-section">Lengths: A=${aKeys.length}, B=${bKeys.length} | Common points: ${commonCount}</div>
            <div class="diff-section">Only in A (${onlyA.length}): ${onlyA.slice(0,6).map(x=>`<code>${x}</code>`).join(', ')}${onlyA.length>6?' …':''}</div>
            <div class="diff-section">Only in B (${onlyB.length}): ${onlyB.slice(0,6).map(x=>`<code>${x}</code>`).join(', ')}${onlyB.length>6?' …':''}</div>
            <div class="diff-section">Index mismatches (${mismatches.length}):<br/>${mismatchPreview || '—'}</div>
            <div class="diff-section">Preview A: <code>${previewA}${aKeys.length>previewLen?' …':''}</code></div>
            <div class="diff-section">Preview B: <code>${previewB}${bKeys.length>previewLen?' …':''}</code></div>
        `;
    }

    private calculateCycleDistance(cycle: L.LatLngExpression[]): number {
        if (cycle.length < 2) return 0;

        const totalMeters = cycle.slice(1).reduce((sum, to, idx) => {
            const from = cycle[idx] as [number, number];
            const [lat1, lon1] = from;
            const [lat2, lon2] = to as [number, number];
            return sum + Graph.calculateDistance(lat1, lon1, lat2, lon2);
        }, 0);

        return totalMeters / 1000; // Convert to km
    }

    // Order cycles by total distance (ascending)
    private sortCyclesByDistance(cycles: L.LatLngExpression[][]): L.LatLngExpression[][] {
        const withDistances = cycles.map(cycle => ({
            cycle,
            distance: this.calculateCycleDistance(cycle)
        }));

        withDistances.sort((a, b) => a.distance - b.distance);

        return withDistances.map(item => item.cycle);
    }

    private addMarker(latLng: LatLng): void {
        console.log(`Clicked at: ${latLng.lat.toFixed(6)}, ${latLng.lng.toFixed(6)}`);

        this.coordinates.push({
            lat: latLng.lat,
            lng: latLng.lng
        });

        L.marker(latLng).addTo(this.map);
    }

    private async addNextMarker(latLng: LatLng): Promise<void> {
        this.addMarker(latLng);

        const [coordFrom, coordTo] = this.coordinates.slice(-2);

        if (coordFrom && coordTo) {
            await this.traceShortestPath(coordFrom, coordTo);
        }
    }

    private async handleMapClick(e: L.LeafletMouseEvent): Promise<void> {
        if (this.mode === 'draw') {
            await this.addNextMarker(e.latlng);
        } else {
            this.addMarker(e.latlng);
            await this.traceLoopPath({lat: e.latlng.lat, lng: e.latlng.lng});
        }
    }

    public destroy() {
        this.abortController.abort();
    }
}
