import L, {LatLng} from "leaflet";
import {fetchBoundingBoxNetwork} from "@/api";
import {Graph} from "@/graph";
import type {Coordinate} from "@/types/map_types";
import type {SharedMapActions} from "@/features/mapActions";

interface AutoExploreDeps extends SharedMapActions {
    map: L.Map;
}

export class AutoExploreFeature {
    private readonly map: L.Map;
    private readonly addMarker: SharedMapActions["addMarker"];
    private readonly createBoundingBox: SharedMapActions["createBoundingBox"];
    private readonly cyclePolylines: Map<number, L.Polyline> = new Map();
    private lastCycles: L.LatLngExpression[][] = [];

    constructor(deps: AutoExploreDeps) {
        this.map = deps.map;
        this.addMarker = deps.addMarker;
        this.createBoundingBox = deps.createBoundingBox;
    }

    public reset(): void {
        this.cyclePolylines.forEach((polyline) => polyline.remove());
        this.cyclePolylines.clear();
        this.lastCycles = [];

        const cyclesContainer = document.getElementById("cycles-container");
        const cyclesList = document.getElementById("cycles-list");
        const diffPanel = document.getElementById("cycle-diff");

        if (cyclesContainer) cyclesContainer.innerHTML = "";
        if (cyclesList) cyclesList.classList.remove("visible");
        if (diffPanel) {
            diffPanel.classList.remove("visible");
            diffPanel.innerHTML = "";
        }
    }

    public async onMapClick(latLng: LatLng): Promise<void> {
        const coordinate = this.addMarker(latLng);
        await this.traceLoopPath(coordinate);
    }

    private async traceLoopPath(coord: Coordinate): Promise<void> {
        const boundingBox = this.createBoundingBox(coord, coord);
        const {nodes, ways} = await fetchBoundingBoxNetwork(boundingBox);
        const graph = new Graph();

        graph.build(nodes, ways);

        const startNodeId = graph.findNearestNode(coord);
        const cyclesResult = graph.findCycles(startNodeId);
        const cycles = cyclesResult.cycles;

        if (!cycles || cycles.length === 0) return;

        const sortedCycles = this.sortCyclesByDistance(cycles);
        this.lastCycles = sortedCycles;
        this.renderCyclesList(sortedCycles);
    }

    private getCycleColor(index: number): string {
        const cycleColors = [
            "red", "blue", "green", "orange", "purple", "darkred", "darkblue", "darkgreen",
            "darkorange", "magenta", "cyan", "lime", "pink", "brown", "navy"
        ];

        return cycleColors[index % cycleColors.length] ?? "red";
    }

    private renderCyclesList(cycles: L.LatLngExpression[][]): void {
        const cyclesContainer = document.getElementById("cycles-container");
        const cyclesList = document.getElementById("cycles-list");
        const diffPanel = document.getElementById("cycle-diff");

        if (!cyclesContainer) return;

        cyclesContainer.innerHTML = "";
        this.cyclePolylines.forEach((polyline) => polyline.remove());
        this.cyclePolylines.clear();

        cycles.forEach((cycle, index) => {
            const color = this.getCycleColor(index);
            const distance = this.calculateCycleDistance(cycle);

            const cycleItem = document.createElement("div");
            cycleItem.className = "cycle-item";
            cycleItem.innerHTML = `
                <span class="cycle-color-box" style="background-color: ${color};"></span>
                <span>Cycle ${index + 1} (${distance.toFixed(2)} km)</span>
            `;

            cycleItem.addEventListener("click", () => {
                this.cyclePolylines.forEach((polyline) => polyline.remove());
                this.cyclePolylines.clear();

                const selected = cycles[index];
                if (!selected) return;
                const polyline = L.polyline(selected, {
                    color: "red",
                    weight: 6,
                    opacity: 0.9,
                }).addTo(this.map);
                this.cyclePolylines.set(index, polyline);

                if (!this.lastCycles || this.lastCycles.length === 0 || !diffPanel) return;
            });

            cyclesContainer.appendChild(cycleItem);
        });

        if (cyclesList) cyclesList.classList.add("visible");
    }

    private calculateCycleDistance(cycle: L.LatLngExpression[]): number {
        if (cycle.length < 2) return 0;

        const totalMeters = cycle.slice(1).reduce((sum, to, idx) => {
            const [lat1, lon1] = cycle[idx] as [number, number];
            const [lat2, lon2] = to as [number, number];
            return sum + Graph.calculateDistance(lat1, lon1, lat2, lon2);
        }, 0);

        return totalMeters / 1000;
    }

    private sortCyclesByDistance(cycles: L.LatLngExpression[][]): L.LatLngExpression[][] {
        const withDistances = cycles.map((cycle) => ({
            cycle,
            distance: this.calculateCycleDistance(cycle),
        }));

        withDistances.sort((a, b) => a.distance - b.distance);

        return withDistances.map((item) => item.cycle);
    }
}
