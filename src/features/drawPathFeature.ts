import L, {LatLng} from "leaflet";
import {fetchBoundingBoxNetwork} from "@/api";
import {Graph} from "@/graph";
import type {Coordinate} from "@/types/map_types";
import type {OSMNode, OSMWay} from "@/types/osm_types";
import type {DrawPathMapActions} from "@/features/mapActions";

interface DrawPathDeps extends DrawPathMapActions {
    map: L.Map;
}

export class DrawPathFeature {
    private readonly map: L.Map;
    private readonly addMarker: DrawPathMapActions["addMarker"];
    private readonly createBoundingBox: DrawPathMapActions["createBoundingBox"];
    private readonly drawBoundingBox: DrawPathMapActions["drawBoundingBox"];
    private readonly coordinates: Coordinate[] = [];
    private currentPathCoordinates: L.LatLngExpression[] = [];

    constructor(deps: DrawPathDeps) {
        this.map = deps.map;
        this.addMarker = deps.addMarker;
        this.createBoundingBox = deps.createBoundingBox;
        this.drawBoundingBox = deps.drawBoundingBox;
    }

    public reset(): void {
        this.coordinates.length = 0;
        this.currentPathCoordinates = [];
    }

    public async onMapClick(latLng: LatLng): Promise<void> {
        const coordinate = this.addMarker(latLng);
        this.coordinates.push(coordinate);

        const [coordFrom, coordTo] = this.coordinates.slice(-2);

        if (coordFrom && coordTo) {
            await this.traceShortestPath(coordFrom, coordTo);
        }
    }

    private drawPaths(nodes: OSMNode[], ways: OSMWay[]): void {
        const nodeMap: Record<number, L.LatLngExpression> = {};
        nodes.forEach((node) => {
            nodeMap[node.id] = [node.lat, node.lon];
        });

        const colors = ["pink", "blue", "purple", "orange", "green"];

        ways.forEach((way, index) => {
            const coords = way.nodes
                .map((id) => nodeMap[id])
                .filter((coord): coord is L.LatLngExpression => coord !== undefined);

            if (coords.length > 1) {
                L.polyline(coords, {
                    color: colors[index % colors.length],
                    weight: 4,
                    opacity: 0.8,
                }).addTo(this.map);
            }
        });
    }

    private async exploreBoundingBox(
        coordFrom: Coordinate,
        coordTo: Coordinate
    ): Promise<{nodes: OSMNode[]; ways: OSMWay[]}> {
        const boundingBox = this.createBoundingBox(coordFrom, coordTo);
        const {nodes, ways} = await fetchBoundingBoxNetwork(boundingBox);

        this.drawBoundingBox(boundingBox);
        this.drawPaths(nodes, ways);

        return {nodes, ways};
    }

    private async traceShortestPath(coordFrom: Coordinate, coordTo: Coordinate): Promise<void> {
        const {nodes, ways} = await this.exploreBoundingBox(coordFrom, coordTo);
        const graph = new Graph();

        graph.build(nodes, ways);

        const [nodeFrom, nodeTo] = graph.translateCoordsToNodes(coordFrom, coordTo);

        if (!nodeFrom || !nodeTo) return;
        const pathCoordinates = graph.findShortestPath(nodeFrom, nodeTo);

        if (!pathCoordinates) return;

        this.currentPathCoordinates = [...this.currentPathCoordinates, ...pathCoordinates];

        L.polyline(this.currentPathCoordinates, {color: "red", weight: 4}).addTo(this.map);
    }
}
