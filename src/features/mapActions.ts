import L, {LatLng, Polygon} from "leaflet";
import type {BoundingBox, Coordinate} from "@/types/map_types";

export interface SharedMapActions {
    addMarker: (latLng: LatLng) => Coordinate;
    createBoundingBox: (coordFrom: Coordinate, coordTo: Coordinate) => BoundingBox;
}

export interface DrawPathMapActions extends SharedMapActions {
    drawBoundingBox: (boundingBox: BoundingBox) => void;
}

export class MapActions implements DrawPathMapActions {
    private currentBoundingBoxPolygon: Polygon | undefined;

    public constructor(private readonly map: L.Map) {
    }

    public reset(): void {
        this.currentBoundingBoxPolygon?.remove();
        this.currentBoundingBoxPolygon = undefined;
    }

    public addMarker(latLng: LatLng): Coordinate {
        console.log(`Clicked at: ${latLng.lat.toFixed(6)}, ${latLng.lng.toFixed(6)}`);

        L.marker(latLng).addTo(this.map);

        return {
            lat: latLng.lat,
            lng: latLng.lng,
        };
    }

    public createBoundingBox(coordFrom: Coordinate, coordTo: Coordinate): BoundingBox {
        const padding = 0.05;

        return {
            south: Math.min(coordFrom.lat, coordTo.lat) - padding,
            west: Math.min(coordFrom.lng, coordTo.lng) - padding,
            north: Math.max(coordFrom.lat, coordTo.lat) + padding,
            east: Math.max(coordFrom.lng, coordTo.lng) + padding,
        };
    }

    public drawBoundingBox(boundingBox: BoundingBox): void {
        const {south, west, north, east} = boundingBox;
        const corners: L.LatLngExpression[] = [
            [south, west],
            [south, east],
            [north, east],
            [north, west],
            [south, west],
        ];

        const polygon = L.polygon(corners, {
            color: "red",
            weight: 2,
            fill: false,
        });

        this.currentBoundingBoxPolygon?.remove();
        this.currentBoundingBoxPolygon = polygon.addTo(this.map);
    }
}
