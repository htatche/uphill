export interface Coordinate {
  lat: number;
  lng: number;
  marker: L.Marker;
}
export interface BoundingBox {
  north: number;
  east: number;
  south: number;
  west: number;
}