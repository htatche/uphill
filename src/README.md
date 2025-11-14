# Technical architecture

## Code path

1. Place waypoint
2. Place second waypoint
4. Pull data from Overpass within a bound box that spans the two waypoints
5. Create a Graph with edges/nodes returned
6. Apply path finding algorithm