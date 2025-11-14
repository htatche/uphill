import type { BoundingBox } from "@/types/map";
import type { OSMNode, OSMWay, OverpassResponse } from "@/types/overpass";

const OVERPASS_API_URL = "https://overpass-api.de/api/interpreter";

export async function fetchBoundingBoxNetwork(
  bounding_box: BoundingBox
): Promise<{ nodes: OSMNode[]; ways: OSMWay[] }> {
  const query = buildNetworkQuery(bounding_box);

  console.log("Fetching trail network in bounding box:", bounding_box);

  try {
    const response = await fetch(OVERPASS_API_URL, {
      method: "POST",
      body: query,
    });

    if (!response.ok) {
      console.error("Overpass API error:", response.statusText);
      return { nodes: [], ways: [] };
    }

    const data: OverpassResponse = await response.json();

    const nodes = data.elements.filter(
      (el): el is OSMNode => el.type === "node"
    );
    const ways = data.elements.filter((el): el is OSMWay => el.type === "way");

    console.log(`Fetched ${nodes.length} nodes and ${ways.length} ways`);

    return { nodes, ways };
  } catch (error) {
    console.error("Error fetching trail network:", error);
    return { nodes: [], ways: [] };
  }
}

export function buildNetworkQuery(bounding_box: BoundingBox): string {
  const coords = [
    bounding_box.south,
    bounding_box.west,
    bounding_box.north,
    bounding_box.east,
  ].join(",");

  return `
    [out:json];
    (
      way["highway"="path"](${coords});
      way["highway"="footway"](${coords});
      way["highway"="track"](${coords});
      way["highway"="cycleway"](${coords});
      way["highway"="bridleway"](${coords});
      way["route"="hiking"](${coords});
      way["route"="foot"](${coords});
      way["sac_scale"](${coords});
    );
    out body;
    >;
    out skel qt;
  `;
}
