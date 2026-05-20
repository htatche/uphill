import type { BoundingBox } from "@/types/map_types";
import type { OSMNode, OSMWay, OverpassResponse } from "@/types/osm_types";

const OVERPASS_API_URL = "https://overpass.kumi.systems/api/interpreter";
const OVERPASS_QUERY_TIMEOUT_SECONDS = 25;
const REQUEST_TIMEOUT_MS = (OVERPASS_QUERY_TIMEOUT_SECONDS + 5) * 1000;

async function fetchOverpass(query: string): Promise<OverpassResponse> {
  const controller = new AbortController();
  const timeoutId = window.setTimeout(() => controller.abort(), REQUEST_TIMEOUT_MS);

  try {
    const response = await fetch(OVERPASS_API_URL, {
      method: "POST",
      body: query,
      signal: controller.signal,
    });

    if (!response.ok) {
      const message = await response.text();
      const details = message || response.statusText;
      throw new Error(
        `Overpass API returned ${response.status}: ${details.slice(0, 300).trim()}`
      );
    }

    return await response.json() as OverpassResponse;
  } finally {
    window.clearTimeout(timeoutId);
  }
}

export async function fetchBoundingBoxNetwork(
  bounding_box: BoundingBox
): Promise<{ nodes: OSMNode[]; ways: OSMWay[] }> {
  const query = buildNetworkQuery(bounding_box);

  console.log(`Generated Overpass query: ${query}`);
  console.log("Fetching trail network in bounding box:", bounding_box);

  try {
    const requestStartedAt = performance.now();
    const data = await fetchOverpass(query);
    const elapsedMs = Math.round(performance.now() - requestStartedAt);
    const nodes = data.elements.filter(
      (el): el is OSMNode => el.type === "node"
    );
    const ways = data.elements.filter((el): el is OSMWay => el.type === "way");

    console.log(`Fetched ${nodes.length} nodes and ${ways.length} ways in ${elapsedMs}ms`);

    return { nodes, ways };
  } catch (error) {
    console.error("Error fetching trail network:", error);
    return { nodes: [], ways: [] };
  }
}

function buildNetworkQuery(bounding_box: BoundingBox): string {
  const coords = [
    bounding_box.south,
    bounding_box.west,
    bounding_box.north,
    bounding_box.east,
  ].join(",");

  return `
    [out:json][timeout:${OVERPASS_QUERY_TIMEOUT_SECONDS}];
    (
      way["highway"~"^(path|footway|track|cycleway|bridleway)$"](${coords});
      way["route"~"^(hiking|foot)$"](${coords});
      way["sac_scale"](${coords});
    );
    out body qt;
    >;
    out skel qt;
  `;
}
