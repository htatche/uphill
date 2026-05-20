import type { BoundingBox } from "@/types/map_types";
import type { OSMNode, OSMWay, OverpassResponse } from "@/types/osm_types";

const OVERPASS_API_URL = "https://overpass.kumi.systems/api/interpreter";
const OVERPASS_QUERY_TIMEOUT_SECONDS = 25;
const BROWSER_REQUEST_TIMEOUT_MS = (OVERPASS_QUERY_TIMEOUT_SECONDS + 5) * 1000;
const MAX_REQUEST_ATTEMPTS = 3;

type TrailNetwork = { nodes: OSMNode[]; ways: OSMWay[] };
type FetchStatus = "loading" | "success" | "retry" | "error";

let statusHideTimeoutId: number | undefined;

function updateQueryStatus(message: string, status: FetchStatus): void {
  const statusElement = document.getElementById("loading-status");
  const messageElement = document.getElementById("loading-status-message");

  if (!statusElement || !messageElement) return;

  window.clearTimeout(statusHideTimeoutId);

  messageElement.textContent = message;
  statusElement.classList.add("visible");
  statusElement.classList.toggle("status-error", status === "error");
  statusElement.classList.toggle("status-success", status === "success");

  if (status === "success" || status === "error") {
    statusHideTimeoutId = window.setTimeout(() => {
      statusElement.classList.remove("visible", "status-error", "status-success");
      messageElement.textContent = "";
    }, 2500);
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

async function fetchOverpass(query: string): Promise<OverpassResponse> {
  const controller = new AbortController();
  const timeoutId = window.setTimeout(
    () => controller.abort(),
    BROWSER_REQUEST_TIMEOUT_MS,
  );

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
        `Overpass API returned ${response.status}: ${details.slice(0, 300).trim()}`,
      );
    }

    return (await response.json()) as OverpassResponse;
  } finally {
    window.clearTimeout(timeoutId);
  }
}

async function fetchTrailNetworkAttempt(
  query: string,
  requestStartedAt: number,
): Promise<TrailNetwork> {
  const data = await fetchOverpass(query);
  const elapsedMs = Math.round(performance.now() - requestStartedAt);
  const nodes = data.elements.filter((el): el is OSMNode => el.type === "node");
  const ways = data.elements.filter((el): el is OSMWay => el.type === "way");

  console.log(
    `Fetched ${nodes.length} nodes and ${ways.length} ways in ${elapsedMs}ms`,
  );

  return { nodes, ways };
}

async function fetchTrailNetworkWithRetries(
  query: string,
): Promise<TrailNetwork> {
  for (let attempt = 1; attempt <= MAX_REQUEST_ATTEMPTS; attempt++) {
    const requestStartedAt = performance.now();
    const attemptMessage = `Querying Overpass (${attempt}/${MAX_REQUEST_ATTEMPTS})`;

    try {
      console.log(`${attemptMessage}, timeout ${BROWSER_REQUEST_TIMEOUT_MS}ms`);
      updateQueryStatus(attemptMessage, "loading");

      const trailNetwork = await fetchTrailNetworkAttempt(query, requestStartedAt);
      updateQueryStatus("Trail network loaded", "success");

      return trailNetwork;
    } catch (error) {
      const elapsedMs = Math.round(performance.now() - requestStartedAt);
      const hasAttemptsRemaining = attempt < MAX_REQUEST_ATTEMPTS;

      console.warn(
        `Overpass request failed (attempt ${attempt}/${MAX_REQUEST_ATTEMPTS}, ${elapsedMs}ms):`,
        error,
      );

      if (hasAttemptsRemaining) {
        updateQueryStatus(`Retrying Overpass (${attempt + 1}/${MAX_REQUEST_ATTEMPTS})`, "retry");
      } else {
        updateQueryStatus("Trail network request failed", "error");
        console.error("Error fetching trail network:", error);
        return { nodes: [], ways: [] };
      }
    }
  }

  return { nodes: [], ways: [] };
}

export async function fetchBoundingBoxNetwork(
  bounding_box: BoundingBox,
): Promise<TrailNetwork> {
  const query = buildNetworkQuery(bounding_box);

  console.log(`Generated Overpass query: ${query}`);
  console.log("Fetching trail network in bounding box:", bounding_box);

  return fetchTrailNetworkWithRetries(query);
}
