export function depthFirstSearchWithStack(adjacencyList: Map<string, string[]>, startNodeId: string): string[] {
    let visited: boolean[] = [];// = new Array(adjacencyList.size).fill(false);
    let traversalOrder: string[] = [];// = new Array(adjacencyList.size).fill(-1);
    let toExplore: string[] = [];

    toExplore.push(startNodeId);

    while (toExplore.length > 0) {
        const index = toExplore.pop();
        if (!visited[index]) {
            visited[index] = true;

            const neighbours = adjacencyList.get(index)
            neighbours.reverse(); // TODO Is it being modified?

            for (const neighbour of neighbours) {
                if (!visited[neighbour]) {
                    traversalOrder[neighbour] = index
                    toExplore.push(neighbour);
                }
            }
        }
    }

    return traversalOrder;
}
