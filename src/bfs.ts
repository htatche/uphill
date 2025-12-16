function bfs(graph: Map<string, string[]>, start: string): string[] {
    const visited = new Set<string>();
    const queue: string[] = [];
    const result: string[] = [];

    visited.add(start);
    queue.push(start);

    while (queue.length > 0) {
        const node = queue.shift()!;
        result.push(node);

        const neighbours = graph.get(node);
        if (!neighbours) continue;

        for (let neighbour of neighbours) {
            if (!visited.has(neighbour)) {
                visited.add(neighbour);
                queue.push(neighbour);
            }
        }
    }

    return result;
}

//         0
//      /  |  \
//     1   2   3
//         |    \
//         4     5
const graph: Map<string, string[]> = new Map([
    ["0", ["1", "2", "3"]],
    ["1", ["0", "2", "3"]],
    ["2", ["0", "3", "4"]],
    ["3", ["1", "2", "5"]],
    ["4", ["2"]],
    ["5", ["3"]]
]);

const start: string = "0";
console.log(bfs(graph, start));