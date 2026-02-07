import json

import pandas as pd  # type: ignore

csv_filename = 'airport_map_big.csv'
df = pd.read_csv(csv_filename, header=None, sep=';')

data = df.values
rows, cols = data.shape

nodes = []
node_map = {}
current_id = 0

for r in range(rows):
    for c in range(cols):
        val = data[r, c]
        if val != 999:
            node_info = {
                "id": current_id,
                "x": c,
                "y": 0,
                "z": rows - 1 - r,
                "type": int(val),
                "neighbors": []
            }
            nodes.append(node_info)
            node_map[(r, c)] = current_id
            current_id += 1

directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

for node in nodes:
    r = rows - 1 - node["z"]
    c = node["x"]
    my_id = node["id"]

    for dr, dc in directions:
        nr, nc = r + dr, c + dc
        if (nr, nc) in node_map:
            neighbor_id = node_map[(nr, nc)]
            node["neighbors"].append(neighbor_id)

output_json = {
    "nodes": nodes
}

output_filename = 'airport.json'
with open(output_filename, 'w') as f:
    json.dump(output_json, f, indent=4)

print(f"File {output_filename} created with {len(nodes)} nodes.")
print("Sample node:", nodes[0])
