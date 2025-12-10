"""

This generates vehicle routes as a graph of nodes (A–I) and edges.

Main Features:
1. Defines nodes, edges, and boundary nodes of the network.
2. Recursively computes all possible routes up to a given depth
   while avoiding cycles, and stores valid boundary-to-boundary paths.
3. Specifies multiple vehicle types (fast car, slow car, ambulance)
   with different speed and acceleration parameters.
4. Creates route definitions for normal vehicles as well as
   special hardcoded ambulance parking routes.
5. Exports all routes into a SUMO-compatible XML file
   (`vehicles.rou.xml`) with proper formatting.
   
"""


import xml.etree.ElementTree as ET  

# ----------------------------
# Define nodes in the road network (graph representation)
nodes = ["A","B","C","D","E","F","G","H","I"]

# Define edges (connections) between nodes
edges = {
    "A": ["B", "D"],
    "B": ["A", "C", "E"],
    "C": ["B", "F"],
    "D": ["A", "E", "G"],
    "E": ["B", "D", "F", "H"],
    "F": ["C", "E", "I"],
    "G": ["D", "H"],
    "H": ["G", "E", "I"],
    "I": ["F", "H"]
}

# Boundary nodes (routes should end when reaching these)
boundary_nodes = ["A", "B", "C", "D", "F", "G", "H", "I"]

# Recursive function to find valid routes in the graph
def find_routes(start, path=None, max_depth=6):
    if path is None:
        path = [start]          # Initialize path with the start node
    else:
        path = path + [start]   # Extend path with the current node

    routes = []
    # If we reach a boundary node (and not the starting point), save the route
    if start in boundary_nodes and len(path) > 1:
        routes.append(path)
    # Stop recursion if maximum path length is reached
    if len(path) >= max_depth:
        return routes
    # Explore neighbors recursively, avoiding cycles
    for neighbor in edges[start]:
        if neighbor not in path:
            routes.extend(find_routes(neighbor, path, max_depth))
    return routes

# Generate all normal routes by starting search from each node
all_routes = []
for node in nodes:
    all_routes.extend(find_routes(node))

# ----------------------------
# SUMO XML generation (routes file structure)
routes_root = ET.Element("routes")   # Root element <routes>

# Define different vehicle types with their parameters
vtypes = [
    {"id": "fastCar", "accel":"4.0","decel":"6.0","sigma":"0.5","length":"5","maxSpeed":"20.0","color":"0,1,0","guiShape":"passenger/sedan"},
    {"id": "slowCar", "accel":"2.0","decel":"4.0","sigma":"0.5","length":"5","maxSpeed":"10.0","color":"0,0,1","guiShape":"passenger/hatchback"},
    {"id": "ambulance", "accel":"3.0","decel":"6.0","sigma":"0.5","length":"10","maxSpeed":"13.9","color":"1,1,1","guiShape":"emergency"}
]

# Add vehicle types to the XML
for vt in vtypes:
    ET.SubElement(routes_root, "vType", vt)

# Add normal routes to the XML
for idx, route in enumerate(all_routes):
    # Convert node sequence into edge format (e.g., A_B B_C ...)
    edges_str = " ".join([f"{route[i]}_{route[i+1]}" for i in range(len(route)-1)])
    ET.SubElement(routes_root, "route", id=f"route{idx}", edges=edges_str)

# ----------------------------
# Hardcoded ambulance parking routes (special cases)
ambulance_parking_routes = ["A_B_parking", "D_G_parking", "H_I_parking"]
for i, edge in enumerate(ambulance_parking_routes):
    ET.SubElement(routes_root, "route", id=f"routeAmbulance{i}", edges=edge)

# Add formatting (newlines) for better readability of XML
routes_root.text = "\n"
for child in routes_root:
    child.tail = "\n"

# Write XML tree to a file (SUMO route file)
tree = ET.ElementTree(routes_root)
tree.write("vehicles.rou.xml", encoding="UTF-8", xml_declaration=True)

# Print summary of how many routes were generated
print(f"Generated {len(all_routes)} normal routes + {len(ambulance_parking_routes)} ambulance parking routes")
