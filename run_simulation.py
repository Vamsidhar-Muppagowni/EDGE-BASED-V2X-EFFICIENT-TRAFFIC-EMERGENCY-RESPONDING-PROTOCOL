import sys
import os
import time
import random
import xml.etree.ElementTree as ET
import math
import uuid
from collections import defaultdict
from cen_broadcast import CENBroadcast
from vehicle import Vehicle
import traci
import sumolib

# -------------------- SUMO PATH SETUP --------------------
if 'SUMO_HOME' not in os.environ:
    os.environ['SUMO_HOME'] = "/usr/share/sumo"

tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
if tools not in sys.path:
    sys.path.append(tools)
from best_erv import select_best_ambulance
# -------------------- SUMO SETUP --------------------
sumoBinary = sumolib.checkBinary("sumo-gui")
sumoConfig = "simulation.sumocfg"
vehicles_dict = {}
V2V_COMMUNICATION_RANGE = 200.0
MAX_HOP_COUNT = 5
COLLISION_DISTANCE = 7.5

# Edge node positions (fixed infrastructure nodes)
EDGE_NODE_POSITIONS = {
    "EdgeNode_A": (30, 20),
    "EdgeNode_D": (30, 120),
    "EdgeNode_C": (200,180),
    "EdgeNode_I": (200,20)
}

# Global state
edge_nodes = {}
broadcasted_accidents = set()
message_history = {}
hop_count_stats = defaultdict(int)
reported_collisions = set()
total_accidents = 0
successful_notifications = 0

# -------------------- PARSE ROUTES AND VEHICLE TYPES --------------------
rou_file = "vehicles.rou.xml"
tree = ET.parse(rou_file)
root = tree.getroot()
routes = [r.attrib['id'] for r in root.findall("route") if not r.attrib['id'].startswith('routeAmbulance')]
vtypes = {v.attrib['id']: v for v in root.findall("vType") if v.attrib['id'] != 'ambulance'}

print("Loaded routes:", len(routes))
print("Loaded vehicle types:", list(vtypes.keys()))

NUM_INITIAL_VEHICLES = 4
SPAWN_INTERVAL = 20
VEHICLE_COUNTER = 0

# -------------------- V2V MESSAGE CLASS --------------------
class V2VMessage:
    def __init__(self, message_id, source_id, message_type, payload, origin_location):
        self.message_id = message_id
        self.source_id = source_id
        self.message_type = message_type
        self.payload = payload
        self.origin_location = origin_location
        self.hop_count = 0
        self.propagation_path = [source_id]
        self.timestamp = traci.simulation.getTime()
        self.reached_edge_node = False

    def increment_hop(self, next_node):
        self.hop_count += 1
        self.propagation_path.append(next_node)

# -------------------- INITIALIZE EDGE NODES --------------------
def initialize_edge_nodes():
    print("EDGE NODE INITIALIZATION")
    print("=" * 50)
    for node_id, position in EDGE_NODE_POSITIONS.items():
        edge_nodes[node_id] = {
            'position': position,
            'connected_vehicles': set(),
            'message_cache': set(),
            'total_messages_received': 0,
            'unique_accidents_reported': set(),
            'accident_alerts_received': []
        }
        print(f"Edge Node: {node_id} - Position: {position}")
    print("=" * 50)

# -------------------- HELPER FUNCTIONS --------------------
# -------------------- HELPER FUNCTIONS WITH LOGGING --------------------
def get_vehicle_position(vehicle_id):
    try:
        if vehicle_id in traci.vehicle.getIDList():
            return traci.vehicle.getPosition(vehicle_id)
    except:
        pass
    return None

def calculate_distance(pos1, pos2):
    if pos1 and pos2:
        return math.hypot(pos1[0] - pos2[0], pos1[1] - pos2[1])
    return float('inf')

def find_vehicles_in_range(source_position, exclude_vehicle=None):
    vehicles_in_range = []
    for vehicle_id in traci.vehicle.getIDList():
        if vehicle_id == exclude_vehicle or vehicle_id.startswith('ambulance'):
            continue
        vehicle_pos = get_vehicle_position(vehicle_id)
        if vehicle_pos:
            distance = calculate_distance(source_position, vehicle_pos)
            if distance <= V2V_COMMUNICATION_RANGE:
                vehicles_in_range.append((vehicle_id, distance, vehicle_pos))
    vehicles_in_range.sort(key=lambda x: x[1])
    if vehicles_in_range:
        print(f"    Vehicles in range of {exclude_vehicle}: {[v[0] for v in vehicles_in_range]}")
    return vehicles_in_range

def find_edge_nodes_in_range(position):
    edge_nodes_in_range = []
    for edge_id, edge_info in edge_nodes.items():
        distance = calculate_distance(position, edge_info['position'])
        if distance <= V2V_COMMUNICATION_RANGE:
            edge_nodes_in_range.append((edge_id, distance))
    edge_nodes_in_range.sort(key=lambda x: x[1])
    if edge_nodes_in_range:
        print(f"    Edge nodes in range at pos {position}: {[e[0] for e in edge_nodes_in_range]}")
    return edge_nodes_in_range

def propagate_v2v_message(message, current_vehicle_id, current_position):
    print(f"    [V2V] Vehicle {current_vehicle_id} propagating message {message.message_id}, hop {message.hop_count}")
    if message.hop_count >= MAX_HOP_COUNT:
        print(f"        [V2V] Message {message.message_id} reached max hops ({MAX_HOP_COUNT})")
        return False

    # Check edge nodes in range
    edge_nodes_in_range = find_edge_nodes_in_range(current_position)
    for edge_id, distance in edge_nodes_in_range:
        if message.message_id not in edge_nodes[edge_id]['message_cache']:
            edge_nodes[edge_id]['message_cache'].add(message.message_id)
            edge_nodes[edge_id]['total_messages_received'] += 1
            if message.message_type == "EMERGENCY":
                edge_nodes[edge_id]['unique_accidents_reported'].add(message.payload.get('accident_id'))
                alert_info = {
                    'accident_id': message.payload.get('accident_id'),
                    'vehicles_involved': message.payload.get('vehicles_involved', []),
                    'location': message.payload.get('location'),
                    'timestamp': message.payload.get('timestamp'),
                    'received_at': traci.simulation.getTime(),
                    'hop_count': message.hop_count,
                    'propagation_path': message.propagation_path.copy()
                }
                edge_nodes[edge_id]['accident_alerts_received'].append(alert_info)
            message.reached_edge_node = True
            hop_count_stats[message.hop_count] += 1
            print(f"        [CEN] Edge node {edge_id} received message {message.message_id} about accident {message.payload.get('accident_id')}")
            return True

    # Propagate to vehicles in range
    vehicles_in_range = find_vehicles_in_range(current_position, exclude_vehicle=current_vehicle_id)
    propagation_successful = False
    for vehicle_id, distance, vehicle_pos in vehicles_in_range:
        if vehicle_id in message.propagation_path:
            continue
        next_message = V2VMessage(
            message.message_id,
            vehicle_id,
            message.message_type,
            message.payload,
            message.origin_location
        )
        next_message.hop_count = message.hop_count + 1
        next_message.propagation_path = message.propagation_path.copy()
        next_message.increment_hop(vehicle_id)
        if next_message.message_id not in message_history:
            message_history[next_message.message_id] = []
        message_history[next_message.message_id].append(next_message)
        print(f"        [V2V] Vehicle {current_vehicle_id} sending message {message.message_id} to vehicle {vehicle_id}")
        success = propagate_v2v_message(next_message, vehicle_id, vehicle_pos)
        if success:
            propagation_successful = True
            break
    return propagation_successful

def broadcast_emergency_alert(source_vehicle_id, accident_location, collision_pair, accident_id):
    global successful_notifications
    print(f"[ACCIDENT] Accident {accident_id} occurred between vehicles {collision_pair} at location {accident_location}")
    source_position = get_vehicle_position(source_vehicle_id)
    if not source_position:
        print(f"    [WARNING] Source vehicle {source_vehicle_id} position not found")
        return False
    message_id = f"EMERGENCY_{uuid.uuid4().hex[:8]}"
    emergency_payload = {
        'accident_id': accident_id,
        'vehicles_involved': collision_pair,
        'location': accident_location,
        'severity': 'HIGH',
        'timestamp': traci.simulation.getTime()
    }
    message = V2VMessage(
        message_id,
        source_vehicle_id,
        "EMERGENCY",
        emergency_payload,
        accident_location
    )
    message_history[message_id] = [message]
    success = propagate_v2v_message(message, source_vehicle_id, source_position)
    if success:
        successful_notifications += 1
        print(f"    [SUCCESS] Emergency alert {message_id} successfully propagated to edge nodes")
    else:
        print(f"    [FAILURE] Emergency alert {message_id} propagation failed")
    return success


# -------------------- START SUMO --------------------
traci.start([sumoBinary, "-c", sumoConfig,
             "--collision.action", "none",
             "--collision.check-junctions", "true",
             "--ignore-route-errors", "true"], port=8813)

# -------------------- INITIALIZE EDGE NODES --------------------
initialize_edge_nodes()
cen = CENBroadcast(interval=10, edge_nodes=edge_nodes)

graph = {
    "A": [("B", 100), ("D", 100)],
    "B": [("A", 100), ("C", 100), ("E", 100)],
    "C": [("B", 100), ("F", 100)],
    "D": [("A", 100), ("E", 100), ("G", 100)],
    "E": [("B", 100), ("D", 100), ("F", 100), ("H", 100)],
    "F": [("C", 100), ("E", 100), ("I", 100)],
    "G": [("D", 100), ("H", 100)],
    "H": [("G", 100), ("E", 100), ("I", 100)],
    "I": [("F", 100), ("H", 100)]
}

# -------------------- SPAWN AMBULANCES --------------------
ambulance_parking_routes = {
    "ambulance0": "routeAmbulance0",
    "ambulance1": "routeAmbulance1",
    "ambulance2": "routeAmbulance2"
}

for vid, route in ambulance_parking_routes.items():
    try:
        traci.vehicle.add(vid, routeID=route, typeID="ambulance", depart=0)
        traci.vehicle.setSpeed(vid, 0)
        traci.vehicle.setMaxSpeed(vid, 0)
        traci.vehicle.setEmergencyDecel(vid, 1000)
        traci.vehicle.setTau(vid, 0)
        traci.vehicle.setMinGap(vid, 0)
        parking_edge = traci.route.getEdges(route)[0]
        lane_id = parking_edge + "_0"
        lane_length = traci.lane.getLength(lane_id)
        pos = lane_length / 2.0
        traci.vehicle.setStop(vid, edgeID=parking_edge, pos=pos, duration=1e6)
    except Exception as e:
        print(f"Failed to spawn ambulance {vid}: {e}")


# ----------------------------best_erv part
# Initialize ambulance readiness dictionary
ambulance_readiness = {amb_id: True for amb_id in ambulance_parking_routes.keys()}

# -------------------- SPAWN VEHICLES FUNCTION --------------------
def spawn_vehicle(step):
    global VEHICLE_COUNTER
    vid = f"veh{VEHICLE_COUNTER}"
    VEHICLE_COUNTER += 1
    r = random.choice(routes)
    t = random.choice(list(vtypes.keys()))
    try:
        traci.vehicle.add(vid, routeID=r, typeID=t, depart=step)
        traci.vehicle.setEmergencyDecel(vid, 1000)
        traci.vehicle.setTau(vid, 0)
        traci.vehicle.setMinGap(vid, 0)
        max_speed = float(vtypes[t].attrib.get("maxSpeed", 13.9))
        speed = random.uniform(max_speed*0.3, max_speed)
        traci.vehicle.setSpeed(vid, speed)
        vehicles_dict[vid] = Vehicle(veh_id=vid, destination=traci.vehicle.getRoute(vid)[-1])
    except Exception as e:
        print(f"Failed to spawn vehicle {vid}: {e}")

# -------------------- SPAWN INITIAL VEHICLES --------------------
for _ in range(NUM_INITIAL_VEHICLES):
    spawn_vehicle(step=0)

# -------------------- SIMULATION LOOP --------------------
step = 0
MAX_STEPS = 500
while step < MAX_STEPS:
    traci.simulationStep()
    step += 1
    time.sleep(0.5)

    # Spawn new vehicles
    if step % SPAWN_INTERVAL == 0:
        for _ in range(random.randint(1, 2)):
            spawn_vehicle(step)

    # Track vehicles
    vehicles = [v for v in traci.vehicle.getIDList() if not v.startswith('ambulance')]
    positions = {vid: get_vehicle_position(vid) for vid in vehicles}

    # Detect new collisions
    new_collisions = []
    for vid1, pos1 in positions.items():
        for vid2, pos2 in positions.items():
            if vid1 >= vid2:
                continue
            distance = calculate_distance(pos1, pos2)
            pair = tuple(sorted([vid1, vid2]))
            if distance < COLLISION_DISTANCE and pair not in reported_collisions:
                for v in pair:
                    try:
                        traci.vehicle.setSpeed(v, 0)
                        traci.vehicle.setColor(v, (255,0,0,255))
                        current_edge = traci.vehicle.getRoadID(v)
                        current_pos = traci.vehicle.getLanePosition(v)
                        traci.vehicle.setStop(v, edgeID=current_edge, pos=current_pos, duration=999999)
                    except:
                        pass
                reported_collisions.add(pair)
                new_collisions.append((pair, pos1))
                total_accidents += 1
                print(f"[ACCIDENT] Vehicles involved: {pair} at position {pos1} (Total accidents: {total_accidents})")

    # Register accidents in CEN
    for collision_pair, location in new_collisions:
        accident_id = f"ACC_{total_accidents:03d}"
        x, y = location
        sim_time = traci.simulation.getTime()
        source_vehicle = collision_pair[0]
        cen.register(accident_id, (x, y), sim_time, source_vehicle)

        def detect_accident(veh1, veh2, x, y):
            """Dummy implementation: returns the current edge of veh1 as the accident edge."""
            try:
                return traci.vehicle.getRoadID(veh1)
            except Exception:
                return None

        accident_edge = detect_accident(collision_pair[0], collision_pair[1], x, y)

            # Collect ambulance positions
        positions = {}
        for amb in ambulance_readiness.keys():
                try:
                    positions[amb] = traci.vehicle.getPosition(amb)
                except Exception:
                    continue

            # GA call
        best_ambulance = select_best_ambulance(x, y, positions, accident_edge)

    # Vehicles listen to CEN broadcasts
    for vid, vehicle in vehicles_dict.items():
        if vid not in traci.vehicle.getIDList():
            continue
        vehicle.listen_and_reroute(cen, {k:v['position'] for k,v in edge_nodes.items()}, graph, comm_range=V2V_COMMUNICATION_RANGE)

    # Periodic CEN broadcast
    cen.broadcast(traci.simulation.getTime(), vehicles_dict=vehicles_dict, graph=graph, comm_range=V2V_COMMUNICATION_RANGE)

    
    
traci.close()