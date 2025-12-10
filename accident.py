# ----------------------------
import sys
import os
import time
import random
import xml.etree.ElementTree as ET
import math

# -------------------- SUMO PATH SETUP --------------------
if 'SUMO_HOME' not in os.environ:
    os.environ['SUMO_HOME'] = "/usr/share/sumo"

tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
if tools not in sys.path:
    sys.path.append(tools)

import traci
import sumolib

# ----------------------------
# SUMO setup
sumoBinary = sumolib.checkBinary("sumo-gui")
sumoConfig = "simulation.sumocfg"

# ----------------------------
# Parse routes and vehicle types from vehicles.rou.xml
rou_file = "vehicles.rou.xml"
tree = ET.parse(rou_file)
root = tree.getroot()

routes = [r.attrib['id'] for r in root.findall("route")]
vtypes = {v.attrib['id']: v for v in root.findall("vType")}

print("Loaded routes:", routes)
print("Loaded vehicle types:", list(vtypes.keys()))

# ----------------------------
NUM_INITIAL_VEHICLES = 8
SPAWN_INTERVAL = 10
VEHICLE_COUNTER = 0
COLLISION_DISTANCE = 2.5  # meters threshold

stopped_vehicles = set()
reported_collisions = set()

# ----------------------------
# Start SUMO
traci.start([
    sumoBinary, "-c", sumoConfig,
    "--collision.action", "none",
    "--collision.check-junctions", "true",
    "--ignore-route-errors", "true"
], port=8813)

# ----------------------------
# Spawn ambulances at fixed parking edges
ambulance_parking_routes = {
    "ambulance0": "routeAmbulance0",  # single-edge parking route
    "ambulance1": "routeAmbulance1",
    "ambulance2": "routeAmbulance2"
}

for vid, route in ambulance_parking_routes.items():
    # Add vehicle on parking route
    traci.vehicle.add(vid, routeID=route, typeID="ambulance", depart=0)

    # Set all emergency and gap parameters to avoid movement
    traci.vehicle.setSpeed(vid, 0)
    traci.vehicle.setEmergencyDecel(vid, 1000)
    traci.vehicle.setTau(vid, 0)
    traci.vehicle.setMinGap(vid, 0)

    # Stop vehicle permanently at the start of the parking edge
    parking_edge = traci.route.getEdges(route)[0]
    traci.vehicle.setStop(vid, edgeID=parking_edge, pos=0, duration=1e6)

    stopped_vehicles.add(vid)
    print(f"Ambulance {vid} spawned on parking route {route} ({parking_edge})")

# ----------------------------
# Function to spawn a normal vehicle
def spawn_vehicle(step):
    global VEHICLE_COUNTER
    vid = f"veh{VEHICLE_COUNTER}"
    VEHICLE_COUNTER += 1

    r = random.choice(routes)
    t = random.choice([v for v in vtypes.keys() if v != "ambulance"])
    traci.vehicle.add(vid, routeID=r, typeID=t, depart=step)
    traci.vehicle.setEmergencyDecel(vid, 1000)
    traci.vehicle.setTau(vid, 0)
    traci.vehicle.setMinGap(vid, 0)

    # Random speed
    max_speed = float(vtypes[t].attrib.get("maxSpeed", 13.9))
    speed = random.uniform(max_speed * 0.3, max_speed)
    traci.vehicle.setSpeed(vid, speed)

    print(f"Spawned {vid} on {r} as {t} at speed {speed:.1f} m/s at step {step}")

# ----------------------------
# Spawn initial vehicles
for _ in range(NUM_INITIAL_VEHICLES):
    spawn_vehicle(step=0)

# ----------------------------
# Run simulation
step = 0
MAX_STEPS = 500  # or infinite loop if desired
while step < MAX_STEPS:
    traci.simulationStep()
    step += 1
    time.sleep(0.5)

    # Spawn new vehicles periodically
    if step % SPAWN_INTERVAL == 0:
        for _ in range(random.randint(1, 2)):
            spawn_vehicle(step)

    # Get positions of all vehicles
    vehicles = traci.vehicle.getIDList()
    positions = {vid: traci.vehicle.getPosition(vid) for vid in vehicles}

    # Check collisions
    for vid1, pos1 in positions.items():
        for vid2, pos2 in positions.items():
            if vid1 >= vid2:
                continue
            distance = math.hypot(pos1[0] - pos2[0], pos1[1] - pos2[1])
            pair = tuple(sorted([vid1, vid2]))
            if distance < COLLISION_DISTANCE and pair not in reported_collisions:
                # Stop involved vehicles permanently
                for v in pair:
                    traci.vehicle.setSpeed(v, 0)
                    stopped_vehicles.add(v)

                x, y = pos1
                sim_time = traci.simulation.getTime()
                print(f"⚠️ Accident detected between {vid1} and {vid2} "
                      f"at location=({x:.2f}, {y:.2f}), time={sim_time:.1f}s")

                reported_collisions.add(pair)

# ----------------------------
traci.close()
