import os
import sys
import time
import traci
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

# SUMO Setup
if 'SUMO_HOME' not in os.environ:
    os.environ['SUMO_HOME'] = "/usr/share/sumo"
tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
if tools not in sys.path:
    sys.path.append(tools)

# -------------------- Fuzzy Logic --------------------
vehicle_count = ctrl.Antecedent(np.arange(0, 21, 1), 'vehicle_count')
avg_speed = ctrl.Antecedent(np.arange(0, 21, 1), 'avg_speed')
congestion = ctrl.Consequent(np.arange(0, 11, 1), 'congestion')

vehicle_count['low'] = fuzz.trimf(vehicle_count.universe, [0,0,5])
vehicle_count['medium'] = fuzz.trimf(vehicle_count.universe, [3,7,12])
vehicle_count['high'] = fuzz.trimf(vehicle_count.universe, [10,20,20])

avg_speed['low'] = fuzz.trimf(avg_speed.universe, [0,0,5])
avg_speed['medium'] = fuzz.trimf(avg_speed.universe, [3,8,12])
avg_speed['high'] = fuzz.trimf(avg_speed.universe, [10,20,20])

congestion['low'] = fuzz.trimf(congestion.universe, [0,0,3])
congestion['medium'] = fuzz.trimf(congestion.universe, [2,5,8])
congestion['high'] = fuzz.trimf(congestion.universe, [7,10,10])

rule1 = ctrl.Rule(vehicle_count['high'] & avg_speed['low'], congestion['high'])
rule2 = ctrl.Rule(vehicle_count['medium'] & avg_speed['medium'], congestion['medium'])
rule3 = ctrl.Rule(vehicle_count['low'] & avg_speed['high'], congestion['low'])
rule4 = ctrl.Rule(vehicle_count['high'] & avg_speed['medium'], congestion['medium'])
rule5 = ctrl.Rule(vehicle_count['medium'] & avg_speed['low'], congestion['high'])
rule6 = ctrl.Rule(vehicle_count['low'] & avg_speed['low'], congestion['medium'])

congestion_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6])
congestion_sim = ctrl.ControlSystemSimulation(congestion_ctrl)

# -------------------- Example Query --------------------
def get_fuzzy_congestion(edge):
    veh_count = len(traci.edge.getLastStepVehicleIDs(edge))
    avg_spd = max(traci.edge.getLastStepMeanSpeed(edge),0)
    congestion_sim.input['vehicle_count'] = veh_count
    congestion_sim.input['avg_speed'] = avg_spd
    congestion_sim.compute()
    return congestion_sim.output['congestion']  # 0-10
