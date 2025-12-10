import random
import math
import traci
from congestion import get_fuzzy_congestion

# -------------------- GA Parameters --------------------
POP_SIZE = 20
GENERATIONS = 30
TOURNAMENT_K = 3
CROSSOVER_RATE = 0.8
MUTATION_RATE = 0.1

# -------------------- Ambulance Data --------------------
ambulance_readiness = {
    "ambulance0": 85,
    "ambulance1": 70,
    "ambulance2": 90
}

ambulance_routes = {
    "ambulance0": "routeAmbulance0",  # A_B_parking
    "ambulance1": "routeAmbulance1",  # D_G_parking
    "ambulance2": "routeAmbulance2"   # H_I_parking
}

ambulance_anchor_edges = {
    "ambulance0": "A_B",
    "ambulance1": "D_G",
    "ambulance2": "H_I"
}

# =======================================================
# FITNESS FUNCTION
# =======================================================
def fitness(individual, accident_x, accident_y, positions, accident_edge=None):
    """Fitness = readiness - distance penalty - congestion penalty + edge bonus"""
    if individual not in positions:
        return -9999.0

    (x, y) = positions[individual]
    dist = math.sqrt((x - accident_x) ** 2 + (y - accident_y) ** 2)
    readiness = ambulance_readiness.get(individual, 50)

    # --- Congestion penalty ---
    try:
        edge_id = traci.vehicle.getRoadID(individual)
        congestion_val = get_fuzzy_congestion(edge_id)
    except Exception:
        congestion_val = 5.0

    # Normalize
    readiness_norm = readiness / 100.0
    distance_penalty = dist / 200.0
    congestion_penalty = congestion_val / 10.0

    # --- Accident edge bonus ---
    edge_bonus = 0.0
    if accident_edge:
        anchor_edge = ambulance_anchor_edges.get(individual)
        if anchor_edge == accident_edge:
            edge_bonus = 1.0
        elif (anchor_edge[0] == accident_edge[0]) or (anchor_edge[-1] == accident_edge[-1]):
            edge_bonus = 0.5

    score = readiness_norm - distance_penalty - congestion_penalty + edge_bonus
    return score

# =======================================================
# SELECTION / CROSSOVER / MUTATION
# =======================================================
def tournament_selection(pop, fitnesses):
    best = random.choice(pop)
    for _ in range(TOURNAMENT_K - 1):
        challenger = random.choice(pop)
        if fitnesses[challenger] > fitnesses[best]:
            best = challenger
    return best

def crossover(parent1, parent2):
    if random.random() < CROSSOVER_RATE:
        return random.choice([parent1, parent2])
    return parent1

def mutate(individual):
    if random.random() < MUTATION_RATE:
        return random.choice(list(ambulance_readiness.keys()))
    return individual

# =======================================================
# GA DRIVER
# =======================================================
def select_best_ambulance(accident_x, accident_y, positions, accident_edge):
    population = [random.choice(list(ambulance_readiness.keys())) for _ in range(POP_SIZE)]

    for _ in range(GENERATIONS):
        fitnesses = {ind: fitness(ind, accident_x, accident_y, positions, accident_edge)
                     for ind in population}
        new_population = []
        while len(new_population) < POP_SIZE:
            p1 = tournament_selection(population, fitnesses)
            p2 = tournament_selection(population, fitnesses)
            child = crossover(p1, p2)
            child = mutate(child)
            new_population.append(child)
        population = new_population

    final_fitnesses = {ind: fitness(ind, accident_x, accident_y, positions, accident_edge)
                       for ind in population}
    best = max(final_fitnesses, key=final_fitnesses.get)
    print(f"Selected Best Ambulance: {best} (Route: {ambulance_routes[best]})")
    return best

# =======================================================
# ACCIDENT HANDLING
# =======================================================
def detect_accident(veh1, veh2, accident_x, accident_y):
    try:
        accident_edge = traci.vehicle.getRoadID(veh1)
    except Exception:
        accident_edge = None

    print("ACCIDENT DETECTED")
    print("=========================")
    print(f"Accident Location: ({accident_x:.2f}, {accident_y:.2f})")
    print(f"Vehicles involved: {veh1}, {veh2}")
    print(f"Accident Edge: {accident_edge}")
    return accident_edge

def handle_accident(veh1, veh2, accident_x, accident_y):
    accident_edge = detect_accident(veh1, veh2, accident_x, accident_y)

    # Collect ambulance positions
    positions = {}
    for amb in ambulance_readiness.keys():
        try:
            pos = traci.vehicle.getPosition(amb)
            positions[amb] = pos
        except Exception:
            continue

    # GA Selection
    best_ambulance = select_best_ambulance(accident_x, accident_y, positions, accident_edge)

    # Deploy ambulance
    try:
        if best_ambulance not in traci.vehicle.getIDList():
            traci.vehicle.add(best_ambulance,
                              routeID=ambulance_routes[best_ambulance],
                              typeID="ambulance")
            print(f"🚑 Spawned {best_ambulance} on {ambulance_routes[best_ambulance]}")
        else:
            print(f"🚑 {best_ambulance} already active in simulation.")

        # Reroute ambulance to accident edge
        if accident_edge:
            traci.vehicle.setRoute(best_ambulance, [accident_edge])
            print(f" {best_ambulance} rerouted to accident edge {accident_edge}")

    except Exception as e:
        print(f"Failed to deploy {best_ambulance}: {e}")

    return best_ambulance
