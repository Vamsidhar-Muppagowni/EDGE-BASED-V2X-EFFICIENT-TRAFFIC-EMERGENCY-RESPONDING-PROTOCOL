# vehicle.py
import math
import traci
import random

class Vehicle:
    def __init__(self, veh_id, destination):
        self.veh_id = veh_id
        self.destination = destination
        self.current_edge = None
        self.route = []
        self.accidents_received = set()  # track accident IDs already received

    def update_position(self):
        try:
            self.current_edge = traci.vehicle.getRoadID(self.veh_id)
            self.route = traci.vehicle.getRoute(self.veh_id)
        except:
            pass

    def distance_to_cen(self, cen_pos):
        try:
            veh_pos = traci.vehicle.getPosition(self.veh_id)
            return math.hypot(veh_pos[0] - cen_pos[0], veh_pos[1] - cen_pos[1])
        except:
            return float('inf')

    def listen_and_reroute(self, cen, cen_positions, graph, comm_range=200):
        self.update_position()
        try:
            veh_pos = traci.vehicle.getPosition(self.veh_id)
        except:
            return

        for acc_id, data in cen.accidents.items():
            # Skip accidents already processed
            if acc_id in self.accidents_received:
                continue

            broadcasting_cen_name = data["registered_by_name"]
            broadcasting_edge = data["registered_by_edge"]
            accident_pos = data["location"]

            if broadcasting_edge not in cen_positions:
                continue

            dist = self.distance_to_cen(cen_positions[broadcasting_edge])
            if dist <= comm_range:
                # Mark accident as received
                self.accidents_received.add(acc_id)

                print(f"[V2I] Vehicle {self.veh_id} received accident {acc_id} info from CEN {broadcasting_cen_name} "
                      f"at t={traci.simulation.getTime():.1f}s (distance {dist:.1f})")

                # Determine the exact edge of the accident
                accident_edge_actual = self.map_position_to_edge(accident_pos, graph)
                
                # Compute new route avoiding the accident
                old_route = list(self.route)
                new_route = self.aco_reroute(graph, blocked_edge=accident_edge_actual)

                # Only update route if different from current
                if new_route != old_route:
                    self.route = new_route
                    try:
                        traci.vehicle.setRoute(self.veh_id, self.route)
                        print(f"[REROUTE] Vehicle {self.veh_id} old route: {old_route}")
                        print(f"[REROUTE] Vehicle {self.veh_id} new route: {self.route}")
                    except traci.exceptions.TraCIException as e:
                        print(f"[ERROR] Failed to set new route for {self.veh_id}: {e}")

    def map_position_to_edge(self, position, graph):
        x, y = position
        min_dist = float('inf')
        nearest_edge = None
        for node, neighbors in graph.items():
            node_pos = neighbors[0][0] if neighbors else node
            dx = x - node_pos[0] if isinstance(node_pos, tuple) else 0
            dy = y - node_pos[1] if isinstance(node_pos, tuple) else 0
            d = math.hypot(dx, dy)
            if d < min_dist:
                min_dist = d
                nearest_edge = node
        return nearest_edge if nearest_edge else self.current_edge

    def aco_reroute(self, graph, blocked_edge):
        NUM_ANTS = 6
        MAX_HOPS = 20
        alpha = 1.0
        beta = 2.0
        evaporation = 0.1
        pheromone = {edge: 1.0 for edge in graph}

        best_route = None
        best_cost = float("inf")
        destination = self.destination

        for _ in range(NUM_ANTS):
            current = self.current_edge
            visited = [current]
            cost = 0

            for _ in range(MAX_HOPS):
                neighbors = [(n, c) for n, c in graph.get(current, []) 
                             if n != blocked_edge and n not in visited]
                if not neighbors:
                    break

                probs = []
                for n, c in neighbors:
                    tau = pheromone.get(n, 1.0) ** alpha
                    eta = (1.0 / c) ** beta
                    probs.append(tau * eta)
                total = sum(probs)
                probs = [p / total for p in probs]

                r = random.random()
                cumulative = 0
                for idx, (n, _) in enumerate(neighbors):
                    cumulative += probs[idx]
                    if r <= cumulative:
                        next_node = n
                        next_cost = neighbors[idx][1]
                        break

                visited.append(next_node)
                cost += next_cost
                current = next_node
                if current == destination:
                    break

            if visited[-1] == destination and cost < best_cost:
                best_cost = cost
                best_route = visited

            for node in visited:
                pheromone[node] = (1 - evaporation) * pheromone.get(node, 1.0) + 0.1

        if not best_route:
            best_route = [e for e in self.route if e != blocked_edge]

        # Ensure current edge is at the start of the route
        if best_route[0] != self.current_edge:
            best_route = [self.current_edge] + best_route

        # Remove blocked edge from route
        best_route = [e for e in best_route if e != blocked_edge]

        return best_route
