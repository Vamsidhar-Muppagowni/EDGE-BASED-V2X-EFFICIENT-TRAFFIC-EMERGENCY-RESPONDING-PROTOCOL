# Edge-Based V2X Efficient Traffic Emergency Responding Protocol

## 📌 Project Overview
This project implements an **Edge-based V2X (Vehicle-to-Everything)** protocol designed to minimize response times for Emergency Response Vehicles (ERVs) in congested urban environments.

The system utilizes a combination of **Edge Computing** and **Task Scheduling** algorithms to detect accidents, identify the optimal ERV, and dynamically reroute traffic to clear a path. The simulation is built using **SUMO (Simulation of Urban MObility)** and **TraCI** (Python) to model realistic vehicular ad-hoc networks (VANETs).

### 🚀 Key Features
* **Dynamic Accident Management:** Random generation of accidents across the road network using SUMO.
* **Smart Congestion Detection:** Utilization of **Fuzzy Logic** to categorize traffic density (Low, Medium, High) based on vehicle count and average speed.
* **Advanced ERV Selection:** A **Genetic Algorithm (GA)** selects the "fittest" ambulance based on:
    * Distance to accident.
    * Real-time traffic congestion.
    * ERV Readiness (availability/equipment).
* **Intelligent Rerouting:** Implementation of **Ant Colony Optimization (ACO)** to reroute normal vehicles away from the accident site and the ERV's projected path.
* **V2X Communication:** Simulation of Vehicle-to-Vehicle (V2V) and Vehicle-to-Infrastructure (V2I) message propagation.

---

## 🏗️ Architecture
The system operates on a three-layer architecture:
1.  **Vehicle Layer:** Accident vehicles, Relay vehicles, and ERVs.
2.  **Edge Layer:** Roadside Units (RSUs) and Cloud Edge Nodes (CEN) that process local data and handle broadcasting.
3.  **Cloud Layer:** Central server for long-term incident monitoring and data storage.

---

## 🛠️ Technologies & Algorithms
* **Simulation Environment:** SUMO (Simulation of Urban MObility)
* **Network Interface:** TraCI (Traffic Control Interface)
* **Language:** Python
* **Core Algorithms:**
    * **Genetic Algorithm (GA):** For optimizing ERV selection.
    * **Fuzzy Logic:** For real-time path congestion scoring (0-10 scale).
    * **Ant Colony Optimization (ACO):** For calculating optimal rerouting paths for civilian vehicles.

---

## 📂 Project Structure

| File Name | Description |
| :--- | :--- |
| `Run simulation.py` | The main entry point to initialize and execute the simulation logic. |
| `Sumo network and accident.py` | Handles the SUMO network configuration and random accident generation. |
| `Congestion.py` | Implements Fuzzy Logic to calculate congestion scores for road segments. |
| `Best erv.py` | Contains the Genetic Algorithm logic to select the optimal Emergency Response Vehicle. |
| `Vehicle.py` | Defines vehicle properties, behavior, and V2V interaction capabilities. |
| `Route generator.py` | Generates routes for vehicles within the simulation network. |
| `Cen broadcast.py` | Manages the Central Edge Node (CEN) logic for broadcasting alerts to vehicles. |

---

## ⚙️ Installation & Usage

### Prerequisites
* Python 3.x
* [SUMO](https://eclipse.dev/sumo/) (installed and added to system PATH)
* `traci` python module (`pip install traci`)

### Running the Simulation
1.  Clone the repository:
    ```bash
    git clone [https://github.com/your-username/edge-v2x-emergency-protocol.git](https://github.com/your-username/edge-v2x-emergency-protocol.git)
    ```
2.  Navigate to the project directory:
    ```bash
    cd edge-v2x-emergency-protocol
    ```
3.  Run the main simulation script:
    ```bash
    python "Run simulation.py"
    ```

---

## 👥 Contributors (Group 12)
* **Isha (CSE23029):** Run Simulation Logic
* **Raghav (CSE23032):** SUMO Network & Accident Logic
* **Vijay (CSE23754):** Congestion Algorithm (Fuzzy Logic)
* **Mohan (CSE23409):** Vehicle Class & Logic
* **Vamsidhar (CSE23163):** Best ERV Selection (Genetic Algorithm)
* **Akshar (CSE23547):** Route Generation
* **Keerthi (CSE23031):** CEN Broadcast Logic

---

## 📄 License
This project was developed for the **DIVANET '21** conference context.
