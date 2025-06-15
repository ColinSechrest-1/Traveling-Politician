import json
import math
import itertools
import requests
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

# Load the JSON file
with open("states.json", "r") as f:
    data = json.load(f)

# Use OpenStreetMap Nominatim API to get lat/lon (no API key needed)
def get_coordinates(address):
    url = f"https://nominatim.openstreetmap.org/search?q={address}&format=json&limit=1"
    headers = {'User-Agent': 'TSP-Example-App'}
    response = requests.get(url, headers=headers)
    if response.ok and response.json():
        result = response.json()[0]
        return float(result['lat']), float(result['lon'])
    return None, None

# Step 1: Add coordinates
for entry in data:
    address = entry["address"] + ", " + entry["zip"]
    lat, lon = get_coordinates(address)
    entry["latitude"] = lat
    entry["longitude"] = lon

# Save updated data with coordinates
with open("states_with_coords.json", "w") as f:
    json.dump(data, f, indent=2)

# Step 2: Calculate distance matrix using Haversine formula
def haversine(lat1, lon1, lat2, lon2):
    R = 6371  # Earth radius in km
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

coords = [(entry["latitude"], entry["longitude"]) for entry in data]
n = len(coords)
distance_matrix = [
    [int(haversine(*coords[i], *coords[j]) * 1000) for j in range(n)]
    for i in range(n)
]

# Step 3: Solve TSP using OR-Tools
def solve_tsp(distance_matrix, start_index=0):
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), 1, start_index)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        return distance_matrix[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    solution = routing.SolveWithParameters(search_params)

    if solution:
        index = routing.Start(0)
        route = []
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        route.append(manager.IndexToNode(index))
        return route
    return None

# Get Iowa's index
start_index = next(i for i, entry in enumerate(data) if entry["state"] == "Iowa")
route = solve_tsp(distance_matrix, start_index)

# Output route
if route:
    print("Shortest route starting in Iowa:")
    for i in route:
        print(data[i]["state"], "-", data[i]["capital"])
else:
    print("No solution found.")
