"""Vehicles Routing Problem (VRP) with Time Windows."""

from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import pandas as pd
import numpy as np
import math
import random
import time
R = 6371.0
speed = 4.17   #speed of each vehicle in m/s

def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data['time_matrix'] = [
        [0, 6, 9, 8, 7, 3, 6, 2, 3, 2, 6, 6, 4, 4, 5, 9, 7],
        [6, 0, 8, 3, 2, 6, 8, 4, 8, 8, 13, 7, 5, 8, 12, 10, 14],
        [9, 8, 0, 11, 10, 6, 3, 9, 5, 8, 4, 15, 14, 13, 9, 18, 9],
        [8, 3, 11, 0, 1, 7, 10, 6, 10, 10, 14, 6, 7, 9, 14, 6, 16],
        [7, 2, 10, 1, 0, 6, 9, 4, 8, 9, 13, 4, 6, 8, 12, 8, 14],
        [3, 6, 6, 7, 6, 0, 2, 3, 2, 2, 7, 9, 7, 7, 6, 12, 8],
        [6, 8, 3, 10, 9, 2, 0, 6, 2, 5, 4, 12, 10, 10, 6, 15, 5],
        [2, 4, 9, 6, 4, 3, 6, 0, 4, 4, 8, 5, 4, 3, 7, 8, 10],
        [3, 8, 5, 10, 8, 2, 2, 4, 0, 3, 4, 9, 8, 7, 3, 13, 6],
        [2, 8, 8, 10, 9, 2, 5, 4, 3, 0, 4, 6, 5, 4, 3, 9, 5],
        [6, 13, 4, 14, 13, 7, 4, 8, 4, 4, 0, 10, 9, 8, 4, 13, 4],
        [6, 7, 15, 6, 4, 9, 12, 5, 9, 6, 10, 0, 1, 3, 7, 3, 10],
        [4, 5, 14, 7, 6, 7, 10, 4, 8, 5, 9, 1, 0, 2, 6, 4, 8],
        [4, 8, 13, 9, 8, 7, 10, 3, 7, 4, 8, 3, 2, 0, 4, 5, 6],
        [5, 12, 9, 14, 12, 6, 6, 7, 3, 3, 4, 7, 6, 4, 0, 9, 2],
        [9, 10, 18, 6, 8, 12, 15, 8, 13, 9, 13, 3, 4, 5, 9, 0, 9],
        [7, 14, 9, 16, 14, 8, 5, 10, 6, 5, 4, 10, 8, 6, 2, 9, 0],
    ]
    data['time_windows'] = [
        (0, 1800),  # depot
        (0, 1800),  # 1
        (0, 1800),  # 2
        (0, 1800),  # 3
        (0, 1800),  # 4
        (0, 1800),  # 5
        (0, 1800),  # 6
        (0, 1800),  # 7
        (0, 1800),  # 8
        (0, 1800),  # 9
        (0, 1800),  # 10
        (0, 1800),  # 11
        (0, 1800),  # 12
        (0, 1800),  # 13
        (0, 1800),  # 14
        (0, 1800),  # 15
        (0, 1800),  # 16
    ]
    data['pickups_deliveries'] = [
        [1, 6],     #2
        [2, 10],    #4
        [4, 3],     #1
        [5, 9],     #2
        [7, 8],     #5
        [15, 11],   #3
        [13, 12],   #1
        [16, 14],   #3
        [6, 4] ,   #0
        [1,10],    #0
           #3
    ]
    data['demands'] = [0, 2, 4, -1, 1, 2, -2, 5, -5, -2, -4, -3, 1, -1, -3, 3, 3]
#    data['pickups_deliveries'] = [
#        [1, 6],     #2
#        [2, 10],    #4
#        [4, 3],     #1
#        [5, 9],     #2
#        [7, 8],     #5
#        [15, 11],   #3
#        [13, 12],   #1
#        [16, 14],   #3
#        [6, 15],    #5
#        [7, 14],    #6
#        [9, 12],    #3
#    ]
#    data['demands'] = [0, 2, 4, -1, 1, 2, 3, 11, -5, -5, -4, -3, 2, 1, -9, -2, 3]
    data['vehicle_capacities'] = [100, 100, 100, 100, 100, 50, 50, 50, 50, 50]
    data['num_vehicles'] = 10
    data['depot'] = 22
#    data['starts'] = [0, 2, 4, 6, 8, 10, 12, 14, 16, 18]
#    data['ends'] = [0, 0, 0, 0]
    return data

def distance(Lat1, Lat2, Lon1, Lon2):
    lat1 = math.radians(Lat1)
    lon1 = math.radians(Lon1)
    lat2 = math.radians(Lat2)
    lon2 = math.radians(Lon2)
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance

def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    time_dimension = routing.GetDimensionOrDie('Time')
    total_time = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        while not routing.IsEnd(index):
            time_var = time_dimension.CumulVar(index)
            plan_output += '{0} Time({1},{2}) -> '.format(
                manager.IndexToNode(index), solution.Min(time_var),
                solution.Max(time_var))
            index = solution.Value(routing.NextVar(index))
        time_var = time_dimension.CumulVar(index)
        plan_output += '{0} Time({1},{2})\n'.format(manager.IndexToNode(index),
                                                    solution.Min(time_var),
                                                    solution.Max(time_var))
        plan_output += 'Time of the route: {}min\n'.format(
            solution.Min(time_var))
        print(plan_output)
        total_time += solution.Min(time_var)
    print('Total time of all routes: {}min'.format(total_time))


def main():
    #global time
    print("1")
    t1 = time.time()
    df = pd.read_csv('/home/karan/InternShipCodes/csv/FL_insurance_sample/201nodesBanglore.csv') 
    entries = df.to_numpy()
    length = (len(entries))
    print("2")
    #print(entries[0][0])
    #initialize distance matrix
#    distmatrix = np.zeros((2*length,2*length))
#    """Entry point of the program."""
#    # writing values to the distance matrix
#    for i in range(length*2):
#        for j in range(length*2):
#            if i%2 == 0:
#      #          print(i,j)
#                Lat1 = entries[int((i/2))][0]
#                Lon1 = entries[int(i/2)][1]
#            if i%2 == 1:
#                Lat1 = entries[int(((i+1)/2)-1)][2]
#                Lon1 = entries[int(((i+1)/2)-1)][3]
#            
#            if j%2 == 0:
#                Lat2 = entries[int(j/2)][0]
#                Lon2 = entries[int(j/2)][1]
#            if j%2 == 1:
#                Lat2 = entries[int(((j+1)/2)-1)][2]
#                Lon2 = entries[int(((j+1)/2)-1)][3]
#            distmatrix[i][j] = int(round(distance(Lat1, Lat2, Lon1, Lon2)*10))    
#    #print(distmatrix)
#    # writing the pick up and delivery info
#    distmatrix = distmatrix.astype(int)
#    print("3")
#    pickdel = np.zeros((len(entries),2))
#    print("4")
#
#    for i in range(length):    
#        pickdel[i][0] = int(i*2)
#        pickdel[i][1] = int(2*i+1)
#    print("4")
#    pickdel = pickdel.astype(int)
#    # Capacity for pick up and delivery
#    demand = []
#    for i in range(length):
#        b = int(round(random.random()*5))   
#        demand.append(b)
#        demand.append(-b)
#    print("5")
#    T = 0,2000
#    TimeWindow = []
#    for i in range(2*length):
#        TimeWindow.append(T)
    distmatrix = np.zeros((2*length-1,2*length-1))
    """Entry point of the program."""
    # writing values to the distance matrix
    for i in range(length*2-1):
        for j in range(length*2-1):
            if i%2 == 0:
      #          print(i,j)
                Lat1 = entries[int((i/2))][0]
                Lon1 = entries[int(i/2)][1]
            if i%2 == 1:
                Lat1 = entries[int(((i+1)/2)-1)][2]
                Lon1 = entries[int(((i+1)/2)-1)][3]
            
            if j%2 == 0:
                Lat2 = entries[int(j/2)][0]
                Lon2 = entries[int(j/2)][1]
            if j%2 == 1:
                Lat2 = entries[int(((j+1)/2)-1)][2]
                Lon2 = entries[int(((j+1)/2)-1)][3]
            distmatrix[i][j] = int(round(distance(Lat1, Lat2, Lon1, Lon2)*1000/speed))    
    #print(distmatrix)
    # writing the pick up and delivery info
    timematrix = distmatrix
    timematrix = timematrix.astype(int)
    print("3")
    pickdel = np.zeros((len(entries)-1,2))
    print("4")

    for i in range(length-1):    
        pickdel[i][0] = int(i*2)
        pickdel[i][1] = int(2*i+1)
    print("4")
    pickdel = pickdel.astype(int)
    # Capacity for pick up and delivery
    demand = []
    for i in range(length):
        b = int(round(random.random()*5+1))  
        if i==(length-1):
            demand.append(0)
        if i!=(length-1):
            demand.append(b)
            demand.append(-b)
    print("5")
    T = 0,100000
    TimeWindow = []
    for i in range(2*length-1):
        TimeWindow.append(T)
    """Entry point of the program."""
    # Instantiate the data problem.
    data = create_data_model()
    data['time_matrix'] = timematrix.tolist()
    data['pickups_deliveries'] = pickdel.tolist()
    data['demands'] = demand
    data['time_windows'] = TimeWindow
    data['depot'] = 2*(length-1)
    #data['ends'] = []
#    for i in range(data['num_vehicles']):
#        data['ends'].append(2*(length-1))
    #print(data['time_matrix'])
#    print(data['demands'])
#    print(data['pickups_deliveries'])
#    print(data['time_windows'])
    """Solve the VRP with time windows."""
    # Instantiate the data problem.
    #data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)


    # Create and register a transit callback.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows constraint.
    time_dim = 'Time'
    routing.AddDimension(
        transit_callback_index,
        30,  # allow waiting time
        30000,  # maximum time per vehicle
        True,  # Don't force start cumul to zero.
        time_dim)
    time_dimension = routing.GetDimensionOrDie(time_dim)
    time_dimension.SetGlobalSpanCostCoefficient(100)
    #print(time_dimension.transits)
    # Add time window constraints for each location except depot.
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx == 0:
            continue
        index = manager.NodeToIndex(location_idx)
        #print(type(time_dimension.CumulVar(index)),index)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
    # Add time window constraints for each vehicle start node.
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(data['time_windows'][0][0],
                                                data['time_windows'][0][1])

    # Instantiate route start and end times to produce feasible times.
    for i in range(data['num_vehicles']):
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.Start(i)))
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.End(i)))
    # Define Transportation Requests.
    for request in data['pickups_deliveries']:
        pickup_index = manager.NodeToIndex(request[0])
        delivery_index = manager.NodeToIndex(request[1])
        routing.AddPickupAndDelivery(pickup_index, delivery_index)
        routing.solver().Add(
            routing.VehicleVar(pickup_index) == routing.VehicleVar(
                delivery_index))
        routing.solver().Add(
            time_dimension.CumulVar(pickup_index) <=
            time_dimension.CumulVar(delivery_index))
    
    
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]
    
    demand_callback_index = routing.RegisterUnaryTransitCallback(
        demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')
    #following two lines set up priority in the assignment of the pick up and delivery requests
#    for vehicle_id in range(data['num_vehicles']):  #comment these two lines to distribute the pick up and deliveries uniformly 
#        time_dimension.SetSpanCostCoefficientForVehicle(int(vehicle_id*3), vehicle_id)  #to all the delivery persons
    
    for vehicle_id in range(data['num_vehicles']):
        cost_coeff = time_dimension.GetSpanCostCoefficientForVehicle(vehicle_id)
        print(vehicle_id, 'cost_coeff', cost_coeff)
             
        
    # Setting first solution heuristic.
    print("hi")
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    print("hi1")
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    print("hi2")
    search_parameters.time_limit.seconds = 1000

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)
    
    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)

    print("Status", routing.status())
    print("Time:", (time.time()-t1)/60)
if __name__ == '__main__':
    main()


