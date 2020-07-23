"""pick up and delivery with capacity and time window constraints with start time of vehicle and their start end nodes. Also
encorporating the penalties of dropping visits. Written by Karan Jagdale, IIT Bombay"""

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
    #All the properties in data are defined in the main code except vehicle capacity and no. of vehicles
    
    data = {}
    data['time_matrix'] = []
        
    data['time_windows'] = []
    
    data['pickups_deliveries'] = []
    
    data['demands'] = []

    data['vehicle_capacities'] = [100, 50, 50, 50, 50, 50, 50, 50]
    data['num_vehicles'] = 8
    
    
#    data['starts'] = [0, 2, 4, 6, 8, 10, 12, 14, 16, 18]
#    data['ends'] = [0, 0, 0, 0]
    return data

'''Following function is used to calculate the distance between two points on Earth using latitude and longitude'''
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
    dropped_nodes = 'Dropped nodes:'
    for node in range(routing.Size()):
        if routing.IsStart(node) or routing.IsEnd(node):
            continue
        if solution.Value(routing.NextVar(node)) == node:
            dropped_nodes += ' {}'.format(manager.IndexToNode(node))
    print(dropped_nodes)

    time_dimension = routing.GetDimensionOrDie('Time')
    total_time = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_load = 0
        while not routing.IsEnd(index):
            route_load += data['demands'][manager.IndexToNode(index)]
            time_var = time_dimension.CumulVar(index)
            plan_output += '{0} Time({1},{2}) Load({3}) -> '.format(
                manager.IndexToNode(index), solution.Min(time_var),
                solution.Max(time_var), route_load)
            index = solution.Value(routing.NextVar(index))
        time_var = time_dimension.CumulVar(index)
        plan_output += '{0} Time({1},{2}) Load({3})\n'.format(manager.IndexToNode(index),
                                                    solution.Min(time_var),
                                                    solution.Max(time_var), route_load)
        plan_output += 'Time of the route: {}min\n'.format(
            solution.Min(time_var))
        print(plan_output)
        total_time += solution.Min(time_var)
    print('Total time of all routes: {}min'.format(total_time))


def main():
    #1,2,3,4,5 and hi1,hi2,hi3 are markers used to see the point od execution of the code for debugging purpose
    print("1")
    t1 = time.time()  #start time
    #Csv file having the information of pick up and delivey latitudes and longitudes also vehicle start and end locations
    #data is arranged such that the no. of rowa equal to no. of vehicles have start and end location of vehicles
    # and ramining rows have information of location of pick up and location of delivery
    df = pd.read_csv('/home/karan/InternShipCodes/csv/FL_insurance_sample/300nodesBanglore.csv') 
    entries = df.to_numpy()
    length = (len(entries))
    print("2")
    
    data = create_data_model()
    distmatrix = np.zeros((2*length,2*length))
    
    # writing values to the distance matrix
    for i in range(length*2):
        for j in range(length*2):
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
    # Here the matrix is actually the time matrix as we have devided the distance by vehicle speed
    timematrix = distmatrix 

    penalty = int(timematrix.max())*100    #penalty for dropping visit
   
    print('penalty:',penalty)
    
    timematrix = timematrix.astype(int)
    print("3")
    
    #making list for pick up and delivery
    pickdel = np.zeros((len(entries)-data['num_vehicles'],2))
   
    for i in range(length-data['num_vehicles']):    
        pickdel[i][0] = int(i*2)
        pickdel[i][1] = int(2*i+1)
    print("4")
    pickdel = pickdel.astype(int)
    
    # Capacity(weight of good) for the pick up and delivery requests
    demand = []
    for i in range(length):
        b = int(round(random.random()*5+1))  
        if i>=(length-data['num_vehicles']): 
            demand.append(0)
            demand.append(0)
        
        if i<(length-data['num_vehicles']):
            demand.append(b)
            demand.append(-b)
    print("5")
    
    #Time windows for completing the pick ups and deliveries
    T = 0,7000
    TimeWindow = []
    for i in range(2*length):
        TimeWindow.append(T)
    
    #Start locations of all vehicles
    start= []
    for i in range(data['num_vehicles']):
        start.append(int((length-data['num_vehicles']+i)*2))
     
    #stop locations of all vehicles 
    stop= []
    for i in range(data['num_vehicles']):
        stop.append(int((length-data['num_vehicles']+i)*2 +1))
    
    startstop = start + stop
    
    #Start time of all vehicles
    veh_start = []
    for i in range(data['num_vehicles']):
        if i < data['num_vehicles']/2:
            veh_start.append((100,100))
        else:
            veh_start.append((200,200))
    
    #Time duration in which vehicles will work
    VehTimeDuration = []
    for i in range(data['num_vehicles']):
        VehTimeDuration.append(15200)
    """Entry point of the program."""
    
    # Instantiate the data problem.
    data['time_matrix'] = timematrix.tolist()
    data['pickups_deliveries'] = pickdel.tolist()
    data['demands'] = demand
    data['time_windows'] = TimeWindow
    data['starts'] = start
    data['ends'] = stop
    data['vehStart'] = veh_start
    data['vehtimeduration'] = VehTimeDuration
    #data['depot'] = 2*(length-1)
   
    #data['ends'] = []
#    for i in range(data['num_vehicles']):
#        data['ends'].append(2*(length-1))
    #print(data['time_matrix'])
    
    print(data['demands'])
    print(data['pickups_deliveries'])
    print(data['starts'])
    print(data['ends'])
    print(len(data['demands']))
    print(len(data['pickups_deliveries']))
    print(len(data['starts']))
    print(len(data['ends']))
    print(len(data['time_windows']))
    print(startstop)
#    print(data['time_windows'])
    """Solve the VRP with time windows."""
    # Instantiate the data problem.
    #data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']),
                                           data['num_vehicles'], data['starts'], data['ends'])

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
    routing.AddDimensionWithVehicleCapacity(
        transit_callback_index,
        30,  # allow waiting time
        data['vehtimeduration'],  # maximum time per vehicle
        False,  # Don't force start cumul to zero.
        time_dim)
    time_dimension = routing.GetDimensionOrDie(time_dim)
    time_dimension.SetGlobalSpanCostCoefficient(100)
    #print(time_dimension.transits)
    # Add time window constraints for each location except depot.
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx in startstop:
            continue
        index = manager.NodeToIndex(location_idx)
        #print(type(time_dimension.CumulVar(index)),index)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
    # Add time window constraints for each vehicle start node.
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(data['vehStart'][vehicle_id][0],
                                                data['vehStart'][vehicle_id][1])

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
    #penalty = 0         
    for node in range(len(data['time_matrix'])):
        if node in startstop:
            continue
        routing.AddDisjunction([manager.NodeToIndex(node)], penalty)    
    # Setting first solution heuristic.
    print("hi")
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    print("hi1")
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
#    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
#    search_parameters.local_search_metaheuristic = (
#        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
#    
#    search_parameters.log_search = True
    print("hi2")
    search_parameters.time_limit.seconds = 900

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)
    
    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)

    print("Status", routing.status())
    print("Time:", (time.time()-t1)/60)
if __name__ == '__main__':
    main()


