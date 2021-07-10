from controller.RouteController import RouteController
from core.Util import ConnectionInfo, Vehicle
import numpy as np
import traci
import math
import copy


class NShortestPathsPolicy(RouteController):

    amount_of_paths = 3

    def __init__(self, connection_info, paths):
        super().__init__(connection_info)
        self.amount_of_paths = paths

    def make_decisions(self, vehicles, connection_info):
        """
        make_decisions algorithm uses Dijkstra's algorithm to determine the 3 fastest routes from a car's source to its destination
        :param vehicles: list of vehicles on the map
        :param connection_info: information about the map (roads, junctions, etc)
        """
        local_targets = {}
        for vehicle in vehicles:

            all_paths = {} # map with key=length of path and value=list of paths per length
            destination_paths = {} # map that stores n shortest paths to desired destination
            count_paths = {edge : 0 for edge in self.connection_info.edge_list} # map that stores number of shortest paths found so far for every node
            starting_path = [vehicle.current_edge]
            all_paths[0] = [starting_path]
            while len(all_paths) > 0 and count_paths[vehicle.destination] < self.amount_of_paths:
                current_path = list(all_paths.values())[0][0]
                current_cost = list(all_paths.keys())[0]
                del all_paths[current_cost][0]
                current_node = current_path[len(current_path) - 1]
                count_paths[current_node] = count_paths[current_node] + 1

                if current_node == vehicle.destination:
                    if current_cost in destination_paths.keys():
                        destination_paths[current_cost].append(current_path)
                    else:
                        new_list = [current_path]
                        destination_paths[current_cost] = new_list

                if count_paths[current_node] <= self.amount_of_paths:
                    for direction, outgoing_edge in self.connection_info.outgoing_edges_dict[current_node].items():
                        new_path = []
                        for edge in current_path:
                            new_path.append(edge)
                        new_path.append(outgoing_edge)
                        new_cost = current_cost + self.connection_info.edge_length_dict[outgoing_edge]
                        if new_cost in all_paths.keys():
                            all_paths[new_cost].append(new_path)
                        else:
                            new_list = [new_path]
                            all_paths[new_cost] = new_list

                all_paths = {cost: paths for cost, paths in sorted(all_paths.items(), key=lambda x: x[0])}

                empty_costs = []
                for cost in all_paths.keys():
                    if len(all_paths[cost]) == 0:
                        empty_costs.append(cost)

                for cost in empty_costs:
                    del all_paths[cost]

            destination_path_times = {}
            for cost, paths in destination_paths.items():
                for path in paths:
                    total_time = 0
                    for edge in path:
                        travel_time = traci.edge.getTraveltime(edge)
                        total_time += travel_time
                    if total_time in destination_path_times.keys():
                        destination_path_times[total_time].append(path)
                    else:
                        new_list = [path]
                        destination_path_times[total_time] = new_list

            destination_directions = {}
            for time, paths in destination_path_times.items():
                for path in paths:
                    direction_list = []
                    for i in range(0, len(path) - 1):
                        for direction in self.connection_info.outgoing_edges_dict[path[i]].keys():
                            if self.connection_info.outgoing_edges_dict[path[i]][direction] == path[i + 1]:
                                direction_list.append(direction)
                    # destination_directions[direction_list] = cost
                    if time in destination_directions.keys():
                        destination_directions[time].append(direction_list)
                    else:
                        new_directions = [direction_list]
                        destination_directions[time] = new_directions

            destination_directions = {time: paths for time, paths in sorted(destination_directions.items(), key=lambda x: x[0])}

            final_time_list = []
            for time, paths in destination_directions.items():
                for i in range(0, len(paths)):
                    final_time_list.append(time)

            final_destination_list = []
            for paths in destination_directions.values():
                for path in paths:
                    final_destination_list.append(path)

            index = 0
            for time in final_time_list:
                if time + traci.simulation.getTime() + 150 > vehicle.deadline:
                    break
                else:
                    index += 1

            index = index - 1
            if index == -1:
                index = 0
            decision_list = final_destination_list[index]
            time_left = vehicle.deadline - traci.simulation.getTime()
            print(str(vehicle.vehicle_id) + " " + vehicle.current_edge + " " + str(index) + " " + str(final_destination_list[index]) + " " + str(time_left))

            local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)

        return local_targets

