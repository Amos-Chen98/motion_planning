import math
import matplotlib.pyplot as plt


class AstarPlanner():

    def __init__(self, a_star_config):
        # Set planning dimension: 2 or 3
        self.dim = getattr(a_star_config, 'dim', 3)
        if self.dim not in [2, 3]:
            raise ValueError(f"Invalid dim: {self.dim}. Must be 2 or 3.")
        
        self.motion = self.get_motion_model()
        self.root_parent_index = -1  # The parent of the start node. No actual meaning, just to avoid duplicate index
        self.resolution = a_star_config.resolution
        self.min_flight_height = a_star_config.min_flight_height
        self.max_flight_height = a_star_config.max_flight_height

    class Node:
        def __init__(self, index, cost, parent_index):
            self.index = index  # a tuple (x, y) for 2D or (x, y, z) for 3D, note that this is not the real position
            self.cost = cost  # the cost in defined in grid, from the start node to the current node, not including the heuristic cost
            self.parent_index = parent_index  # a tuple (x, y) for 2D or (x, y, z) for 3D

    def plan(self, map, start_pos, target_pos):
        '''
        input: start_pos, target_pos: list [x, y] for 2D or [x, y, z] for 3D
        return a path: a list of lists, [start_pos, [x1, y1(, z1)], ... , target_pos]
        '''
        # Validate input dimensions
        if self.dim == 2:
            if len(start_pos) < 2 or len(target_pos) < 2:
                raise ValueError("2D mode requires at least [x, y] coordinates")
            # Use only x, y for 2D planning
            start_pos = start_pos[:2]
            target_pos = target_pos[:2]
        else:  # 3D mode
            if len(start_pos) < 3 or len(target_pos) < 3:
                raise ValueError("3D mode requires [x, y, z] coordinates")
            start_pos = start_pos[:3]
            target_pos = target_pos[:3]
        
        # read map info
        self.map = map  # the map class is defined in src/planner/scripts/map_server/pcl_server.py

        # Check if the straight line from start_pos to target_pos is feasible.
        if hasattr(self.map, 'seg_feasible_check') and self.map.seg_feasible_check(start_pos, target_pos):
            return [start_pos, target_pos]

        # read mission info
        start_node_index = self.calc_index(start_pos)
        start_node = self.Node(start_node_index, 0.0, self.root_parent_index)
        target_node_index = self.calc_index(target_pos)
        self.target_node = self.Node(target_node_index, 0.0, self.root_parent_index)

        # open and close set
        open_set, close_set = dict(), dict()
        open_set[start_node_index] = start_node

        # search
        while True:
            if not open_set:
                print("Open set is empty, no path found")
                return None

            # get the current_node node
            current_node_index = min(open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(open_set[o], self.target_node))
            current_node = open_set[current_node_index]

            # check if the current_node node is the target node
            if current_node.index == self.target_node.index:
                self.target_node.parent_index = current_node.parent_index
                self.target_node.cost = current_node.cost
                break

            # move the current_node node to the close set
            del open_set[current_node_index]
            close_set[current_node_index] = current_node

            # expand the current_node node
            for motion in self.motion:
                if self.dim == 2:
                    move_x, move_y, move_cost = motion
                    candidate_node_index = (int(current_node.index[0] + move_x),
                                            int(current_node.index[1] + move_y))
                else:  # 3D mode
                    move_x, move_y, move_z, move_cost = motion
                    candidate_node_index = (int(current_node.index[0] + move_x),
                                            int(current_node.index[1] + move_y),
                                            int(current_node.index[2] + move_z))
                candidate_node = self.Node(candidate_node_index, current_node.cost + move_cost, current_node_index)

                # check if the candidate_node is valid
                if candidate_node_index in close_set:
                    continue

                if not self.verify_node(candidate_node):
                    continue

                # update the node in open set
                if candidate_node_index not in open_set or open_set[candidate_node_index].cost > candidate_node.cost:
                    open_set[candidate_node_index] = candidate_node

        # retrieve the path
        path = self.retrieve_final_path(close_set)

        # self.visualize_path(path)
        return path

    def get_motion_model(self):
        if self.dim == 2:
            # 2D motion model: dx, dy, cost
            # 4-connected and 8-connected motions
            motion = [
                [1, 0, 1],              # right
                [0, 1, 1],              # up
                [-1, 0, 1],             # left
                [0, -1, 1],             # down
                [1, 1, math.sqrt(2)],   # right-up
                [1, -1, math.sqrt(2)],  # right-down
                [-1, 1, math.sqrt(2)],  # left-up
                [-1, -1, math.sqrt(2)]  # left-down
            ]
        else:  # 3D mode
            # 3D motion model: dx, dy, dz, cost
            motion = [
                # 6-connected (face neighbors)
                [1, 0, 0, 1],
                [0, 1, 0, 1],
                [0, 0, 1, 1],
                [-1, 0, 0, 1],
                [0, -1, 0, 1],
                [0, 0, -1, 1],
                # 12 edge neighbors
                [1, 1, 0, math.sqrt(2)],
                [1, -1, 0, math.sqrt(2)],
                [-1, 1, 0, math.sqrt(2)],
                [-1, -1, 0, math.sqrt(2)],
                [1, 0, 1, math.sqrt(2)],
                [1, 0, -1, math.sqrt(2)],
                [-1, 0, 1, math.sqrt(2)],
                [-1, 0, -1, math.sqrt(2)],
                [0, 1, 1, math.sqrt(2)],
                [0, 1, -1, math.sqrt(2)],
                [0, -1, 1, math.sqrt(2)],
                [0, -1, -1, math.sqrt(2)],
                # 8 corner neighbors
                [1, 1, 1, math.sqrt(3)],
                [1, 1, -1, math.sqrt(3)],
                [1, -1, 1, math.sqrt(3)],
                [1, -1, -1, math.sqrt(3)],
                [-1, 1, 1, math.sqrt(3)],
                [-1, 1, -1, math.sqrt(3)],
                [-1, -1, 1, math.sqrt(3)],
                [-1, -1, -1, math.sqrt(3)]
            ]
        return motion

    def calc_real_pos(self, node_index):
        '''
        return a list [x, y] for 2D or [x, y, z] for 3D: the real position of the node
        '''
        if self.dim == 2:
            return [node_index[0]*self.resolution, node_index[1]*self.resolution]
        else:  # 3D mode
            return [node_index[0]*self.resolution, node_index[1]*self.resolution, node_index[2]*self.resolution]

    def calc_index(self, pos):
        if self.dim == 2:
            return (int(pos[0]/self.resolution), int(pos[1]/self.resolution))
        else:  # 3D mode
            return (int(pos[0]/self.resolution), int(pos[1]/self.resolution), int(pos[2]/self.resolution))

    def calc_heuristic(self, node1, node2):
        dx = node1.index[0] - node2.index[0]
        dy = node1.index[1] - node2.index[1]
        if self.dim == 2:
            return math.sqrt(dx * dx + dy * dy)
        else:  # 3D mode
            dz = node1.index[2] - node2.index[2]
            return math.sqrt(dx * dx + dy * dy + dz * dz)

    def collision_check(self, node):
        # return True if the node is in collision
        node_pos = self.calc_real_pos(node.index)
        if hasattr(self.map, 'has_collision'):
            return self.map.has_collision(node_pos)
        return False

    def verify_node(self, node):
        # Return True if the node is valid
        if self.dim == 3:
            # Check height constraints for 3D planning
            if node.index[2] < self.min_flight_height / self.resolution:
                return False
            if node.index[2] > self.max_flight_height / self.resolution:
                return False
        
        # Check collision for both 2D and 3D
        if self.collision_check(node):
            return False
        return True

    def retrieve_final_path(self, close_set):
        path = []
        node = self.target_node
        while node.parent_index != self.root_parent_index:
            path.append(self.calc_real_pos(node.index))
            node = close_set[node.parent_index]
        path.append(self.calc_real_pos(node.index))  # append the start node
        return path[::-1]

    def visualize_path(self, path):
        if self.dim == 2:
            plt.figure()
            plt.plot([x[0] for x in path], [x[1] for x in path], 'r-', linewidth=2, label='Path')
            plt.plot([x[0] for x in path], [x[1] for x in path], 'bo', markersize=5)
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.title('2D A* Path')
            plt.axis("equal")
            plt.legend()
            plt.grid(True)
            plt.show()
        else:  # 3D mode
            from mpl_toolkits.mplot3d import Axes3D
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            ax.plot([x[0] for x in path], [x[1] for x in path], [x[2] for x in path], 'r-', linewidth=2, label='Path')
            ax.plot([x[0] for x in path], [x[1] for x in path], [x[2] for x in path], 'bo', markersize=5)
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_title('3D A* Path')
            ax.legend()
            plt.show()
