import numpy as np
from scipy import ndimage


class ESDF():
    def __init__(self, safe_dis=0.1):
        self.safe_dis = safe_dis
        self.map_received = False

    def occupancy_map_cb(self, map):
        '''
        Subscribe to the OccupancyGrid and convert it to the ESDF map
        visualize the ESDF map in RViz
        '''
        self.esdf_map, self.occupancy_2d, self.map_resolution, \
            self.map_origin, self.map_height, self.map_width = self.get_esdf_map(map)

        if self.esdf_map.shape[0] < 2 or self.esdf_map.shape[1] < 2:
            self.esdf_grad_y = np.zeros_like(self.esdf_map)
            self.esdf_grad_x = np.zeros_like(self.esdf_map)
            return
        # get the ESDF gradient map
        # grad_x = grad along x axis in map = grad along col in matrix, so y (row) first
        self.esdf_grad_y, self.esdf_grad_x = np.gradient(self.esdf_map)
        self.map_received = True

    @staticmethod
    def get_esdf_map(occupancy_map):
        occupancy_raw = occupancy_map.data  # occupancy_raw is a 1D array
        map_resolution = occupancy_map.info.resolution
        map_origin = occupancy_map.info.origin.position  # the origin of the map is the left-bottom corner
        map_height = occupancy_map.info.height
        map_width = occupancy_map.info.width

        # Convert occupancy_raw to numpy array and binarize directly
        occupancy_2d = np.array(occupancy_raw, dtype=np.uint8)
        occupancy_2d = np.where(occupancy_2d == 100, 1, 0).reshape(map_height, map_width)

        # Calculate ESDF map directly
        esdf_map = ndimage.distance_transform_edt(~occupancy_2d.astype(bool)) * map_resolution

        return esdf_map, occupancy_2d, map_resolution, map_origin, map_height, map_width

    def is_occupied(self, pos):
        '''
        input: x, y in map frame
        return True if the cell is occupied
        '''
        if not self.map_received:
            return False
        x = pos[0]
        y = pos[1]
        # get the real index of the cell
        row_index = int((y - self.map_origin.y) / self.map_resolution)  # row index = y in map frame
        col_index = int((x - self.map_origin.x) / self.map_resolution)  # column index = x in map frame
        if row_index < 0 or row_index >= self.map_height or col_index < 0 or col_index >= self.map_width:
            return False
        else:
            return self.occupancy_2d[row_index, col_index]

    def has_collision(self, pos):
        if not self.map_received:
            return False
        return self.get_edt(pos) < self.safe_dis

    def get_edt(self, pos):
        '''
        input: x, y in map frame
        return the distance to the nearest obstacle
        '''
        if not self.map_received:
            # return a large enough value
            return 10000
        x = pos[0]
        y = pos[1]
        # get the real index of the cell
        row_index = int((y - self.map_origin.y) / self.map_resolution)  # row index = y in map frame
        col_index = int((x - self.map_origin.x) / self.map_resolution)  # column index = x in map frame
        # if index out of range, return a large number
        if row_index < 0 or row_index >= self.map_height or col_index < 0 or col_index >= self.map_width:
            return 10000
        else:
            return self.esdf_map[row_index, col_index]

    def get_edt_grad(self, pos):
        '''
        input: x, y in map frame
        return the gradient of the distance to the nearest obstacle
        '''
        if not self.map_received:
            return [0, 0]
        x = pos[0]
        y = pos[1]
        # get the real index of the cell
        row_index = int((y - self.map_origin.y) / self.map_resolution)
        col_index = int((x - self.map_origin.x) / self.map_resolution)
        if row_index < 0 or row_index >= self.map_height or col_index < 0 or col_index >= self.map_width:
            return [0, 0]
        else:
            return [self.esdf_grad_x[row_index, col_index], self.esdf_grad_y[row_index, col_index]]
