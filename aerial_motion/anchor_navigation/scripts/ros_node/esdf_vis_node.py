import os
import sys
current_dir = os.path.dirname(os.path.dirname(__file__))
sys.path.insert(0, current_dir)
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from map_server.esdf import ESDF



class ESDFVis():

    def __init__(self, node_name="esdf_vis"):
        # Node
        rospy.init_node(node_name, anonymous=False)

        # Subscriber
        self.occupancy_map_sub = rospy.Subscriber('/projected_map', OccupancyGrid, self.show_esdf)

        # Publisher
        self.esdf_map_pub = rospy.Publisher('/esdf_map', OccupancyGrid, queue_size=10)

    def show_esdf(self, map):
        '''
        Get the occupancy map and convert it to the ESDF map
        visualize the ESDF map in RViz
        '''
        esdf_map = ESDF.get_esdf_map(map)[0]
        # guard against empty ESDF
        if esdf_map.size == 0:
            return

        # For visualization only, no physical meaning
        map_range = (98, 1)  # 100 is occupied, 0 is free, map the minimum dis to 98, and the maximum dis to 1
        vmin, vmax = np.min(esdf_map), np.max(esdf_map)
        # avoid degenerate mapping when all values are equal
        if vmin == vmax:
            esdf_map_show = np.full_like(esdf_map, np.mean(map_range))
        else:
            esdf_map_show = np.interp(esdf_map, (vmin, vmax), map_range)

        esdf_map_msg = map
        esdf_map_msg.data = tuple(int(x) for x in esdf_map_show.reshape(-1))  # convert to 1D array
        self.esdf_map_pub.publish(esdf_map_msg)


if __name__ == "__main__":
    esdf_vis = ESDFVis()

    rospy.spin()
