import sys
from pathlib import Path
path_of_interest = str(Path(__file__).resolve().parents[3])
sys.path.insert(0, path_of_interest)
import coal
from geometry_msgs.msg import Pose
import pinocchio as pin
from sensor_msgs.msg import JointState
import rospy
import numpy as np
from gazebo_msgs.msg import LinkStates
from scripts.planner.global_planner import RobotConfig
from tf.transformations import euler_from_quaternion


class CollisionDetector():
    def __init__(self, node_name="collision_detector"):
        # Node
        rospy.init_node(node_name, anonymous=False)
        robot_urdf = path_of_interest + "/urdf/hydrus_xi_20241227.urdf"
        obstacle_urdf = path_of_interest + "/urdf/2d_opening.urdf"

        print("Robot URDF: ", robot_urdf)
        print("Obstacle URDF: ", obstacle_urdf)

        # Variables
        self.rootlink_state = Pose()
        self.joint_state = np.zeros(7)
        self.robot_config = RobotConfig()

        # Flag
        self.rootlink_state_received = False
        self.joint_state_received = False

        # Subscribers
        self.rootlink_state_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.rootlink_state_cb)
        self.joint_state_sub = rospy.Subscriber('joint_states', JointState, self.joint_state_cb)

        # Pinocchio
        self.init_collision_detector(robot_urdf, obstacle_urdf)

        # run collision check
        self.collision_detection_timer = rospy.Timer(rospy.Duration(1.0), self.check_collision)

    def init_collision_detector(self, robot_urdf, obstacle_urdf):
        self.robot_model = pin.buildModelFromUrdf(robot_urdf, pin.JointModelFreeFlyer())  # use JointModelFreeFlyer() for floating base
        self.robot_data = self.robot_model.createData()
        self.robot_geom_model = pin.buildGeomFromUrdf(self.robot_model, robot_urdf, pin.GeometryType.COLLISION)
        self.robot_geom_data = self.robot_geom_model.createData()

        self.obstacle_model = pin.buildModelFromUrdf(obstacle_urdf)
        self.obstacle_data = self.obstacle_model.createData()
        self.obstacle_geom_model = pin.buildGeomFromUrdf(self.obstacle_model, obstacle_urdf, pin.GeometryType.COLLISION)
        self.obstacle_geom_data = self.obstacle_geom_model.createData()

        robot_geom_obj_num = len(self.robot_geom_model.geometryObjects)
        obstacle_geom_obj_num = len(self.obstacle_geom_model.geometryObjects)

        # merge robot and obstacle models into one geom model
        self.merged_geom_model = pin.GeometryModel()

        for geom_obj in self.robot_geom_model.geometryObjects:
            self.merged_geom_model.addGeometryObject(geom_obj)
        for geom_obj in self.obstacle_geom_model.geometryObjects:
            self.merged_geom_model.addGeometryObject(geom_obj)

        print("Geometry objects in merged model:")
        print("-" * 40)
        print(f"{'obj_id':<10}{'obj_name':<15}{'parent_joint_id':<17}{'parent_joint_name':<15}")
        print("-" * 40)
        for obj_id in range(len(self.merged_geom_model.geometryObjects)):
            obj = self.merged_geom_model.geometryObjects[obj_id]
            obj_name = obj.name
            obj_parent_joint = obj.parentJoint
            if obj_id >= robot_geom_obj_num:
                obj_parent_joint_name = self.obstacle_model.names[obj_parent_joint]
            else:
                obj_parent_joint_name = self.robot_model.names[obj_parent_joint]
            print(f"{obj_id:<10}{obj_name:<15}{obj_parent_joint:<17}{obj_parent_joint_name:<15}")
        print("-" * 40)

        # add collision pairs
        for i in range(robot_geom_obj_num):
            for j in range(robot_geom_obj_num, robot_geom_obj_num + obstacle_geom_obj_num):
                collision_pair = pin.CollisionPair(i, j)
                self.merged_geom_model.addCollisionPair(collision_pair)

        self.merged_geom_data = self.merged_geom_model.createData()

        # print all collision pairs
        print("Collision pairs:")
        print("-" * 40)
        print(f"{'pair_id':<10}{'first_obj_id':<15}{'first_obj_name':<15}{'second_obj_id':<15}{'second_obj_name':<15}")
        print("-" * 40)
        for i in range(len(self.merged_geom_model.collisionPairs)):
            pair = self.merged_geom_model.collisionPairs[i]
            first_obj_id = pair.first
            second_obj_id = pair.second
            first_obj_name = self.merged_geom_model.geometryObjects[first_obj_id].name
            second_obj_name = self.merged_geom_model.geometryObjects[second_obj_id].name
            print(f"{i:<10}{first_obj_id:<15}{first_obj_name:<15}{second_obj_id:<15}{second_obj_name:<15}")
        print("-" * 40)

        print("Collision detector initialized!")
        print("Robot model nq:", self.robot_model.nq)
        print("Obstacle model nq:", self.obstacle_model.nq)

        self.gimbal1_idx_q = self.robot_model.joints[self.robot_model.getJointId('gimbal1')].idx_q
        self.gimbal2_idx_q = self.robot_model.joints[self.robot_model.getJointId('gimbal2')].idx_q
        self.gimbal3_idx_q = self.robot_model.joints[self.robot_model.getJointId('gimbal3')].idx_q
        self.gimbal4_idx_q = self.robot_model.joints[self.robot_model.getJointId('gimbal4')].idx_q
        self.joint1_idx_q = self.robot_model.joints[self.robot_model.getJointId('joint1')].idx_q
        self.joint2_idx_q = self.robot_model.joints[self.robot_model.getJointId('joint2')].idx_q
        self.joint3_idx_q = self.robot_model.joints[self.robot_model.getJointId('joint3')].idx_q

    def print_obstacle_pose(self):
        for i in range(len(self.obstacle_geom_model.geometryObjects)):
            obj = self.obstacle_geom_model.geometryObjects[i]
            obj_name = obj.name
            obj_placement = self.obstacle_geom_data.oMg[i]
            obj_rotation = obj_placement.rotation
            print("Object :", obj_name)
            obj_rotation_euler = pin.rpy.matrixToRpy(obj_rotation)
            # print the euler in degrees in :2f
            obj_rotation_euler = obj_rotation_euler * 180 / np.pi
            print(f"{'':<10}{'':<15} {obj_rotation_euler[0]:.2f}, {obj_rotation_euler[1]:.2f}, {obj_rotation_euler[2]:.2f}")

    def rootlink_state_cb(self, data):
        # get the index where data.name == 'hydrus_xi::rootlink'
        rootlink_idx = data.name.index('hydrus_xi::root')  # The index might not be static, so gettting it dynamically
        self.rootlink_state = data.pose[rootlink_idx]
        self.robot_config.rootlink_pos_x = self.rootlink_state.position.x
        self.robot_config.rootlink_pos_y = self.rootlink_state.position.y
        # get yaw from quaternion
        euler = euler_from_quaternion([data.pose[rootlink_idx].orientation.x,
                                       data.pose[rootlink_idx].orientation.y,
                                       data.pose[rootlink_idx].orientation.z,
                                       data.pose[rootlink_idx].orientation.w])

        self.robot_config.rootlink_yaw = euler[2]

        self.rootlink_state_received = True

    def joint_state_cb(self, data):
        self.joint_state = np.array(data.position)  # [gimbal 0-4, joint 0-3]
        self.robot_config.joint_angles = self.joint_state[4:]
        self.joint_state_received = True

    def check_collision(self, event):
        if self.rootlink_state_received and self.joint_state_received:
            collision_flag = self.has_collision(self.robot_config)
            self.get_gimbal_pos(self.robot_config)
            if collision_flag:
                rospy.loginfo("Collision detected!")
                # self.print_geomety_objects()
                self.print_collision_results(self.merged_geom_data.collisionResults)
                self.print_distance_results(self.merged_geom_data.distanceResults)
            else:
                rospy.loginfo("No collision detected!")

            print(self.get_minimum_distance(self.merged_geom_data.distanceResults))
            print("\n")

    def has_collision(self, robot_config):
        # rootlink state - len = 7
        robot_q = self.get_robot_q(robot_config)

        pin.updateGeometryPlacements(self.robot_model, self.robot_data, self.merged_geom_model,
                                     self.merged_geom_data, robot_q)  # = forwardKinematics + updateGeometryPlacements

        pin.computeCollisions(self.merged_geom_model, self.merged_geom_data)

        pin.computeDistances(self.merged_geom_model, self.merged_geom_data)

        collision_detected = any(cr.isCollision() for cr in self.merged_geom_data.collisionResults)

        return collision_detected
    
    def get_gimbal_pos(self, robot_config):
        robot_q = self.get_robot_q(robot_config)
        pin.forwardKinematics(self.robot_model, self.robot_data, robot_q)
        pin.updateFramePlacements(self.robot_model, self.robot_data)

        gimbal1_frame = self.robot_data.oMf[self.robot_model.getFrameId("gimbal1")]
        gimbal2_frame = self.robot_data.oMf[self.robot_model.getFrameId("gimbal2")]
        gimbal3_frame = self.robot_data.oMf[self.robot_model.getFrameId("gimbal3")]
        gimbal4_frame = self.robot_data.oMf[self.robot_model.getFrameId("gimbal4")]

        gimbal1_pos = gimbal1_frame.translation
        gimbal2_pos = gimbal2_frame.translation
        gimbal3_pos = gimbal3_frame.translation
        gimbal4_pos = gimbal4_frame.translation

        print("Gimbal 1 pos: ", gimbal1_pos)
        print("Gimbal 2 pos: ", gimbal2_pos)
        print("Gimbal 3 pos: ", gimbal3_pos)
        print("Gimbal 4 pos: ", gimbal4_pos)


    def get_robot_q(self, robot_config):
        robot_q = np.zeros(self.robot_model.nq)
        robot_q[0] = robot_config.rootlink_pos_x
        robot_q[1] = robot_config.rootlink_pos_y
        robot_q[2] = 0.5

        # get quaternion from robot_config.rootlink_yaw
        rootlink_rotation = pin.rpy.rpyToMatrix(np.array([0, 0, robot_config.rootlink_yaw]))
        rootlink_quat = pin.Quaternion(rootlink_rotation)

        robot_q[3] = rootlink_quat.x
        robot_q[4] = rootlink_quat.y
        robot_q[5] = rootlink_quat.z
        robot_q[6] = rootlink_quat.w

        robot_q[self.gimbal1_idx_q] = 1
        robot_q[self.gimbal1_idx_q+1] = 0
        robot_q[self.gimbal2_idx_q] = 1
        robot_q[self.gimbal2_idx_q+1] = 0
        robot_q[self.gimbal3_idx_q] = 1
        robot_q[self.gimbal3_idx_q+1] = 0
        robot_q[self.gimbal4_idx_q] = 1
        robot_q[self.gimbal4_idx_q+1] = 0

        robot_q[self.joint1_idx_q] = robot_config.joint_angles[0]
        robot_q[self.joint2_idx_q] = robot_config.joint_angles[1]
        robot_q[self.joint3_idx_q] = robot_config.joint_angles[2]
        return robot_q

    def print_geomety_objects(self):
        print("-" * 70)
        print(f"{'obj name':<16}{'obj shape':<12}{'obj half size':<21}{'obj pos':<19}{'obj rpy'}")
        print("-" * 70)
        for i in range(len(self.merged_geom_model.geometryObjects)):
            # obj identifier
            obj = self.merged_geom_model.geometryObjects[i]
            obj_name = obj.name

            # obj placement (in :2f)
            obj_placement = self.merged_geom_data.oMg[i]
            obj_translation = obj_placement.translation
            obj_rotation = obj_placement.rotation

            # convert obj_rotation to euler angles
            obj_rotation_euler = pin.rpy.matrixToRpy(obj_rotation)
            obj_rpy = obj_rotation_euler * 180 / np.pi

            # obj geometry
            obj_geom = obj.geometry
            if isinstance(obj_geom, coal.coal_pywrap.Box):
                shape = "Box"
                half_size = obj_geom.halfSide
                size_str = f"{half_size}"
            elif isinstance(obj_geom, coal.coal_pywrap.Cylinder):
                shape = "Cylinder"
                half_length = obj_geom.halfLength
                size_str = f"r: {obj_geom.radius} L/2: {half_length}"
            else:
                shape = "Unknown"
                size_str = "N/A"

            # print obj name, shape, size, translation, and rotation
            print(
                f"{obj_name:<16} {shape:<10} {size_str:<20} "
                f"[{obj_translation[0]:.2f}, {obj_translation[1]:.2f}, {obj_translation[2]:.2f}] "
                f"[{obj_rpy[0]:.2f}, {obj_rpy[1]:.2f}, {obj_rpy[2]:.2f}]"
            )

        print("-" * 70)

    def print_collision_results(self, collision_results):
        for i in range(len(collision_results)):
            if collision_results[i].isCollision():
                first_obj_idx = self.merged_geom_model.collisionPairs[i].first
                second_obj_idx = self.merged_geom_model.collisionPairs[i].second
                first_obj_name = self.merged_geom_model.geometryObjects[first_obj_idx].name
                second_obj_name = self.merged_geom_model.geometryObjects[second_obj_idx].name
                print(f"Collision between: {first_obj_name} - {second_obj_name}")

                contact_num = collision_results[i].numContacts()
                for j in range(contact_num):
                    contact = collision_results[i].getContact(j)
                    print(f"Contact {j}: Position: {contact.pos}, Depth: {contact.penetration_depth}")

    def print_distance_results(self, distance_results):
        print("-" * 45)
        print(f"{'obj_1':<15}{'obj_2':<15}{'distance':<15}")
        print("-" * 45)
        for i in range(len(distance_results)):
            first_obj_idx = self.merged_geom_model.collisionPairs[i].first
            second_obj_idx = self.merged_geom_model.collisionPairs[i].second
            first_obj_name = self.merged_geom_model.geometryObjects[first_obj_idx].name
            second_obj_name = self.merged_geom_model.geometryObjects[second_obj_idx].name
            distance = distance_results[i].min_distance
            print(f"{first_obj_name:<15}{second_obj_name:<15}{distance:.2f}")

    def get_minimum_distance(self, distance_results):
        min_distance = min(distance_results, key=lambda x: x.min_distance)
        return min_distance.min_distance


if __name__ == "__main__":

    collition_detector = CollisionDetector()

    rospy.spin()
