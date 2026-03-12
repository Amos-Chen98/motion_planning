import sys
from pathlib import Path
path_of_interest = str(Path(__file__).resolve().parents[3])
sys.path.insert(0, path_of_interest)
from scripts.planner.robot_model import Robot
from scripts.ros_node.motion_planner_node import MotionPlanner
import numpy as np
import matplotlib.pyplot as plt
import rospy


# Before running this script, run: roslaunch motion_planner motion_planner.launch

rospy.init_node("fc_vis", anonymous=True)
robot_urdf = path_of_interest + "/urdf/hydrus_xi_20241227.urdf"  # removes 'scripts'

# Create robot instance first
robot = Robot(robot_urdf)

# Create planner config manually (since we need robot instance for some parameters)
planner_config = {}
planner_config['fc_threshold_factor'] = rospy.get_param(
    '/hydrus_xi/motion_planner/fc_threshold_factor', 0.5)
planner_config['joint_angle_bound'] = rospy.get_param("~joint_angle_bound", np.pi / 2)
planner_config['joint_num'] = robot.param['joint_num']
planner_config['link_L'] = robot.param['link_L']

print("fc_threshold_factor: ", planner_config['fc_threshold_factor'])

print("Planner initialized successfully.")


candidate_joint_angles = np.linspace(0, planner_config['joint_angle_bound'], 50)
fc_rp_dis_min_array = np.zeros(len(candidate_joint_angles))
for i, joint_angle in enumerate(candidate_joint_angles):
    fake_rootlink_state = np.array([0, 0, 0])
    candidate_state = np.hstack((fake_rootlink_state,
                                 joint_angle*np.ones(planner_config['joint_num'])))
    fc_rp_dists, _ = robot.get_fc_rp_dists_grad(candidate_state)
    fc_rp_dis_min_array[i] = np.min(fc_rp_dists)


fc_rp_dis_thres = planner_config['link_L'] * planner_config['fc_threshold_factor']

print("Min fc_rp_dis allowed: ", fc_rp_dis_thres)


plt.figure(figsize=(10, 6))
plt.plot(candidate_joint_angles, fc_rp_dis_min_array, label='fc_rp_dis_min')


plt.axhline(y=fc_rp_dis_thres, color='r', linestyle='--',
            label=f'Threshold: {fc_rp_dis_thres:.3f}')


idx_thr = np.argmin(np.abs(fc_rp_dis_min_array - fc_rp_dis_thres))
x_thr = candidate_joint_angles[idx_thr]
y_thr = fc_rp_dis_min_array[idx_thr]
plt.scatter(x_thr, y_thr, color='green', s=100, zorder=5, label='Intersection')
plt.axvline(x=x_thr, color='g', linestyle=':', alpha=0.7)
plt.text(x_thr, plt.ylim()[0], f'{x_thr:.3f}',
         ha='center', va='top', color='g')


idx_max = np.argmax(fc_rp_dis_min_array)
x_max = candidate_joint_angles[idx_max]
y_max = fc_rp_dis_min_array[idx_max]
plt.scatter(x_max, y_max, color='purple', s=100, zorder=5, label='Max fc_rp_dis_min')
plt.axvline(x=x_max, color='purple', linestyle='--', alpha=0.7)


y_axis_min = plt.ylim()[0]
plt.text(
    x_max, y_axis_min,
    f'{x_max:.3f}',
    ha='center', va='top',
    color='purple'
)


plt.xlabel('Joint Angle')
plt.ylabel('FC RP Distance Min')
plt.title('FC RP Distance Min vs Joint Angle')
plt.legend()
plt.grid(True)
plt.show()
