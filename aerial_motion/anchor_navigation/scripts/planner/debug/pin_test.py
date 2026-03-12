import sys
from pathlib import Path
path_of_interest = str(Path(__file__).resolve().parents[3])
sys.path.insert(0, path_of_interest)
import numpy as np
from pinocchio.utils import zero
import pinocchio as pin
import time

print("Pinocchio version:", pin.__version__)
robot_urdf = path_of_interest + "/urdf/hydrus_xi_20241227.urdf"
obstacle_urdf = path_of_interest + "/urdf/2d_opening.urdf"

# load robot and obstacle models
robot_model = pin.buildModelFromUrdf(robot_urdf, pin.JointModelFreeFlyer())  # use JointModelFreeFlyer() for floating base
robot_data = robot_model.createData()
robot_geom_model = pin.buildGeomFromUrdf(robot_model, robot_urdf, pin.GeometryType.COLLISION)
robot_geom_data = robot_geom_model.createData()

obstacle_model = pin.buildModelFromUrdf(obstacle_urdf)
obstacle_data = obstacle_model.createData()
obstacle_geom_model = pin.buildGeomFromUrdf(obstacle_model, obstacle_urdf, pin.GeometryType.COLLISION)
obstacle_geom_data = obstacle_geom_model.createData()

# merge robot and obstacle models into one geom model
merged_geom_model = pin.GeometryModel()

for geom_obj in robot_geom_model.geometryObjects:
    merged_geom_model.addGeometryObject(geom_obj)
for geom_obj in obstacle_geom_model.geometryObjects:
    merged_geom_model.addGeometryObject(geom_obj)

robot_geom_obj_num = len(robot_geom_model.geometryObjects)
obstacle_geom_obj_num = len(obstacle_geom_model.geometryObjects)

# add collision pairs
for i in range(robot_geom_obj_num):
    for j in range(robot_geom_obj_num, robot_geom_obj_num + obstacle_geom_obj_num):
        collision_pair = pin.CollisionPair(i, j)
        merged_geom_model.addCollisionPair(collision_pair)

merged_geom_data = merged_geom_model.createData()

# init config
robot_q = np.ones(robot_model.nq)
obstacle_q = np.zeros(obstacle_model.nq)

pin.forwardKinematics(robot_model, robot_data, robot_q)
pin.updateGeometryPlacements(robot_model, robot_data, merged_geom_model, merged_geom_data)
pin.forwardKinematics(obstacle_model, obstacle_data, obstacle_q)
pin.updateGeometryPlacements(obstacle_model, obstacle_data, merged_geom_model, merged_geom_data)

time_start = time.time()
pin.computeCollisions(merged_geom_model, merged_geom_data)
time_end = time.time()
print("Time elapsed:", time_end - time_start)

collision_results = merged_geom_data.collisionResults

for i in range(len(collision_results)):
    print("Collision result", i, ":", collision_results[i].isCollision())

# collision_detected = any(cr.isCollision() for cr in merged_geom_data.collisionResults)
# print("Collision:", collision_detected)