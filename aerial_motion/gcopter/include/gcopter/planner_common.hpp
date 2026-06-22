#ifndef GCOPTER_PLANNER_COMMON_HPP
#define GCOPTER_PLANNER_COMMON_HPP

#include "gcopter/trajectory.hpp"
#include "gcopter/voxel_map.hpp"
#include "misc/visualizer.hpp"

#include <gcopter/PolyTraj.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace gcopter_planner
{

struct CommonPlannerConfig
{
    std::string worldFrameId = "world";
    double dilateRadius = 0.0;
    double voxelWidth = 0.0;
    std::vector<double> mapBound;
    double timeoutRRT = 0.0;
    double maxVelMag = 0.0;
    double maxBdrMag = 0.0;
    double maxTiltAngle = 0.0;
    double gravAcc = 0.0;
    double weightT = 0.0;
    std::vector<double> chiVec;
    double smoothingEps = 0.0;
    int integralIntervs = 0;
    double relCostTol = 0.0;
    bool useFixedTargetHeight = false;
    double targetHeight = 1.0;
    bool useTargetZ = false;
    bool showPolytopeCorridor = true;

    CommonPlannerConfig() = default;
    explicit CommonPlannerConfig(const ros::NodeHandle &nhPriv);

    std::string validationError() const;
    void validateOrThrow() const;
    double resolveTargetHeight(const geometry_msgs::PoseStamped &msg) const;
};

class PlannerBackend
{
public:
    explicit PlannerBackend(const CommonPlannerConfig &config);

    void setMapPoints(const std::vector<Eigen::Vector3d> &points);
    void setMapVoxels(const std::vector<Eigen::Vector3i> &voxelIds);

    bool query(const Eigen::Vector3d &position) const;
    double voxelScale() const;
    Eigen::Vector3i mapSize() const;
    Eigen::Vector3d mapOrigin() const;
    Eigen::Vector3d mapCorner() const;
    long voxelKey(const Eigen::Vector3d &position) const;
    Eigen::Vector3i voxelIdFromKey(long key) const;
    Eigen::Vector3d clampInsideMap(const Eigen::Vector3d &point,
                                   double clearance) const;

    bool searchPath(const Eigen::Vector3d &start,
                    const Eigen::Vector3d &goal,
                    std::vector<Eigen::Vector3d> &route);
    bool buildCorridor(const std::vector<Eigen::Vector3d> &route,
                       std::vector<Eigen::MatrixX4d> &hPolys);
    bool optimizeTrajectory(const Eigen::Matrix3d &initialState,
                            const Eigen::Matrix3d &finalState,
                            const std::vector<Eigen::MatrixX4d> &hPolys,
                            Trajectory<5> &trajectory,
                            const std::string &plannerLabel);

    static Trajectory<5> timeScaledTrajectory(const Trajectory<5> &input,
                                              double scale);

private:
    bool enforceVelocityLimit(Trajectory<5> &trajectory,
                              const std::string &plannerLabel) const;

    CommonPlannerConfig config_;
    voxel_map::VoxelMap voxelMap_;
    int dilateVoxelRadius_;
};

class PlannerRosInterface
{
public:
    PlannerRosInterface(const CommonPlannerConfig &config,
                        ros::NodeHandle &nh);

    bool odomReceived() const;
    const Eigen::Vector3d &latestPosition() const;
    std::string resolvedOdomTopic() const;

    bool pointCloudToWorld(const sensor_msgs::PointCloud2 &msg,
                           std::vector<Eigen::Vector3d> &pointsWorld,
                           std::string *error = nullptr) const;
    static bool decodePointCloud(const sensor_msgs::PointCloud2 &msg,
                                 std::vector<Eigen::Vector3d> &points,
                                 std::string *error = nullptr);

    void publishTrajectory(const Trajectory<5> &trajectory,
                           double startTime);
    static gcopter::PolyTraj trajectoryMessage(const Trajectory<5> &trajectory,
                                               const ros::Time &startTime,
                                               int trajectoryId);

    Visualizer &visualizer();

private:
    static Eigen::Quaterniond normalizedQuaternion(
        const geometry_msgs::Quaternion &quaternion);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

    CommonPlannerConfig config_;
    ros::NodeHandle nh_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    ros::Subscriber odomSub_;
    ros::Publisher trajectoryPub_;
    Visualizer visualizer_;
    Eigen::Vector3d latestPosition_;
    bool odomReceived_;
    int trajectoryId_;
};

} // namespace gcopter_planner

#endif // GCOPTER_PLANNER_COMMON_HPP
