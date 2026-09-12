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

struct RouteSearchTiming
{
    // Measured from solve() entry; negative means no exact solution observed.
    double first_exact_solution_ms = -1.0;
    // Complete searchPath() duration, including setup and path extraction.
    double total_ms = 0.0;
};

// Shared by root-route guidance and the GCOPTER trajectory optimizer.
struct RoutePlannerConfig
{
    std::string worldFrameId = "world";
    double dilateRadius = 0.0;
    double voxelWidth = 0.0;
    std::vector<double> mapBound;
    double timeoutRRT = 0.0;
    double maxVelMag = 0.0;
    bool fixTargetHeight = false;
    double targetHeight = 1.0;

    RoutePlannerConfig() = default;
    explicit RoutePlannerConfig(const ros::NodeHandle &nhPriv, bool fixedBounds = true);

    std::string validationError() const;
    void validateOrThrow() const;
    double resolveTargetHeight(const geometry_msgs::PoseStamped &msg) const;
};

struct CommonPlannerConfig : RoutePlannerConfig
{
    double maxBdrMag = 0.0;
    double maxTiltAngle = 0.0;
    double gravAcc = 0.0;
    double weightT = 0.0;
    std::vector<double> chiVec;
    double smoothingEps = 0.0;
    int integralIntervs = 0;
    double relCostTol = 0.0;
    bool showPolytopeCorridor = true;

    CommonPlannerConfig() = default;
    explicit CommonPlannerConfig(const ros::NodeHandle &nhPriv);

    std::string validationError() const;
    void validateOrThrow() const;
};

class RoutePlannerBackend
{
public:
    explicit RoutePlannerBackend(const RoutePlannerConfig &config);

    RoutePlannerBackend(const RoutePlannerConfig &config, const Eigen::Vector3d &origin,
                        const Eigen::Vector3i &size);
    void setInflatedBits(const std::vector<uint8_t>& bits);

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
                    std::vector<Eigen::Vector3d> &route,
                    RouteSearchTiming *timing = nullptr, double timeout = -1.0) const;

protected:
    voxel_map::VoxelMap voxelMap_;

private:
    RoutePlannerConfig config_;
    int dilateVoxelRadius_;
};

class PlannerBackend : public RoutePlannerBackend
{
public:
    explicit PlannerBackend(const CommonPlannerConfig &config);

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
};

class PlannerRosInterface
{
public:
    PlannerRosInterface(const RoutePlannerConfig &config,
                        ros::NodeHandle &nh);

    bool odomReceived() const;
    const Eigen::Vector3d &latestPosition() const;
    const Eigen::Quaterniond &latestOrientation() const;
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

    RoutePlannerConfig config_;
    ros::NodeHandle nh_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    ros::Subscriber odomSub_;
    ros::Publisher trajectoryPub_;
    Visualizer visualizer_;
    Eigen::Vector3d latestPosition_;
    Eigen::Quaterniond latestOrientation_;
    bool odomReceived_;
    int trajectoryId_;
};

} // namespace gcopter_planner

#endif // GCOPTER_PLANNER_COMMON_HPP
