#include <motion_primitive_planner/dragon_geometry.h>

#include <multilink_copilot/follow_the_leader.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motion_primitive_planner
{
namespace
{
constexpr double kEpsilon = 1e-6;

bool extractSegmentIndex(const std::string& name, const std::string& suffix, int& segment)
{
  constexpr char prefix[] = "joint";
  const size_t suffix_position = name.rfind(suffix);
  if (name.compare(0, sizeof(prefix) - 1, prefix) != 0 || suffix_position == std::string::npos ||
      suffix_position + suffix.size() != name.size())
  {
    return false;
  }
  try
  {
    segment = std::stoi(name.substr(sizeof(prefix) - 1, suffix_position - (sizeof(prefix) - 1))) - 1;
  }
  catch (const std::exception&)
  {
    return false;
  }
  return segment >= 0;
}
}  // namespace

double yawFromQuaternion(const Eigen::Quaterniond& quaternion)
{
  if (quaternion.norm() < kEpsilon)
  {
    return 0.0;
  }
  const Eigen::Quaterniond q = quaternion.normalized();
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

double yawFromQuaternion(const geometry_msgs::Quaternion& quaternion)
{
  return yawFromQuaternion(Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y, quaternion.z));
}

RootAttitude rootAttitudeFromQuaternion(const Eigen::Quaterniond& quaternion)
{
  RootAttitude attitude;
  if (quaternion.norm() < kEpsilon)
  {
    return attitude;
  }
  const Eigen::Quaterniond q = quaternion.normalized();
  attitude.yaw = yawFromQuaternion(q);
  const double sine_pitch = 2.0 * (q.w() * q.y() - q.z() * q.x());
  attitude.pitch = std::asin(std::max(-1.0, std::min(1.0, sine_pitch)));
  return attitude;
}

RootAttitude rootAttitudeFromQuaternion(const geometry_msgs::Quaternion& quaternion)
{
  return rootAttitudeFromQuaternion(
      Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y, quaternion.z));
}

RootAttitude tangentAttitude(const Eigen::Vector3d& velocity,
                             const RootAttitude& fallback)
{
  RootAttitude target = fallback;
  if (!velocity.allFinite() || velocity.norm() <= 1e-3)
  {
    return target;
  }
  const double horizontal_speed = velocity.head<2>().norm();
  if (horizontal_speed > 1e-6)
  {
    target.yaw = std::atan2(velocity.y(), velocity.x());
  }
  target.pitch = -std::atan2(velocity.z(), horizontal_speed);
  return target;
}

RootAttitude interpolateRootAttitude(const RootAttitude& start,
                                     const RootAttitude& goal,
                                     double ratio)
{
  const double clamped_ratio = std::max(0.0, std::min(1.0, ratio));
  RootAttitude result;
  result.yaw = start.yaw + clamped_ratio * shortestYawDelta(start.yaw, goal.yaw);
  result.pitch = start.pitch + clamped_ratio * (goal.pitch - start.pitch);
  return result;
}

RootAttitude advanceRootAttitude(const RootAttitude& current,
                                 const Eigen::Vector3d& velocity,
                                 double dt,
                                 const FollowerConfig& config,
                                 bool command_pitch)
{
  if (dt <= 0.0 || velocity.norm() <= 1e-3)
  {
    return current;
  }
  RootAttitude target = tangentAttitude(velocity, current);
  RootAttitude result = current;
  if (config.publish_yaw_command)
  {
    double yaw_delta = shortestYawDelta(current.yaw, target.yaw);
    const double limit = config.max_angular_vel * dt;
    yaw_delta = std::max(-limit, std::min(limit, yaw_delta));
    result.yaw = std::remainder(current.yaw + yaw_delta, 2.0 * M_PI);
  }
  if (command_pitch)
  {
    double pitch_delta = target.pitch - current.pitch;
    const double limit = config.max_angular_vel * dt;
    pitch_delta = std::max(-limit, std::min(limit, pitch_delta));
    result.pitch = current.pitch + pitch_delta;
  }
  return result;
}

Eigen::Matrix3d fluRotation(const RootAttitude& attitude)
{
  return (Eigen::AngleAxisd(attitude.yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(attitude.pitch, Eigen::Vector3d::UnitY())).toRotationMatrix();
}

Eigen::Matrix3d linkRotation(const RootAttitude& attitude)
{
  return fluRotation(attitude) *
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

Eigen::Vector3d worldAngularVelocity(const RootAttitude& attitude,
                                     double yaw_rate,
                                     double pitch_rate)
{
  return Eigen::Vector3d(-pitch_rate * std::sin(attitude.yaw),
                         pitch_rate * std::cos(attitude.yaw),
                         yaw_rate);
}

double advanceYaw(double current_yaw, const Eigen::Vector3d& velocity, double dt,
                  const FollowerConfig& config)
{
  if (!config.publish_yaw_command || dt <= 0.0 || velocity.head<2>().squaredNorm() <= 1e-6)
  {
    return current_yaw;
  }
  double difference = std::remainder(std::atan2(velocity.y(), velocity.x()) - current_yaw, 2.0 * M_PI);
  const double limit = config.max_angular_vel * dt;
  difference = std::max(-limit, std::min(limit, difference));
  return std::remainder(current_yaw + difference, 2.0 * M_PI);
}

DragonModelInfo::DragonModelInfo(const boost::shared_ptr<Dragon::HydrusLikeRobotModel>& model)
{
  if (!model)
  {
    throw std::invalid_argument("DragonModelInfo requires a DRAGON robot model");
  }
  link_num_ = model->getRotorNum();
  link_length_ = model->getLinkLength();
  link_joint_names_ = model->getLinkJointNames();
  link_joint_indices_ = model->getLinkJointIndices();
  pitch_joint_indices_.assign(static_cast<size_t>(std::max(0, link_num_ - 1)), -1);
  yaw_joint_indices_.assign(static_cast<size_t>(std::max(0, link_num_ - 1)), -1);
  for (size_t index = 0; index < link_joint_names_.size(); ++index)
  {
    joint_name_to_index_[link_joint_names_[index]] = static_cast<int>(index);
    int segment = -1;
    if (extractSegmentIndex(link_joint_names_[index], "_pitch", segment) &&
        segment < static_cast<int>(pitch_joint_indices_.size()))
    {
      pitch_joint_indices_[static_cast<size_t>(segment)] = static_cast<int>(index);
    }
    else if (extractSegmentIndex(link_joint_names_[index], "_yaw", segment) &&
             segment < static_cast<int>(yaw_joint_indices_.size()))
    {
      yaw_joint_indices_[static_cast<size_t>(segment)] = static_cast<int>(index);
    }
  }
  if (link_num_ <= 0 || link_length_ <= 0.0 || link_joint_names_.empty() ||
      link_joint_names_.size() != link_joint_indices_.size() ||
      std::find(pitch_joint_indices_.begin(), pitch_joint_indices_.end(), -1) != pitch_joint_indices_.end() ||
      std::find(yaw_joint_indices_.begin(), yaw_joint_indices_.end(), -1) != yaw_joint_indices_.end())
  {
    throw std::runtime_error("Incomplete DRAGON link geometry or joint mapping");
  }
}

bool DragonModelInfo::readCompleteJointState(const sensor_msgs::JointState& message,
                                             Eigen::VectorXd& joint_positions) const
{
  Eigen::VectorXd measured = joint_positions.size() == jointCount() ?
                                 joint_positions : Eigen::VectorXd::Zero(jointCount());
  std::vector<bool> seen(link_joint_names_.size(), false);
  const size_t count = std::min(message.name.size(), message.position.size());
  for (size_t index = 0; index < count; ++index)
  {
    const auto found = joint_name_to_index_.find(message.name[index]);
    if (found == joint_name_to_index_.end() || !std::isfinite(message.position[index]))
    {
      continue;
    }
    measured(found->second) = message.position[index];
    seen[static_cast<size_t>(found->second)] = true;
  }
  if (std::find(seen.begin(), seen.end(), false) != seen.end())
  {
    return false;
  }
  joint_positions = measured;
  return true;
}

DragonCollisionGeometry DragonModelInfo::collisionGeometry() const
{
  DragonCollisionGeometry geometry;
  geometry.link_num = link_num_;
  geometry.link_length = link_length_;
  geometry.pitch_joint_indices = pitch_joint_indices_;
  geometry.yaw_joint_indices = yaw_joint_indices_;
  return geometry;
}

RootCommandKinematics tailFluToRootLinkCommand(const Eigen::Vector3d& tail_position,
                                                const Eigen::Vector3d& tail_velocity,
                                                const Eigen::Matrix3d& tail_flu_rotation,
                                                const Eigen::Vector3d& angular_velocity,
                                                double link_length)
{
  RootCommandKinematics command;
  const Eigen::Matrix3d link_rotation = tail_flu_rotation *
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  command.orientation = Eigen::Quaterniond(link_rotation).normalized();
  command.angular_velocity = angular_velocity;
  const Eigen::Vector3d link_direction = link_rotation.col(0);
  command.position = tail_position - link_length * link_direction;
  command.linear_velocity = tail_velocity - link_length * angular_velocity.cross(link_direction);
  return command;
}

std::vector<Eigen::Vector3d> linkEndpoints(const Eigen::Vector3d& link1_tail,
                                           const Eigen::Matrix3d& root_link_rotation,
                                           const Eigen::VectorXd& joint_positions,
                                           const std::vector<int>& pitch_joint_indices,
                                           const std::vector<int>& yaw_joint_indices,
                                           int link_num,
                                           double link_length)
{
  std::vector<Eigen::Vector3d> endpoints;
  if (link_num <= 0 || link_length <= 0.0)
  {
    return endpoints;
  }
  endpoints.reserve(static_cast<size_t>(link_num + 1));
  Eigen::Matrix3d rotation = root_link_rotation;
  Eigen::Vector3d position = link1_tail - link_length * rotation.col(0);
  endpoints.push_back(position);
  endpoints.push_back(link1_tail);
  position = link1_tail;
  for (int link = 1; link < link_num; ++link)
  {
    const size_t joint = static_cast<size_t>(link - 1);
    const int pitch_index = joint < pitch_joint_indices.size() ? pitch_joint_indices[joint] : -1;
    const int yaw_index = joint < yaw_joint_indices.size() ? yaw_joint_indices[joint] : -1;
    const double pitch = pitch_index >= 0 && pitch_index < joint_positions.size() ? joint_positions(pitch_index) : 0.0;
    const double yaw = yaw_index >= 0 && yaw_index < joint_positions.size() ? joint_positions(yaw_index) : 0.0;
    rotation = rotation * multilink_copilot::follow_the_leader::rotationAroundY(pitch) *
               multilink_copilot::follow_the_leader::rotationAroundZ(yaw);
    position += link_length * rotation.col(0);
    endpoints.push_back(position);
  }
  return endpoints;
}

bool bodyCollides(const std::vector<Eigen::Vector3d>& endpoints,
                  double sample_spacing,
                  const std::function<bool(const Eigen::Vector3d&)>& occupied)
{
  if (endpoints.size() < 2 || sample_spacing <= 0.0 || !occupied)
  {
    return true;
  }
  for (size_t segment = 1; segment < endpoints.size(); ++segment)
  {
    const Eigen::Vector3d delta = endpoints[segment] - endpoints[segment - 1];
    const int sample_count = std::max(1, static_cast<int>(std::ceil(delta.norm() / sample_spacing)));
    for (int sample = 0; sample <= sample_count; ++sample)
    {
      if (occupied(endpoints[segment - 1] + static_cast<double>(sample) / sample_count * delta))
      {
        return true;
      }
    }
  }
  return false;
}

bool wholeBodyCollides(const WholeBodyConfiguration& configuration,
                       const DragonCollisionGeometry& geometry,
                       double sample_spacing,
                       const std::function<bool(const Eigen::Vector3d&)>& occupied)
{
  const std::vector<Eigen::Vector3d> endpoints =
      linkEndpoints(configuration.link1_tail, configuration.root_link_rotation,
                    configuration.joint_positions, geometry.pitch_joint_indices,
                    geometry.yaw_joint_indices, geometry.link_num, geometry.link_length);
  return bodyCollides(endpoints, sample_spacing, occupied);
}

double shortestYawDelta(double start_yaw, double end_yaw)
{
  if (!std::isfinite(start_yaw) || !std::isfinite(end_yaw))
  {
    throw std::invalid_argument("Yaw endpoints must be finite");
  }
  return std::remainder(end_yaw - start_yaw, 2.0 * M_PI);
}

}  // namespace motion_primitive_planner
