#include <pcd_self_filter/cloud_validation.h>
#include <pcd_self_filter/model_adapter.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <robot_body_filter/RobotBodyFilter.h>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>

namespace pcd_self_filter
{
class ConfirmedTfWatchdog : public robot_body_filter::TFFramesWatchdog
{
public:
  using TFFramesWatchdog::TFFramesWatchdog;

  // Called only after every required transform was found at the observation
  // time. Upstream's periodic discovery otherwise drops the first cloud from
  // a new sensor frame, even if the transform is already in its buffer.
  void confirm(const std::string& frame)
  {
    addMonitoredFrame(frame);
    markReachable(frame);
  }
};

// Only integration checks live here. Geometry, containment and coordinate
// transforms are implemented by the unmodified upstream filter.
class StrictBodyFilter : public robot_body_filter::RobotBodyFilterPointCloud2
{
public:
  explicit StrictBodyFilter(const ros::NodeHandle& nh)
  {
    nodeHandle = nh;
    privateNodeHandle = nh;
    failWithoutRobotDescription = true;
  }

  void initialize(XmlRpc::XmlRpcValue config, const FilterModel& model)
  {
    // Install the observation-aware watchdog before upstream configuration.
    // Replacing an already running ROS-time watchdog would block startup while
    // a rosbag clock is paused. Start it only after model loading succeeds.
    tfBuffer = std::make_shared<tf2_ros::Buffer>(ros::Duration(60.0));
    tfListener = std::make_unique<tf2_ros::TransformListener>(*tfBuffer);
    confirmed_watchdog_ = std::make_shared<ConfirmedTfWatchdog>(
        std::string(config["params"]["frames/filtering"]),
        std::set<std::string>(model.collision_frames.begin(), model.collision_frames.end()),
        tfBuffer, ros::Duration(0), ros::Rate(1.0));
    tfFramesWatchdog = confirmed_watchdog_;
    if (!filters::FilterBase<sensor_msgs::PointCloud2>::configure(config))
      throw std::runtime_error("robot_body_filter configuration failed");
    // Dynamic updates must not replace the adapted private model with another
    // application's unprefixed model.
    robotDescriptionUpdatesListener.shutdown();
    std::set<std::string> loaded;
    for (const auto& shape : shapesToLinks)
      loaded.insert(shape.second.cacheKey);
    if (loaded.size() != model.collision_count)
      throw std::runtime_error("Not all collision shapes loaded; check mesh resources");
    frames_ = model.collision_frames;
    confirmed_watchdog_->start();
  }

  bool waitForTransforms(const sensor_msgs::PointCloud2& cloud, double timeout,
                         std::string& error)
  {
    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(timeout);
    auto frames = frames_;
    frames.push_back(cloud.header.frame_id);
    // Share the exact buffer used by upstream, not a second independently
    // populated listener. The deadline is shared across all links.
    do
    {
      bool complete = true;
      for (const auto& frame : frames)
      {
        if (!tfBuffer->canTransform(filteringFrame, frame, cloud.header.stamp,
                                    ros::Duration(0), &error))
        {
          complete = false;
          error = "Missing TF at cloud time for " + frame + ": " + error;
          break;
        }
      }
      if (complete && confirmed_watchdog_->isRunning())
      {
        for (const auto& frame : frames)
          confirmed_watchdog_->confirm(frame);
        return true;
      }
      if (complete)
        error = "TF watchdog is starting";
      const double remaining = (deadline - ros::WallTime::now()).toSec();
      if (remaining <= 0.0)
        break;
      ros::WallDuration(std::min(0.005, remaining)).sleep();
    } while (ros::ok());
    return false;
  }

  bool filter(const sensor_msgs::PointCloud2& input, sensor_msgs::PointCloud2& output,
              std::string& error)
  {
    // Upstream's iterators assume nonempty storage. An empty, valid observation
    // is still meaningful to a mapper configured to replace its map.
    if (input.width == 0)
    {
      output = input;
      output.header.frame_id = outputFrame;
      return true;
    }
    if (!RobotBodyFilterPointCloud2::update(input, output))
    {
      error = "robot_body_filter is waiting for TF or rejected the observation";
      return false;
    }
    // Upstream can skip a link when its watchdog cannot supply a transform.
    // Reject that result rather than publishing a partially filtered scan.
    std::lock_guard<std::mutex> lock(*modelMutex);
    for (const auto& shape : shapesToLinks)
      if (transformCache.find(shape.second.cacheKey) == transformCache.end())
      {
        error = "Incomplete body transform cache: " + shape.second.link->name;
        return false;
      }
    return true;
  }

private:
  std::vector<std::string> frames_;
  std::shared_ptr<ConfirmedTfWatchdog> confirmed_watchdog_;
};

class SelfFilterNode
{
public:
  SelfFilterNode() : nh_("~")
  {
    nh_.param<std::string>("robot_ns", robot_ns_, "dragon");
    robot_ns_ = normalizedFrame(robot_ns_);
    while (!robot_ns_.empty() && robot_ns_.back() == '/')
      robot_ns_.pop_back();
    const std::string robot_path = robot_ns_.empty() ? "" : "/" + robot_ns_;
    nh_.param<std::string>("robot_description_param", model_param_, robot_path + "/robot_description");
    nh_.param<std::string>("tf_prefix", tf_prefix_, robot_ns_);
    nh_.param<std::string>("world_frame_id", world_frame_, "world");
    world_frame_ = normalizedFrame(world_frame_);
    nh_.param("padding", padding_, 0.02);
    nh_.param("tf_timeout", tf_timeout_, 0.2);
    nh_.param("debug", debug_, false);
    int queue_size;
    nh_.param("queue_size", queue_size, 5);
    if (world_frame_.empty() || !std::isfinite(padding_) || padding_ < 0.0 ||
        !std::isfinite(tf_timeout_) || tf_timeout_ < 0.0 || queue_size <= 0)
      throw std::invalid_argument("Invalid frame, padding, TF timeout or queue size");

    const auto input_topic = topic("input", robot_path + "/cloud_registered_body");
    const auto output_topic = topic("output", robot_path + "/cloud_self_filtered");
    if (input_topic == output_topic)
      throw std::invalid_argument("Input and output topics must be different");
    output_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic, 1);
    diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1, true);
    input_sub_ = nh_.subscribe(input_topic, queue_size,
                              &SelfFilterNode::cloudCallback, this,
                              ros::TransportHints().tcpNoDelay());
    timer_ = nh_.createWallTimer(ros::WallDuration(1.0), &SelfFilterNode::timerCallback, this);
    tryInitialize();
    publishDiagnostics();
  }

private:
  std::string topic(const std::string& key, const std::string& default_topic) const
  {
    const auto resolved = nh_.resolveName(key);
    return resolved == nh_.resolveName(key, false) ? default_topic : resolved;
  }

  void tryInitialize()
  {
    if (filter_)
      return;
    std::string source;
    if (!nh_.getParam(model_param_, source) || source.empty())
    {
      last_error_ = "Waiting for " + model_param_;
      ROS_WARN_THROTTLE(5.0, "%s", last_error_.c_str());
      return;
    }
    if (source == last_attempted_model_)
      return;
    last_attempted_model_ = source;
    try
    {
      const FilterModel model = makeFilterModel(source, tf_prefix_);
      const auto private_model = nh_.resolveName("filter_robot_description");
      nh_.setParam("filter_robot_description", model.xml);
      XmlRpc::XmlRpcValue config;
      config["name"] = "pcd_self_filter";
      config["type"] = "robot_body_filter/RobotBodyFilterPointCloud2";
      auto& params = config["params"];
      params["body_model/robot_description_param"] = private_model;
      params["body_model/inflation/padding"] = padding_;
      XmlRpc::XmlRpcValue per_link;
      if (nh_.getParam("per_link_padding", per_link))
      {
        if (per_link.getType() != XmlRpc::XmlRpcValue::TypeStruct)
          throw std::invalid_argument("per_link_padding must be a dictionary");
        for (auto it = per_link.begin(); it != per_link.end(); ++it)
        {
          if (it->second.getType() != XmlRpc::XmlRpcValue::TypeDouble &&
              it->second.getType() != XmlRpc::XmlRpcValue::TypeInt)
            throw std::invalid_argument("per_link_padding values must be numeric");
          const double value = it->second.getType() == XmlRpc::XmlRpcValue::TypeInt ?
              static_cast<int>(it->second) : static_cast<double>(it->second);
          if (!std::isfinite(value) || value < 0.0)
            throw std::invalid_argument("per_link_padding values must be finite and nonnegative");
          it->second = value;
        }
        params["body_model/inflation/per_link/padding"] = per_link;
      }
      params["frames/fixed"] = world_frame_;
      params["frames/filtering"] = world_frame_;
      params["frames/output"] = world_frame_;
      params["frames/sensor"] = std::string("");
      params["sensor/point_by_point"] = false;
      params["filter/do_clipping"] = false;
      params["filter/do_contains_test"] = true;
      params["filter/do_shadow_test"] = false;
      params["filter/keep_clouds_organized"] = false;
      params["transforms/require_all_reachable"] = true;
      // The wrapper waits once for all historical transforms. Upstream should
      // then only read that buffer, without a timeout for each collision shape.
      params["transforms/timeout/reachable"] = 0.0;
      params["transforms/timeout/unreachable"] = tf_timeout_;
      params["debug/pcl/inside"] = debug_;
      params["debug/marker/contains"] = debug_;
      auto filter = std::make_unique<StrictBodyFilter>(nh_);
      filter->initialize(config, model);
      filter_ = std::move(filter);
      last_error_.clear();
      ROS_INFO("Self filter ready: %zu URDF collision shapes, frame=%s, padding=%.3f m",
               model.collision_count, world_frame_.c_str(), padding_);
    }
    catch (const std::exception& error)
    {
      last_error_ = error.what();
      ROS_ERROR("Cannot initialize self filter: %s", last_error_.c_str());
    }
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& message)
  {
    const ros::WallTime start = ros::WallTime::now();
    ++received_;
    input_points_ = uint64_t(message->width) * message->height;
    kept_points_ = 0;
    last_input_stamp_ = message->header.stamp;
    last_error_.clear();
    std::string failure;
    try
    {
      const auto input = prepareCloud(*message);
      if (!filter_)
      {
        failure = "model";
        last_error_ = "Filter model is not ready";
      }
      else if (!filter_->waitForTransforms(input, tf_timeout_, last_error_))
        failure = "tf";
      else
      {
        sensor_msgs::PointCloud2 output;
        if (!filter_->filter(input, output, last_error_))
          failure = "filter";
        else
        {
          kept_points_ = uint64_t(output.width) * output.height;
          output_pub_.publish(output);
          ++published_;
          last_success_stamp_ = output.header.stamp;
        }
      }
    }
    catch (const std::invalid_argument& error)
    {
      failure = "invalid_cloud";
      last_error_ = error.what();
    }
    catch (const std::exception& error)
    {
      failure = "filter";
      last_error_ = error.what();
    }
    if (!failure.empty())
    {
      ++failures_[failure];
      ROS_WARN_THROTTLE(1.0, "Self filter dropped cloud: %s", last_error_.c_str());
    }
    processing_ms_ = (ros::WallTime::now() - start).toSec() * 1000.0;
    publishDiagnostics();
  }

  void timerCallback(const ros::WallTimerEvent&)
  {
    tryInitialize();
    publishDiagnostics();
  }

  void publishDiagnostics()
  {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus status;
    status.name = ros::this_node::getName();
    status.hardware_id = robot_ns_;
    status.level = !filter_ ? diagnostic_msgs::DiagnosticStatus::ERROR :
                   (!last_error_.empty() || published_ == 0 ? diagnostic_msgs::DiagnosticStatus::WARN :
                                                            diagnostic_msgs::DiagnosticStatus::OK);
    status.message = !last_error_.empty() ? last_error_ :
                     (published_ == 0 ? "Waiting for first valid observation" : "Filtering");
    const auto add = [&status](const std::string& key, const std::string& value) {
      diagnostic_msgs::KeyValue item;
      item.key = key;
      item.value = value;
      status.values.push_back(item);
    };
    add("received", std::to_string(received_));
    add("published", std::to_string(published_));
    add("failure_count", std::to_string(received_ - published_));
    add("input_points", std::to_string(input_points_));
    add("kept_points", std::to_string(kept_points_));
    add("filtered_ratio", last_error_.empty() ?
        std::to_string(input_points_ == 0 ? 0.0 : 1.0 - double(kept_points_) / input_points_) : "n/a");
    add("processing_ms", std::to_string(processing_ms_));
    add("last_input_stamp", std::to_string(last_input_stamp_.toNSec()));
    add("last_success_stamp", std::to_string(last_success_stamp_.toNSec()));
    for (const auto& failure : failures_)
      add("failures/" + failure.first, std::to_string(failure.second));
    array.status.push_back(status);
    diagnostics_pub_.publish(array);
  }

  ros::NodeHandle nh_;
  ros::Subscriber input_sub_;
  ros::Publisher output_pub_, diagnostics_pub_;
  ros::WallTimer timer_;
  std::unique_ptr<StrictBodyFilter> filter_;
  std::string robot_ns_, model_param_, tf_prefix_, world_frame_, last_attempted_model_, last_error_;
  double padding_ = 0.02, tf_timeout_ = 0.2, processing_ms_ = 0.0;
  bool debug_ = false;
  uint64_t received_ = 0, published_ = 0, input_points_ = 0, kept_points_ = 0;
  std::map<std::string, uint64_t> failures_;
  ros::Time last_input_stamp_, last_success_stamp_;
};
}  // namespace pcd_self_filter

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcd_self_filter");
  try
  {
    pcd_self_filter::SelfFilterNode node;
    ros::spin();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL("Cannot start self filter: %s", error.what());
    return 1;
  }
  return 0;
}
