#include "wavemap_ros/map_operations/publish_esdf_operation.h"

#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/utils/sdf/quasi_euclidean_sdf_generator.h>
#include <wavemap_msgs/Map.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>

namespace wavemap {
DECLARE_CONFIG_MEMBERS(PublishEsdfOperationConfig,
                      (once_every)
                      (max_distance)
                      (occupancy_threshold)
                      (tree_height)
                      (topic));

bool PublishEsdfOperationConfig::isValid(bool verbose) const {
  bool all_valid = true;
  all_valid &= IS_PARAM_GT(once_every, 0.f, verbose);
  all_valid &= IS_PARAM_GT(max_distance, 0.f, verbose);
  all_valid &= IS_PARAM_GE(tree_height, 0, verbose);
  all_valid &= IS_PARAM_NE(topic, "", verbose);
  return all_valid;
}

PublishEsdfOperation::PublishEsdfOperation(
    const PublishEsdfOperationConfig& config, MapBase::Ptr occupancy_map,
    std::string world_frame, ros::NodeHandle nh_private)
    : MapOperationBase(std::move(occupancy_map)),
      config_(config.checkValid()),
      world_frame_(std::move(world_frame)) {
  esdf_pub_ = nh_private.advertise<wavemap_msgs::Map>(config_.topic, 10, true);
}

bool PublishEsdfOperation::shouldRun(const ros::Time& current_time) {
  return config_.once_every < (current_time - last_run_timestamp_).toSec();
}

void PublishEsdfOperation::run(bool force_run) {
  const ros::Time current_time = ros::Time::now();
  if (force_run || shouldRun(current_time)) {
    publishEsdf(current_time);
    last_run_timestamp_ = current_time;
  }
}

void PublishEsdfOperation::publishEsdf(const ros::Time& current_time) {
  if (occupancy_map_->empty()) {
    return;
  }

  const auto* hashed_map =
      dynamic_cast<const HashedWaveletOctree*>(occupancy_map_.get());
  if (!hashed_map) {
    ROS_WARN_THROTTLE(5.0, "publish_esdf only supports hashed_wavelet_octree");
    return;
  }

  const IndexElement tree_height =
      std::min(config_.tree_height, hashed_map->getTreeHeight() - 1);
  const QuasiEuclideanSDFGenerator sdf_generator{
      config_.max_distance, config_.occupancy_threshold, tree_height};
  const HashedBlocks esdf = sdf_generator.generate(*hashed_map);

  wavemap_msgs::Map map_msg;
  map_msg.header.frame_id = world_frame_;
  map_msg.header.stamp = current_time;
  convert::mapToRosMsg(esdf, map_msg.hashed_blocks.emplace_back());
  esdf_pub_.publish(map_msg);
}
}  // namespace wavemap
