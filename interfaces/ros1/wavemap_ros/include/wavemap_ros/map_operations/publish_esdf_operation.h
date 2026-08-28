#ifndef WAVEMAP_ROS_MAP_OPERATIONS_PUBLISH_ESDF_OPERATION_H_
#define WAVEMAP_ROS_MAP_OPERATIONS_PUBLISH_ESDF_OPERATION_H_

#include <string>

#include <ros/ros.h>
#include <wavemap/core/config/config_base.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/core/utils/time/time.h>
#include <wavemap/pipeline/map_operations/map_operation_base.h>
#include <wavemap_msgs/Map.h>

namespace wavemap {
struct PublishEsdfOperationConfig
    : public ConfigBase<PublishEsdfOperationConfig, 5> {
  //! Time period controlling how often the ESDF is published.
  Seconds<FloatingPoint> once_every = 2.f;

  //! Maximum distance (meters) computed in the ESDF.
  FloatingPoint max_distance = 4.f;

  //! Log-odds threshold above which cells are treated as occupied.
  FloatingPoint occupancy_threshold = 0.f;

  //! Octree height at which the ESDF is generated. 0 (default) uses the
  //! map's finest resolution (min_cell_width); each level above that
  //! doubles the cell width, trading resolution for speed.
  IndexElement tree_height = 0;

  //! Name of the topic the ESDF map will be published on.
  std::string topic = "esdf";

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};

class PublishEsdfOperation : public MapOperationBase {
 public:
  PublishEsdfOperation(const PublishEsdfOperationConfig& config,
                       MapBase::Ptr occupancy_map, std::string world_frame,
                       ros::NodeHandle nh_private);

  bool shouldRun(const ros::Time& current_time);

  void run(bool force_run) override;

 private:
  const PublishEsdfOperationConfig config_;
  const std::string world_frame_;
  ros::Time last_run_timestamp_;
  ros::Publisher esdf_pub_;

  void publishEsdf(const ros::Time& current_time);
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_MAP_OPERATIONS_PUBLISH_ESDF_OPERATION_H_
