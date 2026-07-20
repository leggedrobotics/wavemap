#include "wavemap/core/integrator/projection_model/lidar_channel_projector.h"

#include <algorithm>
#include <fstream>
#include <sstream>

#include "wavemap/core/utils/math/angle_normalization.h"

namespace wavemap {
namespace {
std::string trim(const std::string& s) {
  constexpr const char* kWhitespace = " \t\r\n";
  const auto start = s.find_first_not_of(kWhitespace);
  if (start == std::string::npos) {
    return "";
  }
  const auto end = s.find_last_not_of(kWhitespace);
  return s.substr(start, end - start + 1);
}
}  // namespace

LidarChannelProjector::LidarChannelProjector(const Config& config)
    : LidarChannelProjector(config,
                            loadElevationAngles(config.elevation_angles_file)) {
}

LidarChannelProjector::LidarChannelProjector(
    const Config& config, std::vector<FloatingPoint> elevation_angles)
    : ProjectorBase(
          {static_cast<IndexElement>(elevation_angles.size()),
           config.azimuth.num_cells},
          Vector2D(
              (elevation_angles.back() - elevation_angles.front()) /
                  static_cast<FloatingPoint>(elevation_angles.size() - 1),
              (config.azimuth.max_angle - config.azimuth.min_angle) /
                  static_cast<FloatingPoint>(config.azimuth.num_cells - 1)),
          {elevation_angles.front(), config.azimuth.min_angle},
          {elevation_angles.front(), config.azimuth.min_angle},
          {elevation_angles.back(), config.azimuth.max_angle}),
      config_(config.checkValid()),
      azimuth_projector_(config.azimuth),
      elevation_angles_(std::move(elevation_angles)) {}

Eigen::Matrix<bool, 3, 1> LidarChannelProjector::sensorAxisIsPeriodic() const {
  // Elevation is a bounded channel list -- never periodic.
  const FloatingPoint y_difference =
      angle_math::normalize_near(config_.azimuth.max_angle +
                                 index_to_image_scale_factor_.y()) -
      config_.azimuth.min_angle;
  return {false,
         0.f <= y_difference && y_difference <= index_to_image_scale_factor_.y(),
         false};
}

AABB<Vector3D> LidarChannelProjector::cartesianToSensorAABB(
    const AABB<Point3D>& W_aabb,
    const kindr::minimal::QuatTransformationTemplate<
        FloatingPoint>::RotationMatrix& R_C_W,
    const Point3D& t_W_C) const {
  AABB<Vector3D> sensor_coordinate_aabb;

  sensor_coordinate_aabb.min.z() = W_aabb.minDistanceTo(t_W_C);
  sensor_coordinate_aabb.max.z() = W_aabb.maxDistanceTo(t_W_C);

  const Point3D C_aabb_min = R_C_W * (W_aabb.min - t_W_C);
  const Transformation3D::RotationMatrix C_aabb_edges =
      R_C_W * (W_aabb.max - W_aabb.min).asDiagonal();

  std::array<Point3D, AABB<Point3D>::kNumCorners> C_aabb_corners;
  for (int corner_idx = 0; corner_idx < AABB<Point3D>::kNumCorners;
       ++corner_idx) {
    C_aabb_corners[corner_idx] = C_aabb_min;
    for (int dim_idx = 0; dim_idx < 3; ++dim_idx) {
      if (bit_ops::is_bit_set(corner_idx, dim_idx)) {
        C_aabb_corners[corner_idx] += C_aabb_edges.col(dim_idx);
      }
    }
  }

  std::array<Vector2D, AABB<Point3D>::kNumCorners> corner_sensor_coordinates;
  for (int corner_idx = 0; corner_idx < AABB<Point3D>::kNumCorners;
       ++corner_idx) {
    corner_sensor_coordinates[corner_idx] =
        cartesianToImage(C_aabb_corners[corner_idx]);
  }

  for (const int axis : {0, 1}) {
    FloatingPoint& min_coordinate = sensor_coordinate_aabb.min[axis];
    FloatingPoint& max_coordinate = sensor_coordinate_aabb.max[axis];
    for (int corner_idx = 0; corner_idx < AABB<Point3D>::kNumCorners;
         ++corner_idx) {
      min_coordinate =
          std::min(min_coordinate, corner_sensor_coordinates[corner_idx][axis]);
      max_coordinate =
          std::max(max_coordinate, corner_sensor_coordinates[corner_idx][axis]);
    }

    const bool angle_interval_wraps_around =
        sensorAxisCouldBePeriodic()[axis] &&
        kPi < (max_coordinate - min_coordinate);
    if (angle_interval_wraps_around) {
      min_coordinate = AABB<Vector3D>::kInitialMin;
      max_coordinate = AABB<Vector3D>::kInitialMax;
      for (int corner_idx = 0; corner_idx < AABB<Point3D>::kNumCorners;
           ++corner_idx) {
        const FloatingPoint angle = corner_sensor_coordinates[corner_idx][axis];
        if (0.f < angle) {
          min_coordinate = std::min(min_coordinate, angle);
        } else {
          max_coordinate = std::max(max_coordinate, angle);
        }
      }
    }
  }

  return sensor_coordinate_aabb;
}

Vector2D LidarChannelProjector::imageToIndexReal(
    const ImageCoordinates& image_coordinates) const {
  return {elevationAngleToRealIndex(image_coordinates[0]),
         azimuth_projector_.angleToRealIndex(image_coordinates[1])};
}

ImageCoordinates LidarChannelProjector::indexToImage(
    const Index2D& index) const {
  return {elevationIndexToAngle(static_cast<FloatingPoint>(index.x())),
         azimuth_projector_.indexToAngle(index.y())};
}

FloatingPoint LidarChannelProjector::elevationAngleToRealIndex(
    FloatingPoint angle) const {
  const auto num_angles = static_cast<IndexElement>(elevation_angles_.size());
  const auto it = std::upper_bound(elevation_angles_.begin(),
                                   elevation_angles_.end(), angle);
  IndexElement i;
  if (it == elevation_angles_.begin()) {
    i = 0;  // At or below the first channel -- extrapolate from segment 0.
  } else if (it == elevation_angles_.end()) {
    i = num_angles - 2;  // At or above the last channel -- extrapolate.
  } else {
    i = static_cast<IndexElement>(it - elevation_angles_.begin()) - 1;
  }
  const FloatingPoint lower = elevation_angles_[i];
  const FloatingPoint upper = elevation_angles_[i + 1];
  return static_cast<FloatingPoint>(i) + (angle - lower) / (upper - lower);
}

FloatingPoint LidarChannelProjector::elevationIndexToAngle(
    FloatingPoint index) const {
  const auto num_angles = static_cast<IndexElement>(elevation_angles_.size());
  IndexElement i = static_cast<IndexElement>(std::floor(index));
  i = std::max<IndexElement>(0, std::min(i, num_angles - 2));
  const FloatingPoint lower = elevation_angles_[i];
  const FloatingPoint upper = elevation_angles_[i + 1];
  const FloatingPoint frac = index - static_cast<FloatingPoint>(i);
  return lower + frac * (upper - lower);
}

std::vector<FloatingPoint> LidarChannelProjector::loadElevationAngles(
    const std::string& file_path) {
  std::ifstream file(file_path);
  CHECK(file.is_open()) << "LidarChannelProjector could not open elevation "
                          "angles file \""
                       << file_path << "\".";

  std::string header_line;
  CHECK(static_cast<bool>(std::getline(file, header_line)))
      << "LidarChannelProjector: elevation angles file \"" << file_path
      << "\" is empty (expected a header row).";

  std::vector<std::string> header_fields;
  {
    std::stringstream ss(header_line);
    std::string field;
    while (std::getline(ss, field, ',')) {
      header_fields.emplace_back(trim(field));
    }
  }
  const auto elevation_col_it =
      std::find(header_fields.begin(), header_fields.end(), "Elevation");
  CHECK(elevation_col_it != header_fields.end())
      << "LidarChannelProjector: elevation angles file \"" << file_path
      << "\" has no \"Elevation\" column in its header.";
  const auto elevation_col =
      static_cast<size_t>(elevation_col_it - header_fields.begin());

  std::vector<FloatingPoint> elevation_angles_deg;
  std::string line;
  while (std::getline(file, line)) {
    if (trim(line).empty()) {
      continue;
    }
    std::stringstream ss(line);
    std::string field;
    size_t col_idx = 0;
    while (std::getline(ss, field, ',')) {
      if (col_idx == elevation_col) {
        elevation_angles_deg.push_back(std::stof(trim(field)));
        break;
      }
      ++col_idx;
    }
  }
  CHECK_LE(2u, elevation_angles_deg.size())
      << "LidarChannelProjector: elevation angles file \"" << file_path
      << "\" must list at least 2 channels (found "
      << elevation_angles_deg.size() << ").";

  std::sort(elevation_angles_deg.begin(), elevation_angles_deg.end());
  std::vector<FloatingPoint> elevation_angles_rad;
  elevation_angles_rad.reserve(elevation_angles_deg.size());
  for (size_t i = 0; i < elevation_angles_deg.size(); ++i) {
    if (0 < i) {
      CHECK_LT(elevation_angles_deg[i - 1], elevation_angles_deg[i])
          << "LidarChannelProjector: elevation angles file \"" << file_path
          << "\" contains a duplicate elevation angle ("
          << elevation_angles_deg[i - 1]
          << " deg appears more than once), which would make the elevation "
             "axis non-monotonic.";
    }
    elevation_angles_rad.push_back(elevation_angles_deg[i] * kPi / 180.f);
  }
  return elevation_angles_rad;
}

DECLARE_CONFIG_MEMBERS(LidarChannelProjectorConfig,
                      (elevation_angles_file)(azimuth));

bool LidarChannelProjectorConfig::isValid(bool verbose) const {
  bool is_valid = true;
  is_valid &= IS_PARAM_NE(elevation_angles_file, std::string(""), verbose);
  is_valid &= azimuth.isValid(verbose);
  return is_valid;
}
}  // namespace wavemap
