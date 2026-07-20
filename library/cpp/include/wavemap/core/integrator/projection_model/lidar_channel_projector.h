#ifndef WAVEMAP_CORE_INTEGRATOR_PROJECTION_MODEL_LIDAR_CHANNEL_PROJECTOR_H_
#define WAVEMAP_CORE_INTEGRATOR_PROJECTION_MODEL_LIDAR_CHANNEL_PROJECTOR_H_

#include <string>
#include <utility>
#include <vector>

#include "wavemap/core/config/config_base.h"
#include "wavemap/core/integrator/projection_model/circular_projector.h"
#include "wavemap/core/integrator/projection_model/projector_base.h"

namespace wavemap {
/**
 * Config struct for the lidar channel projection model.
 */
struct LidarChannelProjectorConfig
    : ConfigBase<LidarChannelProjectorConfig, 2, CircularProjectorConfig> {
  //! Path to a file listing this lidar's per-channel elevation angles.
  //! Expected format: CSV with a header row and an "Elevation" column
  //! (values in degrees), one row per channel -- the same format as the
  //! per-channel calibration/correction tables shipped by lidar vendors
  //! (e.g. "Channel,Elevation,Azimuth"). Rows do not need to be pre-sorted;
  //! they are sorted by elevation angle on load.
  std::string elevation_angles_file;
  //! Properties of the projection model along the azimuth axis. Unlike
  //! elevation, azimuth is assumed to have a uniform (regular) pitch.
  CircularProjectorConfig azimuth;

  static MemberMap memberMap;

  // Constructors
  LidarChannelProjectorConfig() = default;
  LidarChannelProjectorConfig(std::string elevation_angles_file,
                              CircularProjectorConfig azimuth)
      : elevation_angles_file(std::move(elevation_angles_file)),
        azimuth(std::move(azimuth)) {}

  bool isValid(bool verbose) const override;
};

// A spherical-style projection model for lidars whose vertical channels are
// NOT evenly spaced. Unlike SphericalProjector/OusterProjector, which assume
// a constant elevation pitch derived from {min_angle, max_angle, num_cells},
// this projector loads the actual per-channel elevation angles from a file
// and looks them up via binary search + linear interpolation instead of O(1)
// affine arithmetic. This avoids the elevation-bin collisions/gaps that
// occur when an irregularly spaced channel layout (e.g. many real
// mechanical multi-channel lidars) is forced onto a uniform grid.
class LidarChannelProjector : public ProjectorBase {
 public:
  using Config = LidarChannelProjectorConfig;

  explicit LidarChannelProjector(const Config& config);

  Eigen::Matrix<bool, 3, 1> sensorAxisIsPeriodic() const final;
  Eigen::Matrix<bool, 3, 1> sensorAxisCouldBePeriodic() const final {
    // Elevation is a bounded list of channels, never periodic. Azimuth could
    // be, exactly like for SphericalProjector/OusterProjector.
    return {false, true, false};
  }
  SiUnit getImageCoordinatesUnit() const final { return SiUnit::kRadians; }

  // Coordinate transforms between Cartesian and sensor space
  SensorCoordinates cartesianToSensor(const Point3D& C_point) const final;
  Point3D sensorToCartesian(const SensorCoordinates& coordinates) const final;
  FloatingPoint imageOffsetToErrorSquaredNorm(
      const ImageCoordinates& linearization_point,
      const Vector2D& offset) const final;
  std::array<FloatingPoint, 4> imageOffsetsToErrorSquaredNorms(
      const ImageCoordinates& linearization_point,
      const CellToBeamOffsetArray& offsets) const final;

  // Projection from Cartesian space onto the sensor's image surface
  ImageCoordinates cartesianToImage(const Point3D& C_point) const final;
  FloatingPoint cartesianToSensorZ(const Point3D& C_point) const final;

  // NOTE: When the AABB is right behind the sensor, the angle range will wrap
  //       around at +-PI and a min_angle >= max_angle will be returned.
  AABB<Vector3D> cartesianToSensorAABB(
      const AABB<Point3D>& W_aabb,
      const Transformation3D::RotationMatrix& R_C_W,
      const Point3D& t_W_C) const final;

  // Overridden to support the elevation axis' irregular spacing -- see
  // ProjectorBase's NOTEs on these two methods for why they're virtual.
  Vector2D imageToIndexReal(
      const ImageCoordinates& image_coordinates) const final;
  ImageCoordinates indexToImage(const Index2D& index) const final;

 private:
  const Config config_;
  // Azimuth keeps a regular pitch, so its index math is delegated to the
  // same CircularProjector used by SphericalProjector/OusterProjector.
  const CircularProjector azimuth_projector_;
  // Elevation angles in radians, sorted ascending. Loaded from
  // config_.elevation_angles_file.
  const std::vector<FloatingPoint> elevation_angles_;

  // Delegating constructor: loads the elevation angle file exactly once and
  // uses it both to size/bound the base ProjectorBase and to initialize
  // elevation_angles_.
  LidarChannelProjector(const Config& config,
                       std::vector<FloatingPoint> elevation_angles);

  // Real-valued (fractional) elevation index for a given elevation angle,
  // via binary search + linear interpolation over elevation_angles_.
  FloatingPoint elevationAngleToRealIndex(FloatingPoint angle) const;
  // Elevation angle for a given (possibly fractional or out-of-bounds)
  // elevation index, via lookup/linear interpolation/extrapolation.
  FloatingPoint elevationIndexToAngle(FloatingPoint index) const;

  static std::vector<FloatingPoint> loadElevationAngles(
      const std::string& file_path);
};
}  // namespace wavemap

#include "wavemap/core/integrator/projection_model/impl/lidar_channel_projector_inl.h"

#endif  // WAVEMAP_CORE_INTEGRATOR_PROJECTION_MODEL_LIDAR_CHANNEL_PROJECTOR_H_
