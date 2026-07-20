#ifndef WAVEMAP_CORE_INTEGRATOR_PROJECTION_MODEL_IMPL_PROJECTOR_BASE_INL_H_
#define WAVEMAP_CORE_INTEGRATOR_PROJECTION_MODEL_IMPL_PROJECTOR_BASE_INL_H_

#include <utility>

namespace wavemap {
inline Index2D ProjectorBase::imageToNearestIndex(
    const ImageCoordinates& image_coordinates) const {
  return imageToIndexReal(image_coordinates)
      .array()
      .round()
      .cast<IndexElement>();
}

inline Index2D ProjectorBase::imageToFloorIndex(
    const ImageCoordinates& image_coordinates) const {
  return imageToIndexReal(image_coordinates)
      .array()
      .floor()
      .cast<IndexElement>();
}

inline Index2D ProjectorBase::imageToCeilIndex(
    const ImageCoordinates& image_coordinates) const {
  return imageToIndexReal(image_coordinates)
      .array()
      .ceil()
      .cast<IndexElement>();
}

inline std::pair<Index2D, Vector2D> ProjectorBase::imageToNearestIndexAndOffset(
    const ImageCoordinates& image_coordinates) const {
  // NOTE: The offset is computed as indexToImage(index_rounded) -
  //       image_coordinates rather than (index_rounded - index_real) *
  //       index_to_image_scale_factor_. The two are algebraically equivalent
  //       when the axis has a constant pitch, but only the indexToImage-based
  //       form remains correct for projectors with irregular (non-uniform)
  //       axis spacing, since it goes through indexToImage's virtual
  //       dispatch instead of assuming a single global scale factor.
  const Vector2D index_real = imageToIndexReal(image_coordinates);
  const Index2D index_rounded =
      index_real.array().round().cast<IndexElement>();
  Vector2D image_coordinate_offset =
      indexToImage(index_rounded) - image_coordinates;
  return {index_rounded, std::move(image_coordinate_offset)};
}

inline ProjectorBase::NearestIndexArray ProjectorBase::imageToNearestIndices(
    const ImageCoordinates& image_coordinates) const {
  NearestIndexArray indices;

  // Populate the indices for the min and max corners, where
  // min_corner = [floor_x, floor_y] and max_corner = [ceil_x, ceil_y]
  const Vector2D index_real = imageToIndexReal(image_coordinates);
  indices.col(0) = index_real.array().floor().cast<IndexElement>();
  indices.col(3) = index_real.array().ceil().cast<IndexElement>();

  // Fill in the intermediate corners, where
  // corner_1 = [ceil_x, floor_y] and corner_2 = [floor_x, ceil_y]
  indices(0, 1) = indices(0, 3);
  indices(1, 1) = indices(1, 0);
  indices(0, 2) = indices(0, 0);
  indices(1, 2) = indices(1, 3);

  return indices;
}

inline std::pair<ProjectorBase::NearestIndexArray,
                 ProjectorBase::CellToBeamOffsetArray>
ProjectorBase::imageToNearestIndicesAndOffsets(
    const ImageCoordinates& image_coordinates) const {
  std::pair<NearestIndexArray, CellToBeamOffsetArray> result;
  auto& indices = result.first;
  auto& offsets = result.second;

  // Write the real-valued indices for the min and max corners into a
  // temporary, where min_corner = [floor_x, floor_y], max_corner =
  // [ceil_x, ceil_y]
  const Vector2D index_real = imageToIndexReal(image_coordinates);
  Eigen::Matrix<FloatingPoint, 2, 4> real_indices;
  real_indices.col(0) = index_real.array().floor();
  real_indices.col(3) = index_real.array().ceil();
  // Also fill in the intermediate corners, where
  // corner_1 = [ceil_x, floor_y] and corner_2 = [floor_x, ceil_y]
  real_indices(0, 1) = real_indices(0, 3);
  real_indices(1, 1) = real_indices(1, 0);
  real_indices(0, 2) = real_indices(0, 0);
  real_indices(1, 2) = real_indices(1, 3);

  // Obtain the indices by casting the real-values into integers
  indices = real_indices.cast<IndexElement>();

  // Compute each corner's offset via indexToImage rather than a single
  // global scale factor, so this remains correct for projectors with
  // irregular (non-uniform) axis spacing. See NOTE in
  // imageToNearestIndexAndOffset above.
  for (int corner_idx = 0; corner_idx < 4; ++corner_idx) {
    const Index2D corner_index = indices.col(corner_idx);
    offsets.col(corner_idx) = indexToImage(corner_index) - image_coordinates;
  }

  return result;
}

inline ImageCoordinates ProjectorBase::indexToImage(
    const Index2D& index) const {
  return index.cast<FloatingPoint>().cwiseProduct(
             index_to_image_scale_factor_) +
         image_offset_;
}

inline FloatingPoint ProjectorBase::imageOffsetToErrorNorm(
    const ImageCoordinates& linearization_point, const Vector2D& offset) const {
  return std::sqrt(imageOffsetToErrorSquaredNorm(linearization_point, offset));
}

inline std::array<FloatingPoint, 4> ProjectorBase::imageOffsetsToErrorNorms(
    const ImageCoordinates& linearization_point,
    const ProjectorBase::CellToBeamOffsetArray& offsets) const {
  auto error_norms =
      imageOffsetsToErrorSquaredNorms(linearization_point, offsets);
  for (auto& error_norm : error_norms) {
    error_norm = std::sqrt(error_norm);
  }
  return error_norms;
}

inline Vector2D ProjectorBase::imageToIndexReal(
    const ImageCoordinates& image_coordinates) const {
  return (image_coordinates - image_offset_)
      .cwiseProduct(image_to_index_scale_factor_);
}
}  // namespace wavemap

#endif  // WAVEMAP_CORE_INTEGRATOR_PROJECTION_MODEL_IMPL_PROJECTOR_BASE_INL_H_
