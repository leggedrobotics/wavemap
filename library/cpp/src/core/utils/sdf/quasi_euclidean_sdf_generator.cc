#include "wavemap/core/utils/sdf/quasi_euclidean_sdf_generator.h"

#include <wavemap/core/utils/profiler_interface.h>

#include "wavemap/core/indexing/index_conversions.h"
#include "wavemap/core/utils/iterate/grid_iterator.h"
#include "wavemap/core/utils/query/query_accelerator.h"

namespace wavemap {
HashedBlocks QuasiEuclideanSDFGenerator::generate(
    const HashedWaveletOctree& occupancy_map) const {
  ProfilerZoneScoped;
  // Initialize the SDF data structure
  const FloatingPoint cell_width = convert::heightToCellWidth(
      occupancy_map.getMinCellWidth(), tree_height_);
  const MapBaseConfig config{cell_width, 0.f, max_distance_};
  HashedBlocks sdf(config, max_distance_);

  // Initialize the bucketed priority queue
  const int num_bins =
      static_cast<int>(std::ceil(max_distance_ / cell_width));
  BucketQueue<Index3D> open{num_bins, max_distance_};

  // Seed and propagate the SDF
  seed(occupancy_map, sdf, open);
  propagate(occupancy_map, sdf, open);

  return sdf;
}

void QuasiEuclideanSDFGenerator::seed(const HashedWaveletOctree& occupancy_map,
                                      HashedBlocks& sdf,
                                      BucketQueue<Index3D>& open_queue) const {
  ProfilerZoneScoped;
  // Create an occupancy query accelerator
  QueryAccelerator occupancy_query_accelerator{occupancy_map};

  // For all free cells that border an obstacle:
  // - initialize their SDF value, and
  // - add them to the open queue
  //
  // NOTE: Traversal is truncated at tree_height_, so obstacle nodes finer
  //       than the SDF resolution are reported as pseudo-leaves holding
  //       their averaged occupancy, and all indices below are expressed in
  //       tree_height_-sized grid cells rather than true (height 0) leaves.
  const FloatingPoint cell_width = convert::heightToCellWidth(
      occupancy_map.getMinCellWidth(), tree_height_);
  occupancy_map.forEachLeaf(
      [this, &occupancy_query_accelerator, &sdf, &open_queue, cell_width](
          const OctreeIndex& node_index, FloatingPoint node_occupancy) {
        // Only process obstacles
        if (!classifier_.is(node_occupancy, Occupancy::kOccupied)) {
          return;
        }

        // Span a grid at the SDF resolution that pads the multi-resolution
        // obstacle cell with 1 voxel in all directions
        const Index3D min_corner = int_math::div_exp2_floor<3>(
            convert::nodeIndexToMinCornerIndex(node_index), tree_height_);
        const Index3D max_corner = int_math::div_exp2_floor<3>(
            convert::nodeIndexToMaxCornerIndex(node_index), tree_height_);
        const Grid<3> grid{
            Grid<3>(min_corner - Index3D::Ones(), max_corner + Index3D::Ones())};

        // Iterate over the grid
        for (const Index3D& index : grid) {
          // Skip cells that are inside the occupied node (obstacle)
          // NOTE: Occupied cells (negative distances) are handled in the
          //       propagation stage.
          const Index3D nearest_inner_index =
              index.cwiseMax(min_corner).cwiseMin(max_corner);
          const bool voxel_is_inside = (index == nearest_inner_index);
          if (voxel_is_inside) {
            continue;
          }

          // Skip the cell if it is not free
          const FloatingPoint occupancy = occupancy_query_accelerator.getCellValue(
              OctreeIndex{tree_height_, index});
          if (!classifier_.is(occupancy, Occupancy::kFree)) {
            continue;
          }

          // Get the voxel's SDF value
          FloatingPoint& sdf_value = sdf.getOrAllocateValue(index);
          const bool sdf_uninitialized = sdf.getDefaultValue() == sdf_value;

          // Update the voxel's SDF value
          const FloatingPoint distance_to_surface =
              0.5f * cell_width *
              (index - nearest_inner_index).cast<FloatingPoint>().norm();
          sdf_value = std::min(sdf_value, distance_to_surface);

          // If the voxel is not yet in the open queue, add it
          if (sdf_uninitialized) {
            open_queue.push(distance_to_surface, index);
          }
        }
      },
      tree_height_);
}

void QuasiEuclideanSDFGenerator::propagate(
    const HashedWaveletOctree& occupancy_map, HashedBlocks& sdf,
    BucketQueue<Index3D>& open_queue) const {
  ProfilerZoneScoped;
  // Create an occupancy query accelerator
  QueryAccelerator occupancy_query_accelerator{occupancy_map};

  // Precompute the neighbor distance offsets
  const FloatingPoint cell_width = convert::heightToCellWidth(
      occupancy_map.getMinCellWidth(), tree_height_);
  const auto neighbor_distance_offsets =
      GridNeighborhood<3>::computeOffsetLengths(kNeighborIndexOffsets,
                                                cell_width);
  const FloatingPoint half_max_neighbor_distance_offset =
      0.5f * std::sqrt(3.f) * cell_width + 1e-3f;

  // Propagate the distance
  while (!open_queue.empty()) {
    ProfilerPlot("QueueLength", static_cast<int64_t>(open_queue.size()));
    const Index3D index = open_queue.front();
    const FloatingPoint sdf_value = sdf.getCellValue(index);
    const FloatingPoint df_value = std::abs(sdf_value);
    ProfilerPlot("Distance", df_value);
    open_queue.pop();

    for (size_t neighbor_idx = 0; neighbor_idx < kNeighborIndexOffsets.size();
         ++neighbor_idx) {
      // Compute the neighbor's distance if reached from the current voxel
      FloatingPoint neighbor_df_candidate =
          df_value + neighbor_distance_offsets[neighbor_idx];
      if (max_distance_ <= neighbor_df_candidate) {
        continue;
      }

      // Get the neighbor's SDF value
      const Index3D neighbor_index =
          index + kNeighborIndexOffsets[neighbor_idx];
      FloatingPoint& neighbor_sdf = sdf.getOrAllocateValue(neighbor_index);

      // If the neighbor is uninitialized, get its sign from the occupancy map
      const bool neighbor_uninitialized = sdf.getDefaultValue() == neighbor_sdf;
      if (neighbor_uninitialized) {
        const FloatingPoint neighbor_occupancy =
            occupancy_query_accelerator.getCellValue(
                OctreeIndex{tree_height_, neighbor_index});
        // Never initialize or update unknown cells
        // if (classifier_.is(neighbor_occupancy, Occupancy::kUnobserved)) {
        //   continue;
        // }
        // Set the sign
        if (classifier_.is(neighbor_occupancy, Occupancy::kOccupied)) {
          neighbor_sdf = -sdf.getDefaultValue();
        }
      }

      // Handle sign changes when propagating across the surface
      const bool crossed_surface =
          std::signbit(neighbor_sdf) != std::signbit(sdf_value);
      if (crossed_surface) {
        if (neighbor_sdf < 0.f) {
          // NOTE: When the opened cell and the neighbor cell have the same
          //       sign, the distance field value and offset are summed to
          //       obtain the unsigned neighbor distance. Whereas when moving
          //       across the surface, the df_value and offset have opposite
          //       signs and reduce each other instead.
          // Skip if df_value would yield a non-positive candidate distance
          // (can happen when unobserved cells are propagated with large
          // distances before reaching an adjacent occupied cell).
          if (df_value >= neighbor_distance_offsets[neighbor_idx]) {
            continue;
          }
          neighbor_df_candidate =
              neighbor_distance_offsets[neighbor_idx] - df_value;
        } else {
          continue;
        }
      }

      // Update the neighbor's SDF value
      FloatingPoint neighbor_df = std::abs(neighbor_sdf);
      neighbor_df = std::min(neighbor_df, neighbor_df_candidate);
      neighbor_sdf = std::copysign(neighbor_df, neighbor_sdf);

      // If the neighbor is not yet in the open queue, add it
      if (neighbor_uninitialized) {
        open_queue.push(neighbor_df_candidate, neighbor_index);
      }
    }
  }
}
}  // namespace wavemap
