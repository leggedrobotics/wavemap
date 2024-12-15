#include <filesystem>
#include <fstream>
#include <vector>

#include <glog/logging.h>
#include <wavemap/core/common.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/io/file_conversions.h>
#include <wavemap/core/utils/query/query_accelerator.h>
#include <wavemap/core/utils/neighbors/grid_neighborhood.h>

using namespace wavemap;  // NOLINT


static const auto kNeighborOffsets =
    GridNeighborhood<3>::generateIndexOffsets<Adjacency::kAnyDisjoint>();


class MapProcessor {
private:
  mutable std::optional<QueryAccelerator<HashedWaveletOctree>>
      query_accelerator_;

  FloatingPoint surface_occupancy_threshold_ = 0.0f;  // You might want to set an appropriate value

  FloatingPoint unknown_occupancy_threshold_ = 1e-4f;
public:
    bool isUnknown(FloatingPoint log_odds) const {
        unknown_occupancy_threshold_;
    }
  bool hasFreeNeighbor(const OctreeIndex& cell_index) const {
    for (const auto& offset : kNeighborOffsets) {  // NOLINT
      const OctreeIndex neighbor_index = {cell_index.height,
                                          cell_index.position + offset};
      const FloatingPoint neighbor_log_odds =
          query_accelerator_->getCellValue(neighbor_index);
      
      // Check if the neighbor is free and observed
      if (neighbor_log_odds < surface_occupancy_threshold_ &&
          !isUnknown(neighbor_log_odds)) {
        return true;
      }
    }
    return false;
  }

  std::vector<Point3D> processMap(const std::filesystem::path& map_file_path) {
    // Initialize GLOG
    google::InitGoogleLogging("MapProcessor");
    google::InstallFailureSignalHandler();
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;

    // Load the occupancy map
    MapBase::Ptr occupancy_map;
    if (!io::fileToMap(map_file_path, occupancy_map)) {
      LOG(WARNING) << "Could not open map for reading: " << map_file_path;
      return {};
    }
    const auto hashed_map =
      std::dynamic_pointer_cast<const HashedWaveletOctree>(occupancy_map);
    query_accelerator_.emplace(*hashed_map);

    // Convert each occupied leaf node to high-res points and add to pointcloud
    std::vector<Point3D> pointcloud;
    constexpr FloatingPoint kOccupancyThreshold = 0.01f;
    const FloatingPoint min_cell_width = occupancy_map->getMinCellWidth();
    
    occupancy_map->forEachLeaf([this, min_cell_width, &pointcloud, kOccupancyThreshold](
                                 const OctreeIndex& node_index,
                                 FloatingPoint node_log_odds) {
      if (kOccupancyThreshold < node_log_odds) {
        if (node_index.height == 0) {
          const Point3D center =
              convert::indexToCenterPoint(node_index.position, min_cell_width);
          if (this->hasFreeNeighbor(node_index)) {
            pointcloud.emplace_back(center);
          }
        } else {
          const Index3D node_min_corner =
              convert::nodeIndexToMinCornerIndex(node_index);
          const Index3D node_max_corner =
              convert::nodeIndexToMaxCornerIndex(node_index);
          for (const Index3D& index : Grid(node_min_corner, node_max_corner)) {
            const Point3D center =
                convert::indexToCenterPoint(index, min_cell_width);
            if (this->hasFreeNeighbor(node_index)) {
              pointcloud.emplace_back(center);
            }
          }
        }
      }
    });

    return pointcloud;
  }

  bool writePlyFile(const std::vector<Point3D>& pointcloud, 
                    const std::filesystem::path& map_file_path,
                    FloatingPoint min_cell_width) {
    // Create the PLY output file
    const std::filesystem::path ply_file_path =
        std::filesystem::path(map_file_path).replace_extension("_surface.ply");
    LOG(INFO) << "Creating PLY file: " << ply_file_path;
    std::ofstream ply_file(ply_file_path,
                           std::ofstream::out | std::ofstream::binary);
    if (!ply_file.is_open()) {
      LOG(WARNING) << "Could not open file for writing. Error: "
                   << strerror(errno);
      return false;
    }

    // Write the PLY header
    ply_file << "ply\n"
                "format ascii 1.0\n"
                "comment The voxel size is " << min_cell_width << " meters.\n"
                "element vertex " << pointcloud.size() << "\n"
                "property float x\n"
                "property float y\n"
                "property float z\n"
                "end_header\n";

    // Add the points
    for (const Point3D& point : pointcloud) {
      ply_file << point.x() << " " << point.y() << " " << point.z() << "\n";
    }
    ply_file.flush();

    // Close the file and communicate whether writing succeeded
    ply_file.close();
    return static_cast<bool>(ply_file);
  }
};

int main(int argc, char** argv) {
  // Check command line arguments
  if (argc != 2) {
    LOG(ERROR)
        << "Please supply a path to an occupancy map as the first argument.";
    return EXIT_FAILURE;
  }

  MapProcessor processor;
  const std::filesystem::path map_file_path = argv[1];

  // Process the map and get pointcloud
  std::vector<Point3D> pointcloud = processor.processMap(map_file_path);

  // Load the map again to get min cell width (since processMap might have modified the map)
  MapBase::Ptr occupancy_map;
  if (!io::fileToMap(map_file_path, occupancy_map)) {
    LOG(WARNING) << "Could not open map for reading: " << map_file_path;
    return EXIT_FAILURE;
  }
  const FloatingPoint min_cell_width = occupancy_map->getMinCellWidth();

  // Write pointcloud to PLY file
  if (!processor.writePlyFile(pointcloud, map_file_path, min_cell_width)) {
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}