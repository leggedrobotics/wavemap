#include <memory>

#include <wavemap/core/common.h>
#include <wavemap/core/utils/edit/transform.h>
#include <wavemap/io/file_conversions.h>

int main(int, char**) {
  // Load the map
  const std::filesystem::path home_dir = CHECK_NOTNULL(getenv("HOME"));
  const std::filesystem::path input_map_path = home_dir / "your_map.wvmp";
  wavemap::MapBase::Ptr map_base;
  bool success = wavemap::io::fileToMap(input_map_path, map_base);
  CHECK(success);

  // Downcast it to a concrete map type
  auto map = std::dynamic_pointer_cast<wavemap::HashedWaveletOctree>(map_base);
  CHECK_NOTNULL(map);

  // Define a transformation that flips the map upside down, for illustration
  wavemap::Transformation3D T_AB;
  T_AB.getRotation() = wavemap::Rotation3D{0.f, 1.f, 0.f, 0.f};

  // Transform the map
  auto thread_pool = std::make_shared<wavemap::ThreadPool>();  // Optional
  map = wavemap::edit::transform(*map, T_AB, thread_pool);

  // Save the map
  const std::filesystem::path output_map_path =
      home_dir / "your_map_transformed.wvmp";
  success &= wavemap::io::mapToFile(*map, output_map_path);
  CHECK(success);
}
