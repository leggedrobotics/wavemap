#include <memory>

#include <wavemap/core/common.h>
#include <wavemap/core/utils/edit/multiply.h>
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

  // Use the multiply method to implement exponential forgetting
  const wavemap::FloatingPoint decay_factor = 0.9;
  auto thread_pool = std::make_shared<wavemap::ThreadPool>();  // Optional
  wavemap::edit::multiply(*map, decay_factor, thread_pool);

  // Save the map
  const std::filesystem::path output_map_path =
      home_dir / "your_map_decayed.wvmp";
  success &= wavemap::io::mapToFile(*map, output_map_path);
  CHECK(success);
}