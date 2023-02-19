#ifndef WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_DATA_STRUCTURE_BASE_H_
#define WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_DATA_STRUCTURE_BASE_H_

#include <algorithm>
#include <memory>
#include <string>

#include "wavemap/common.h"
#include "wavemap/indexing/ndtree_index.h"
#include "wavemap/utils/config_utils.h"

namespace wavemap {
struct VolumetricDataStructureConfig
    : ConfigBase<VolumetricDataStructureConfig> {
  FloatingPoint min_cell_width = 0.1f;

  // Constructors
  VolumetricDataStructureConfig() = default;
  VolumetricDataStructureConfig(FloatingPoint min_cell_width)  // NOLINT
      : min_cell_width(min_cell_width) {}

  bool isValid(bool verbose) const override;
  static VolumetricDataStructureConfig from(const param::Map& params);
};

class VolumetricDataStructureBase {
 public:
  static constexpr int kDim = 3;
  using Ptr = std::shared_ptr<VolumetricDataStructureBase>;

  explicit VolumetricDataStructureBase(
      const VolumetricDataStructureConfig& config)
      : config_(config.checkValid()) {}
  virtual ~VolumetricDataStructureBase() = default;

  virtual bool empty() const = 0;
  virtual size_t size() const = 0;
  virtual void prune() = 0;
  virtual void clear() = 0;

  virtual const VolumetricDataStructureConfig& getConfig() const {
    return config_;
  }
  FloatingPoint getMinCellWidth() const { return config_.min_cell_width; }
  virtual size_t getMemoryUsage() const = 0;

  virtual Index3D getMinIndex() const = 0;
  virtual Index3D getMaxIndex() const = 0;

  virtual FloatingPoint getCellValue(const Index3D& index) const = 0;
  virtual void setCellValue(const Index3D& index, FloatingPoint new_value) = 0;
  virtual void addToCellValue(const Index3D& index, FloatingPoint update) = 0;

  using IndexedLeafVisitorFunction =
      std::function<void(const OctreeIndex& index, FloatingPoint value)>;
  virtual void forEachLeaf(IndexedLeafVisitorFunction visitor_fn) const = 0;

  virtual bool save(const std::string& file_path_prefix) const = 0;
  virtual bool load(const std::string& file_path_prefix) = 0;

 protected:
  const VolumetricDataStructureConfig config_;
};
}  // namespace wavemap

#endif  // WAVEMAP_DATA_STRUCTURE_VOLUMETRIC_VOLUMETRIC_DATA_STRUCTURE_BASE_H_
