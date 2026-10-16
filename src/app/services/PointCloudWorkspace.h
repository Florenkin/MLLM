#pragma once
#include "core/Types.h"

namespace mllm
{
struct CloudSnapshot
{
    std::size_t documentId = 0;
    std::shared_ptr<const Cloud> points;
    std::vector<std::size_t> selection;
    bool selectedOnly = false;
};
struct CloudDocument
{
    std::size_t id = 0, parentId = 0;
    std::string name;
    std::shared_ptr<const Cloud> points;
    std::vector<std::size_t> selection;
};
// 数据集只读，处理结果创建新条目；选择和处理参数按值快照。
class PointCloudWorkspace final
{
  public:
    std::size_t add(std::string name, std::shared_ptr<const Cloud> points, std::size_t parentId = 0);
    void activate(std::size_t id);
    void select(std::vector<std::size_t> indices);
    const CloudDocument* current() const;
    const std::vector<CloudDocument>& documents() const;
    CloudSnapshot snapshot(bool selectedOnly) const;

  private:
    std::vector<CloudDocument> documents_;
    std::size_t currentId_ = 0, nextId_ = 1;
};
} // namespace mllm
