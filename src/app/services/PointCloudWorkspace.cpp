#include "app/services/PointCloudWorkspace.h"
#include <algorithm>

namespace mllm
{
std::size_t PointCloudWorkspace::add(std::string name, std::shared_ptr<const Cloud> points,
                                     std::size_t parentId)
{
    if (!points || points->empty())
        throw std::runtime_error("点云为空");
    if (parentId && std::none_of(documents_.begin(), documents_.end(),
                                 [parentId](const auto& item) { return item.id == parentId; }))
        throw std::runtime_error("点云来源已失效");
    const auto id = nextId_++;
    documents_.push_back({id, parentId, std::move(name), std::move(points), {}});
    currentId_ = id;
    return id;
}
void PointCloudWorkspace::activate(std::size_t id)
{
    if (std::none_of(documents_.begin(), documents_.end(), [id](const auto& item) { return item.id == id; }))
        throw std::runtime_error("点云条目不存在");
    currentId_ = id;
}
void PointCloudWorkspace::select(std::vector<std::size_t> indices)
{
    auto found = std::find_if(documents_.begin(), documents_.end(),
                              [this](const auto& item) { return item.id == currentId_; });
    if (found == documents_.end())
        return;
    std::sort(indices.begin(), indices.end());
    indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
    if (!indices.empty() && indices.back() >= found->points->size())
        throw std::runtime_error("点云选区索引超出范围");
    found->selection = std::move(indices);
}
const CloudDocument* PointCloudWorkspace::current() const
{
    const auto found = std::find_if(documents_.begin(), documents_.end(),
                                    [this](const auto& item) { return item.id == currentId_; });
    return found == documents_.end() ? nullptr : &*found;
}
const std::vector<CloudDocument>& PointCloudWorkspace::documents() const
{
    return documents_;
}
CloudSnapshot PointCloudWorkspace::snapshot(bool selectedOnly) const
{
    const auto* item = current();
    if (!item)
        throw std::runtime_error("请先加载或重建点云");
    if (selectedOnly && item->selection.empty())
        throw std::runtime_error("请先框选点云");
    return {item->id, item->points, item->selection, selectedOnly};
}
} // namespace mllm
