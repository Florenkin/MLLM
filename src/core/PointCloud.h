#pragma once
#include "core/Types.h"
namespace mllm
{
struct CloudProjection
{
    cv::Point3d center;
    cv::Vec3d horizontal{1, 0, 0}, vertical{0, 1, 0};
    double scale = 1;
    cv::Point2d offset;
};
cv::Point2d projectCloudPoint(const cv::Point3d& point, const CloudProjection& projection);
std::vector<std::size_t> selectProjectedCloud(const Cloud& cloud, const CloudProjection& projection,
                                              const cv::Rect2d& rectangle, const TaskContext& task = {});
std::string cloudText(const std::string& extension, const Cloud& cloud, const TaskContext& task = {});
void exportCloud(const Path& path, const Cloud& cloud, const TaskContext& task = {});
Cloud importCloud(const Path& path, const TaskContext& task = {});
Cloud voxelDownsample(const Cloud& cloud, double millimetres, const TaskContext& task = {});
Cloud selectCloud(const Cloud& cloud, const std::vector<std::size_t>& indices, const TaskContext& task = {});
Cloud cropCloud(const Cloud& cloud, int axis, double minimum, double maximum, const TaskContext& task = {});
std::pair<cv::Point3d, cv::Point3d> cloudBounds(const Cloud& cloud);
} // namespace mllm
