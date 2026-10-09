#pragma once
#include "core/Types.h"
namespace mllm
{
std::string cloudText(const std::string& extension, const Cloud& cloud);
void exportCloud(const Path& path, const Cloud& cloud);
Cloud importCloud(const Path& path);
Cloud voxelDownsample(const Cloud& cloud, double millimetres);
std::pair<cv::Point3d, cv::Point3d> cloudBounds(const Cloud& cloud);
} // namespace mllm
