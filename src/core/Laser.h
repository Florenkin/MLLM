#pragma once
#include "core/Types.h"
namespace mllm
{
LaserLine extractLaser(const cv::Mat& image, const LaserConfig& config);
}
