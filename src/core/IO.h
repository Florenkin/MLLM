#pragma once
#include "core/Types.h"
namespace mllm
{
std::vector<Path> imageFiles(const Path& directory);
cv::Mat readImage(const Path& path, int mode = 1);
void writeImage(const Path& path, const cv::Mat& image);
std::string readText(const Path& path);
void writeText(const Path& path, const std::string& text);
Path uniqueDirectory(const Path& root, const std::string& prefix);
std::string calibrationYaml(const Calibration& calibration);
Calibration loadCalibration(const Path& path);
void saveCalibration(const Path& path, const Calibration& calibration);
} // namespace mllm
