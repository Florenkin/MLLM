#pragma once
#include "core/Types.h"
namespace mllm
{
std::vector<cv::Point3f> boardPoints(const BoardConfig& board);
bool detectBoard(const cv::Mat& image, const BoardConfig& board, std::vector<cv::Point2f>& corners);
CameraCalibration calibrateCamera(const Path& directory, const BoardConfig& board,
                                  const TaskContext& task = {});
PlaneFit fitPlane(const Cloud& points, double inlierThresholdMm = 0.5);
LaserPlane calibrateLaserPlane(const Path& offDirectory, const Path& onDirectory,
                               const CameraCalibration& camera, const BoardConfig& board,
                               const LaserConfig& laser, double position, const TaskContext& task = {});
} // namespace mllm
