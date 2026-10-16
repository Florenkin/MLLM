#pragma once
#include <atomic>
#include <cmath>
#include <filesystem>
#include <functional>
#include <memory>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace mllm
{
using Path = std::filesystem::path;
using Cloud = std::vector<cv::Point3d>;
struct Cancelled : std::runtime_error
{
    Cancelled() : std::runtime_error("任务已取消") {}
};
struct TaskContext
{
    std::shared_ptr<std::atomic_bool> cancel = std::make_shared<std::atomic_bool>(false);
    std::function<void(int, const std::string&)> progress;
    void check() const
    {
        if (cancel->load())
            throw Cancelled();
    }
    void report(int percent, const std::string& text) const
    {
        check();
        if (progress)
            progress(percent, text);
    }
};
struct BoardConfig
{
    cv::Size corners{11, 8};
    double squareMm = 15;
};
struct CameraCalibration
{
    cv::Mat K, D;
    cv::Size imageSize;
    double rms = 0;
    std::vector<double> errors;
    std::vector<std::string> files, rejected;
    bool valid() const;
};
struct LaserConfig
{
    int method = 0;          // 0: intensity centroid, 1: Steger Gaussian Hessian.
    int channel = 0;         // gray, red, green, blue.
    bool horizontal = false; // false: scan rows for vertical stripes.
    int threshold = 100;
    double width = 3;
    cv::Rect roi; // width/height zero means full remaining frame.
};
struct LaserLine
{
    std::vector<cv::Point2d> pixels;
    cv::Mat preview;
};
struct LaserPlane
{
    double position = 0;
    cv::Vec4d coefficients{0, 0, 0, 0}; // aX+bY+cZ+d=0; norm(a,b,c)=1.
    double rmsMm = 0, maxErrorMm = 0;
    int samples = 0, pairs = 0;
    bool valid() const;
};
struct Calibration
{
    CameraCalibration camera;
    std::vector<LaserPlane> planes;
};
struct ReconstructionConfig
{
    LaserConfig laser;
    int mode = 0; // fixed plane, linear translation, calibrated position table.
    double stepMm = 0;
    int axis = 1;
    double positionStart = 0, positionStep = 1;
    double minDepth = 1, maxDepth = 10000;
    bool saveDiagnostics = true;
};
struct FrameResult
{
    Cloud points;
    LaserLine line;
    std::size_t rejected = 0;
};
struct BatchResult
{
    Cloud points;
    Path directory;
    std::vector<Path> diagnostics;
    std::vector<std::string> warnings;
    int frames = 0, failed = 0;
};
struct PlaneFit
{
    cv::Vec4d plane;
    double rms = 0, maxError = 0;
    std::size_t inliers = 0;
};
} // namespace mllm
