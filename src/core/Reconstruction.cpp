#include "core/Reconstruction.h"
#include "core/CaptureManifest.h"
#include "core/IO.h"
#include "core/Laser.h"
#include "core/PointCloud.h"
#include <cmath>
#include <iomanip>
#include <opencv2/calib3d.hpp>
#include <sstream>

namespace mllm
{
const LaserPlane& planeAt(const Calibration& c, double position)
{
    for (const auto& p : c.planes)
        if (std::abs(p.position - position) < 1e-6 && p.valid())
            return p;
    throw std::runtime_error("缺少位置 " + std::to_string(position) + " 的已标定激光平面；不进行插值或外推");
}
FrameResult reconstructFrame(const cv::Mat& image, const CameraCalibration& camera, const LaserPlane& plane,
                             const ReconstructionConfig& c)
{
    if (!camera.valid() || !plane.valid())
        throw std::runtime_error("相机或激光平面标定无效");
    if (image.size() != camera.imageSize)
        throw std::runtime_error("图像尺寸与标定尺寸不一致；请使用原标定分辨率");
    if (!std::isfinite(c.minDepth) || !std::isfinite(c.maxDepth) || c.minDepth <= 0 ||
        c.maxDepth <= c.minDepth)
        throw std::runtime_error("深度范围无效");
    FrameResult result;
    result.line = extractLaser(image, c.laser);
    if (result.line.pixels.empty())
        return result;
    std::vector<cv::Point2d> normalized;
    cv::undistortPoints(result.line.pixels, normalized, camera.K, camera.D);
    for (const auto& p : normalized)
    {
        const auto& planeCoeffs = plane.coefficients;
        const double denominator = planeCoeffs[0] * p.x + planeCoeffs[1] * p.y + planeCoeffs[2];
        if (std::abs(denominator) < 1e-9)
        {
            ++result.rejected;
            continue;
        }
        const double z = -planeCoeffs[3] / denominator;
        if (!std::isfinite(z) || z < c.minDepth || z > c.maxDepth)
        {
            ++result.rejected;
            continue;
        }
        result.points.emplace_back(p.x * z, p.y * z, z);
    }
    return result;
}
BatchResult reconstructBatch(const Path& input, const Path& output, const Calibration& calibration,
                             const ReconstructionConfig& c, const TaskContext& task)
{
    task.check();
    if (!calibration.camera.valid() || calibration.planes.empty())
        throw std::runtime_error("请先完成相机和激光平面两项标定");
    if (c.mode < 0 || c.mode > 2 || c.axis < 0 || c.axis > 2 || !std::isfinite(c.stepMm) ||
        !std::isfinite(c.positionStart) || !std::isfinite(c.positionStep))
        throw std::runtime_error("扫描参数无效");
    BatchResult result;
    const auto plans = reconstructionFrames(input, c, result.warnings, task);
    // Validate every calibrated slot before creating any reconstruction output.
    for (const auto& plan : plans)
        planeAt(calibration, plan.planePosition);
    result.directory = uniqueDirectory(output, "reconstruction");
    saveCalibration(result.directory / "calibration.yml", calibration);
    std::ostringstream report;
    report << "index,file,position,translation_mm,line_points,cloud_points,rejected,status\n";
    for (std::size_t i = 0; i < plans.size(); ++i)
    {
        const auto& plan = plans[i];
        const auto& sourceFile = plan.file;
        task.report(static_cast<int>(90 * i / plans.size()), "重建：" + sourceFile.filename().u8string());
        const double position = plan.planePosition;
        report << plan.sourceIndex << ",\"" << sourceFile.filename().u8string() << "\"," << position << ","
               << plan.translationMm << ",";
        FrameResult frame;
        try
        {
            frame = reconstructFrame(readImage(sourceFile), calibration.camera,
                                     planeAt(calibration, position), c);
            if (frame.points.empty())
                throw std::runtime_error("未获得有效交点，请检查颜色、阈值、ROI 和深度范围");
            if (c.mode == 1)
                for (auto& p : frame.points)
                {
                    const double shift = plan.translationMm;
                    if (c.axis == 0)
                        p.x += shift;
                    else if (c.axis == 1)
                        p.y += shift;
                    else
                        p.z += shift;
                }
        }
        catch (const Cancelled&)
        {
            throw;
        }
        catch (const std::exception& e)
        {
            ++result.failed;
            result.warnings.push_back(sourceFile.filename().u8string() + ": " + e.what());
            report << "0,0,0,failed\n";
            continue;
        }
        // Storage failures must not be misreported as rejected image inputs.
        if (c.saveDiagnostics)
        {
            const Path diagnostic = result.directory / "laser_lines" /
                                    (std::to_string(i) + "_" + sourceFile.stem().u8string() + ".png");
            writeImage(diagnostic, frame.line.preview);
            result.diagnostics.push_back(diagnostic);
        }
        result.points.insert(result.points.end(), frame.points.begin(), frame.points.end());
        ++result.frames;
        report << frame.line.pixels.size() << "," << frame.points.size() << "," << frame.rejected << ",ok\n";
    }
    task.check();
    writeText(result.directory / "frames.csv", report.str());
    if (result.points.empty())
        throw std::runtime_error("全部图像重建失败，报告位于：" + result.directory.u8string());
    task.report(95, "保存点云与处理参数");
    auto saveTask = task;
    saveTask.progress = {};
    exportCloud(result.directory / "cloud.xyz", result.points, saveTask);
    exportCloud(result.directory / "cloud.pcd", result.points, saveTask);
    exportCloud(result.directory / "cloud.ply", result.points, saveTask);
    const auto manifest = findCaptureManifest(input);
    if (!manifest.empty())
        writeText(result.directory / "capture_manifest.yml", readText(manifest));
    writeText(result.directory / "parameters.txt", reconstructionParameters(c));
    task.report(100, "重建完成");
    task.check();
    writeText(result.directory / "completed.txt",
              "Reconstruction completed. See frames.csv for skipped inputs.\n");
    return result;
}
std::string reconstructionParameters(const ReconstructionConfig& c)
{
    std::ostringstream settings;
    settings << std::setprecision(17) << "units=mm\nmedium=air\nmode=" << c.mode << "\nstep_mm=" << c.stepMm
             << "\naxis=" << c.axis << "\nposition_start=" << c.positionStart
             << "\nposition_step=" << c.positionStep << "\nmin_depth=" << c.minDepth
             << "\nmax_depth=" << c.maxDepth << "\nmethod=" << c.laser.method
             << "\nchannel=" << c.laser.channel << "\nhorizontal=" << c.laser.horizontal
             << "\nthreshold=" << c.laser.threshold << "\nstripe_width=" << c.laser.width
             << "\nroi=" << c.laser.roi << "\n";
    return settings.str();
}
} // namespace mllm
