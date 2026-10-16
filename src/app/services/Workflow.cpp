#include "app/services/Workflow.h"
#include "core/Calibration.h"
#include "core/CaptureManifest.h"
#include "core/IO.h"
#include "core/Logger.h"
#include "core/PointCloud.h"
#include "core/Reconstruction.h"
#include <algorithm>
#include <sstream>
namespace mllm
{
namespace
{
TaskContext loggedTask(const TaskContext& task, const char* module)
{
    auto result = task;
    result.progress =
        [module, progress = task.progress, last = -10](int percent, const std::string& text) mutable
    {
        if (percent >= last + 10 || percent == 100)
        {
            Logger::instance().info(module, "进度 " + std::to_string(percent) + "%：" + text);
            last = percent;
        }
        if (progress)
            progress(percent, text);
    };
    return result;
}
} // namespace
const char* jobModule(JobKind kind)
{
    switch (kind)
    {
    case JobKind::CameraCalibration:
        return "CameraCalibration";
    case JobKind::LaserCalibration:
        return "LaserCalibration";
    case JobKind::Reconstruction:
        return "Reconstruction";
    default:
        return "PointCloud";
    }
}
JobResult executeJob(JobKind kind, const TaskContext& task, const JobWork& work)
{
    const auto module = jobModule(kind);
    JobResult result;
    result.kind = kind;
    try
    {
        const auto currentTask = loggedTask(task, module);
        currentTask.check();
        work(result, currentTask);
        result.status = JobStatus::Succeeded;
        if (kind == JobKind::CameraCalibration)
            for (const auto& rejected : result.calibration.camera.rejected)
                Logger::instance().warning(module, rejected);
        if (result.batch)
            for (const auto& warning : result.batch->warnings)
                Logger::instance().warning(module, warning);
        Logger::instance().info(module, result.message);
        if (!result.directory.empty())
            Logger::instance().info(module, "结果目录：" + result.directory.u8string());
    }
    catch (const Cancelled& e)
    {
        result.status = JobStatus::Cancelled;
        result.message = e.what();
        Logger::instance().warning(module, result.message);
    }
    catch (const std::exception& e)
    {
        result.message = e.what();
        Logger::instance().error(module, result.message);
    }
    catch (...)
    {
        result.message = "任务发生未知异常";
        Logger::instance().error(module, result.message);
    }
    return result;
}
JobResult cameraWorkflow(const Path& images, const Path& output, const BoardConfig& board,
                         const TaskContext& task)
{
    Logger::instance().info("CameraCalibration", "开始相机标定，图像目录：" + images.u8string());
    return executeJob(JobKind::CameraCalibration, task,
                      [&](JobResult& r, const TaskContext& currentTask)
                      {
                          r.calibration.camera = calibrateCamera(images, board, currentTask);
                          currentTask.check();
                          r.directory = uniqueDirectory(output, "camera_calibration");
                          saveCalibration(r.directory / "calibration.yml", r.calibration);
                          std::ostringstream report;
                          report << "file,rms_px\n";
                          for (std::size_t i = 0; i < r.calibration.camera.files.size(); ++i)
                              report << '"' << r.calibration.camera.files[i] << "\","
                                     << r.calibration.camera.errors[i] << "\n";
                          writeText(r.directory / "errors.csv", report.str());
                          std::ostringstream rejected;
                          for (const auto& message : r.calibration.camera.rejected)
                              rejected << message << "\n";
                          writeText(r.directory / "rejected.txt", rejected.str());
                          r.message =
                              "相机标定完成，RMS = " + std::to_string(r.calibration.camera.rms) + " px";
                      });
}
JobResult laserWorkflow(const Path& off, const Path& on, const Path& output, Calibration calibration,
                        const BoardConfig& board, const LaserConfig& laser, double position,
                        const TaskContext& task)
{
    Logger::instance().info("LaserCalibration", "开始激光平面标定，位置 " + std::to_string(position) +
                                                    "，关闭目录：" + off.u8string() + "，开启目录：" +
                                                    on.u8string());
    return executeJob(
        JobKind::LaserCalibration, task,
        [&](JobResult& r, const TaskContext& currentTask)
        {
            const auto plane =
                calibrateLaserPlane(off, on, calibration.camera, board, laser, position, currentTask);
            calibration.planes.erase(std::remove_if(calibration.planes.begin(), calibration.planes.end(),
                                                    [&](const LaserPlane& p)
                                                    { return std::abs(p.position - position) < 1e-6; }),
                                     calibration.planes.end());
            calibration.planes.push_back(plane);
            std::sort(calibration.planes.begin(), calibration.planes.end(),
                      [](const LaserPlane& a, const LaserPlane& b) { return a.position < b.position; });
            currentTask.check();
            r.calibration = calibration;
            r.directory = uniqueDirectory(output, "laser_calibration");
            saveCalibration(r.directory / "calibration.yml", r.calibration);
            r.message = "激光面标定完成，RMS = " + std::to_string(plane.rmsMm) + " mm，有效图像对 " +
                        std::to_string(plane.pairs);
        });
}
JobResult reconstructionWorkflow(const Path& images, const Path& output, const Calibration& calibration,
                                 const ReconstructionConfig& config, const TaskContext& task)
{
    Logger::instance().info("Reconstruction", "开始批量重建，图像目录：" + images.u8string());
    Logger::instance().debug("Reconstruction", reconstructionParameters(config));
    return executeJob(JobKind::Reconstruction, task,
                      [&](JobResult& r, const TaskContext& currentTask)
                      {
                          r.batch = std::make_shared<BatchResult>(
                              reconstructBatch(images, output, calibration, config, currentTask));
                          r.directory = r.batch->directory;
                          r.message = "重建完成：" + std::to_string(r.batch->frames) + " 帧，" +
                                      std::to_string(r.batch->points.size()) + " 点；失败 " +
                                      std::to_string(r.batch->failed) + " 帧";
                      });
}
JobResult singleFrameWorkflow(const Path& image, const Path& output, const Calibration& calibration,
                              const ReconstructionConfig& config, const TaskContext& task)
{
    Logger::instance().info("Reconstruction", "开始单帧重建，图像：" + image.u8string());
    return executeJob(
        JobKind::Reconstruction, task,
        [&](JobResult& result, const TaskContext& currentTask)
        {
            currentTask.check();
            FramePlan plan{image, 0, config.positionStart, 0};
            const auto manifest = findCaptureManifest(image.parent_path());
            std::vector<std::string> warnings;
            if (!manifest.empty())
            {
                const auto plans = reconstructionFrames(image.parent_path(), config, warnings, currentTask);
                const auto found = std::find_if(plans.begin(), plans.end(), [&](const FramePlan& item)
                                                { return item.file.filename() == image.filename(); });
                if (found == plans.end())
                    throw std::runtime_error("单帧图像未登记在采集记录中");
                plan = *found;
            }
            auto frame = reconstructFrame(readImage(image), calibration.camera,
                                          planeAt(calibration, plan.planePosition), config);
            if (!manifest.empty() && config.mode == 1)
                for (auto& point : frame.points)
                {
                    if (config.axis == 0)
                        point.x += plan.translationMm;
                    else if (config.axis == 1)
                        point.y += plan.translationMm;
                    else
                        point.z += plan.translationMm;
                }
            if (frame.points.empty())
                throw std::runtime_error("未获得有效激光交点");
            currentTask.check();
            result.directory = uniqueDirectory(output, "single_frame");
            auto batch = std::make_shared<BatchResult>();
            batch->directory = result.directory;
            batch->points = frame.points;
            batch->frames = 1;
            batch->warnings = std::move(warnings);
            result.batch = batch;
            saveCalibration(result.directory / "calibration.yml", calibration);
            exportCloud(result.directory / "cloud.xyz", frame.points, currentTask);
            exportCloud(result.directory / "cloud.pcd", frame.points, currentTask);
            exportCloud(result.directory / "cloud.ply", frame.points, currentTask);
            writeText(result.directory / "parameters.txt",
                      reconstructionParameters(config) + "source=" + image.u8string() +
                          "\nframe_plane_position=" + std::to_string(plan.planePosition) +
                          "\nframe_translation_mm=" + std::to_string(plan.translationMm) + "\n");
            if (!manifest.empty())
                writeText(result.directory / "capture_manifest.yml", readText(manifest));
            if (config.saveDiagnostics)
            {
                const auto diagnostic = result.directory / "laser_lines/line.png";
                writeImage(diagnostic, frame.line.preview);
                batch->diagnostics.push_back(diagnostic);
            }
            currentTask.check();
            writeText(result.directory / "completed.txt", "Single frame reconstruction completed.\n");
            result.message = "单帧重建完成：" + std::to_string(frame.points.size()) + " 点";
        });
}
} // namespace mllm
