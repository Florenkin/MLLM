#include "app/acquisition/Camera.h"
#include "app/services/Workflow.h"
#include "core/Calibration.h"
#include "core/IO.h"
#include "core/Laser.h"
#include "core/PointCloud.h"
#include "core/Reconstruction.h"
#include <QCoreApplication>
#include <QEventLoop>
#include <QTemporaryDir>
#include <QTimer>
#include <cmath>
#include <functional>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

using namespace mllm;
namespace
{
int checks = 0;
void require(bool condition, const char* message)
{
    ++checks;
    if (!condition)
        throw std::runtime_error(message);
}
void near(double a, double b, double tolerance, const char* message)
{
    require(std::isfinite(a) && std::abs(a - b) < tolerance, message);
}
void fails(const std::function<void()>& f, const char* message)
{
    bool caught = false;
    try
    {
        f();
    }
    catch (const std::exception&)
    {
        caught = true;
    }
    require(caught, message);
}
CameraCalibration camera()
{
    CameraCalibration c;
    c.K = (cv::Mat_<double>(3, 3) << 800, 0, 320, 0, 800, 240, 0, 0, 1);
    c.D = cv::Mat::zeros(1, 5, CV_64F);
    c.imageSize = {640, 480};
    return c;
}
cv::Mat stripe(int w, int h, double center, bool horizontal = false)
{
    cv::Mat image(h, w, CV_8UC1);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            image.at<uchar>(y, x) =
                static_cast<uchar>(8 + 235 * std::exp(-std::pow((horizontal ? y : x) - center, 2) / 8));
    return image;
}
cv::Mat renderBoard(const BoardConfig& board, const CameraCalibration& cam, const cv::Vec3d& rvec,
                    const cv::Vec3d& tvec, bool laserOn)
{
    const int scale = 3;
    cv::Mat image(cam.imageSize.height * scale, cam.imageSize.width * scale, CV_8UC1, cv::Scalar(110));
    cv::Mat K = cam.K.clone();
    K.row(0) *= scale;
    K.row(1) *= scale;
    for (int y = -1; y < board.corners.height; ++y)
        for (int x = -1; x < board.corners.width; ++x)
        {
            const float s = static_cast<float>(board.squareMm);
            std::vector<cv::Point3f> square{{x * s, y * s, 0},
                                            {(x + 1) * s, y * s, 0},
                                            {(x + 1) * s, (y + 1) * s, 0},
                                            {x * s, (y + 1) * s, 0}};
            std::vector<cv::Point2f> projected;
            cv::projectPoints(square, rvec, tvec, K, cam.D, projected);
            std::vector<cv::Point> polygon;
            for (const auto& p : projected)
                polygon.emplace_back(cvRound(p.x), cvRound(p.y));
            cv::fillConvexPoly(image, polygon, cv::Scalar(((x + y) % 2 == 0) ? 225 : 20), cv::LINE_AA);
        }
    if (laserOn)
    {
        cv::Mat R;
        cv::Rodrigues(rvec, R);
        const cv::Vec3d n(1, 0, -0.2);
        const cv::Vec3d u(R.at<double>(0, 0), R.at<double>(1, 0), R.at<double>(2, 0)),
            v(R.at<double>(0, 1), R.at<double>(1, 1), R.at<double>(2, 1));
        std::vector<cv::Point3f> ends;
        for (double y : {-board.squareMm, board.corners.height * board.squareMm})
        {
            const double x = -(n.dot(v) * y + n.dot(tvec) + 100) / n.dot(u);
            ends.emplace_back(static_cast<float>(x), static_cast<float>(y), 0);
        }
        std::vector<cv::Point2f> projected;
        cv::projectPoints(ends, rvec, tvec, K, cam.D, projected);
        cv::Mat glow = cv::Mat::zeros(image.size(), CV_8UC1);
        cv::line(glow, projected[0], projected[1], cv::Scalar(255), 6, cv::LINE_AA);
        cv::GaussianBlur(glow, glow, {9, 9}, 1.2);
        cv::add(image, glow, image);
    }
    cv::GaussianBlur(image, image, {3, 3}, 0.5);
    cv::resize(image, image, cam.imageSize, 0, 0, cv::INTER_AREA);
    return image;
}
void makeData(const Path& root)
{
    const auto cam = camera();
    const BoardConfig b{{9, 6}, 20};
    for (int i = 0; i < 18; ++i)
    {
        const cv::Vec3d r(0.07 * (i % 5 - 2), 0.08 * (i % 4 - 1), 0.03 * (i % 3 - 1));
        const cv::Vec3d t(-85 + (i % 3 - 1) * 10, -55 + (i % 4 - 1) * 8, 500 + i * 12);
        const auto name = std::to_string(i) + ".png";
        const auto off = renderBoard(b, cam, r, t, false);
        writeImage(root / "camera" / name, off);
        writeImage(root / "off" / name, off);
        writeImage(root / "on" / name, renderBoard(b, cam, r, t, true));
    }
    Calibration c;
    c.camera = cam;
    LaserPlane p;
    p.coefficients = {1, 0, -0.2, 100};
    p.coefficients /= std::sqrt(1.04);
    c.planes.push_back(p);
    saveCalibration(root / "calibration.yml", c);
    for (int i = 0; i < 8; ++i)
        writeImage(root / "scan" / (std::to_string(i) + ".png"), stripe(640, 480, 325 + i * 5));
    writeText(root / "README.txt",
              "Synthetic demo only. Air, mm, board 9x6 inner corners, 20 mm square. Camera K=[800 0 320;0 "
              "800 240;0 0 1]. Laser plane X-0.2Z+100=0. Use grayscale threshold 60, vertical stripe, fixed "
              "plane at position 0. Never use this calibration for real equipment.\n");
}
} // namespace
int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    try
    {
        if (argc == 3 && std::string(argv[1]) == "--generate-demo")
        {
            makeData(std::filesystem::u8path(argv[2]));
            std::cout << "Demo generated\n";
            return 0;
        }
        QTemporaryDir temporary;
        require(temporary.isValid(), "temporary directory");
        const Path root = Path(temporary.path().toStdWString()) / L"单目测试";
        const auto cam = camera();
        require(cam.valid(), "valid camera");
        LaserConfig l;
        l.threshold = 60;
        for (int method = 0; method < 2; ++method)
        {
            l.method = method;
            auto result = extractLaser(stripe(256, 120, 121.35), l);
            require(result.pixels.size() == 120, "vertical line count");
            for (const auto& p : result.pixels)
                near(p.x, 121.35, 0.12, "subpixel stripe accuracy");
            l.horizontal = true;
            result = extractLaser(stripe(140, 240, 100.2, true), l);
            require(result.pixels.size() == 140, "horizontal line count");
            for (const auto& p : result.pixels)
                near(p.y, 100.2, 0.12, "horizontal subpixel accuracy");
            l.horizontal = false;
        }
        l.method = 0;
        for (int method = 0; method < 2; ++method)
        {
            auto uniformConfig = l;
            uniformConfig.method = method;
            require(extractLaser(cv::Mat(100, 100, CV_8UC1, cv::Scalar(220)), uniformConfig).pixels.empty(),
                    "uniform bright background must not reconstruct a false stripe");
        }
        l.roi = {100, 10, 40, 80};
        const auto roi = extractLaser(stripe(256, 120, 121.35), l);
        require(roi.pixels.size() == 80, "ROI count");
        near(roi.pixels.front().y, 10, 0.01, "ROI full image offset");
        l.roi = {};
        LaserConfig invalid = l;
        invalid.roi = {255, 0, 3, 20};
        fails([&] { extractLaser(stripe(256, 120, 121), invalid); }, "invalid ROI rejected");
        cv::Mat doubleLine(30, 100, CV_8UC1, cv::Scalar(0));
        doubleLine.colRange(10, 13).setTo(255);
        doubleLine.colRange(70, 73).setTo(150);
        const auto selected = extractLaser(doubleLine, l);
        near(selected.pixels[0].x, 11, 0.01, "reflections do not drag centroid into gap");
        LaserPlane plane;
        plane.coefficients = {0, 0, 1, -500};
        ReconstructionConfig c;
        c.laser = l;
        auto frame = reconstructFrame(stripe(640, 480, 320), cam, plane, c);
        require(frame.points.size() == 480, "reconstruction count");
        near(frame.points[10].z, 500, 1e-8, "metric depth");
        near(frame.points[10].x, 0, 0.1, "camera ray x");
        plane.coefficients = {0, 0, 1, 500};
        require(reconstructFrame(stripe(640, 480, 320), cam, plane, c).points.empty(),
                "negative depth rejected");
        plane.coefficients = {1, 0, 0, -100};
        require(reconstructFrame(stripe(640, 480, 320), cam, plane, c).points.empty(),
                "parallel ray rejected");
        fails([&] { reconstructFrame(stripe(320, 240, 160), cam, plane, c); },
              "mismatched image size rejected");
        Cloud points;
        for (int y = 0; y < 11; ++y)
            for (int x = 0; x < 11; ++x)
                points.emplace_back(x * 10, y * 10, 500);
        for (int i = 0; i < 20; ++i)
            points.emplace_back(i * 4, i * 7, 700 + i);
        const auto fit = fitPlane(points);
        require(fit.inliers == 121, "RANSAC outlier rejection");
        near(fit.rms, 0, 1e-8, "plane fit residual");
        fails([&] { fitPlane({{0, 0, 0}, {1, 1, 1}, {2, 2, 2}}); }, "collinear plane rejected");
        for (const auto& ext : {".xyz", ".pcd", ".ply"})
        {
            const auto file = root / (std::string("点云") + ext);
            exportCloud(file, points);
            const auto loaded = importCloud(file);
            require(loaded.size() == points.size(), "cloud roundtrip count");
            near(loaded[50].z, 500, 1e-9, "cloud roundtrip coordinates");
            fails([&] { exportCloud(file, points); }, "output is not overwritten");
        }
        const auto down = voxelDownsample({{0.1, 0, 0}, {0.2, 0, 0}, {1.2, 0, 0}}, 1);
        require(down.size() == 2, "voxel count");
        near(down[0].x, 0.15, 1e-9, "voxel centroid");
        makeData(root / "data");
        const auto calibration = loadCalibration(root / "data/calibration.yml");
        require(calibration.camera.valid() && calibration.planes.size() == 1, "calibration YAML roundtrip");
        fails([&] { planeAt(calibration, 10); }, "missing position rejected");
        const BoardConfig board{{9, 6}, 20};
        const auto calibrated = calibrateCamera(root / "data/camera", board);
        require(calibrated.files.size() == 18, "all synthetic boards detected");
        require(calibrated.rms < 0.7, "camera reprojection accuracy");
        near(calibrated.K.at<double>(0, 0), 800, 80, "camera focal estimate");
        l.threshold = 60;
        const auto laser = calibrateLaserPlane(root / "data/off", root / "data/on", cam, board, l, 0);
        require(laser.pairs >= 3 && laser.samples > 100, "laser paired calibration samples");
        require(laser.rmsMm < 0.5, "laser plane fit residual");
        near(std::abs(laser.coefficients[0]), 1 / std::sqrt(1.04), 0.01, "laser plane normal recovery");
        near(std::abs(laser.coefficients[3]), 100 / std::sqrt(1.04), 2, "laser plane offset recovery");
        c.laser = l;
        const auto batch =
            reconstructionWorkflow(root / "data/scan", root / "results", calibration, c, TaskContext{});
        require(batch.success && batch.batch.frames == 8 && batch.batch.points.size() > 1000,
                "batch workflow");
        require(std::filesystem::exists(batch.directory / "cloud.pcd"), "batch exports");
        require(batch.batch.diagnostics.size() == 8, "diagnostic images");
        const auto single =
            singleFrameWorkflow(root / "data/scan/0.png", root / "results", calibration, c, TaskContext{});
        require(single.success && single.batch.frames == 1, "single frame workflow");
        require(std::filesystem::exists(single.directory / "parameters.txt"),
                "single frame parameter snapshot");
        auto indexed = calibration;
        for (int i = 1; i < 8; ++i)
        {
            auto slot = indexed.planes.front();
            slot.position = i;
            slot.coefficients[3] *= 1 + i * 0.01;
            indexed.planes.push_back(slot);
        }
        c.mode = 2;
        const auto tableBatch =
            reconstructionWorkflow(root / "data/scan", root / "results", indexed, c, TaskContext{});
        require(tableBatch.success, "indexed laser plane workflow");
        near(tableBatch.batch.points[480].z / batch.batch.points[480].z, 1.01, 1e-8,
             "per-position geometry selected");
        std::string crlf = cloudText(".pcd", {{1, 2, 3}});
        for (std::size_t i = 0; i < crlf.size(); ++i)
            if (crlf[i] == '\n')
            {
                crlf.insert(i, 1, '\r');
                ++i;
            }
        writeText(root / "windows.PCD", crlf);
        near(importCloud(root / "windows.PCD").front().z, 3, 1e-9, "Windows CRLF and uppercase PCD");
        c.mode = 2;
        const auto missing =
            reconstructionWorkflow(root / "data/scan", root / "results", calibration, c, TaskContext{});
        require(!missing.success, "indexed plane preflight");
        c.mode = 1;
        c.stepMm = 2;
        const auto translated =
            reconstructionWorkflow(root / "data/scan", root / "results", calibration, c, TaskContext{});
        require(translated.success, "linear scan workflow");
        near(translated.batch.points[480].y - batch.batch.points[480].y, 2, 1e-8,
             "per-frame translation applied");
        TaskContext cancelled;
        cancelled.cancel->store(true);
        const auto stop =
            reconstructionWorkflow(root / "data/scan", root / "results", calibration, c, cancelled);
        require(stop.cancelled && !stop.success, "task cancellation");
        writeImage(root / "ordering/frame10.png", stripe(30, 20, 12));
        writeImage(root / "ordering/frame2.png", stripe(30, 20, 12));
        const auto ordering = imageFiles(root / "ordering");
        require(ordering[0].filename() == "frame2.png", "natural frame order");
        CameraThread demo("demo", 3000, 0);
        QEventLoop loop;
        int received = 0;
        qRegisterMetaType<cv::Mat>("cv::Mat");
        QObject::connect(
            &demo, &CameraThread::frameReady, &loop,
            [&](const cv::Mat& f)
            {
                if (!f.empty())
                    ++received;
                if (received >= 3)
                {
                    demo.stop();
                    loop.quit();
                }
            },
            Qt::QueuedConnection);
        QTimer::singleShot(3000, &loop, &QEventLoop::quit);
        demo.start();
        loop.exec();
        demo.stop();
        demo.wait();
        require(received >= 3, "simulated acquisition lifecycle");
        std::cout << "PASS: " << checks << " assertions, camera RMS=" << calibrated.rms
                  << " px, laser RMS=" << laser.rmsMm << " mm\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAIL after " << checks << " checks: " << error.what() << "\n";
        return 1;
    }
}
