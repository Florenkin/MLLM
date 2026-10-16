#include "app/services/CaptureSessionService.h"
#include "core/IO.h"
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSaveFile>
#include <algorithm>
#include <iomanip>
#include <opencv2/imgcodecs.hpp>
#include <sstream>

namespace mllm
{
namespace
{
QString qtPath(const Path& path)
{
    return QString::fromStdWString(path.wstring());
}
void atomicText(const Path& path, const std::string& text)
{
    QSaveFile file(qtPath(path));
    if (!file.open(QIODevice::WriteOnly) ||
        file.write(text.data(), static_cast<qint64>(text.size())) != static_cast<qint64>(text.size()) ||
        !file.commit())
        throw std::runtime_error("采集记录保存失败：" + path.u8string());
}
void atomicImage(const Path& path, const cv::Mat& image)
{
    if (std::filesystem::exists(path))
        throw std::runtime_error("采集图像已存在：" + path.u8string());
    std::filesystem::create_directories(path.parent_path());
    std::vector<uchar> bytes;
    if (!cv::imencode(".png", image, bytes))
        throw std::runtime_error("采集图像编码失败");
    QSaveFile file(qtPath(path));
    if (!file.open(QIODevice::WriteOnly) ||
        file.write(reinterpret_cast<const char*>(bytes.data()), static_cast<qint64>(bytes.size())) !=
            static_cast<qint64>(bytes.size()) ||
        !file.commit())
        throw std::runtime_error("采集图像保存失败：" + path.u8string());
}
std::string numbered(std::size_t index)
{
    std::ostringstream name;
    name << std::setw(6) << std::setfill('0') << index << ".png";
    return name.str();
}
} // namespace
CaptureSessionService::CaptureSessionService(ManifestCommit commit)
    : commit_(commit ? std::move(commit) : atomicText)
{
}
Path CaptureSessionService::start(const Path& output)
{
    if (output.empty())
        throw std::runtime_error("请设置采集输出目录");
    if (awaitingLaserOn())
        throw std::runtime_error("请先完成或放弃当前激光图像对");
    directory_ = uniqueDirectory(output, "capture");
    manifest_ = {};
    manifest_.sessionId = directory_.filename().u8string();
    nextPair_ = 1;
    attempt_ = 0;
    failed_ = false;
    pendingOff_.clear();
    return directory_;
}
void CaptureSessionService::bindFrame(const CameraFrame& frame)
{
    if (failed_)
        throw std::runtime_error("采集保存未能恢复，请新建会话");
    if (frame.image.empty() || frame.metadata.deviceId.empty() || frame.metadata.streamId.empty() ||
        frame.metadata.hostTimeMs <= 0 || frame.metadata.lostPackets ||
        !std::isfinite(frame.metadata.requestedExposureUs) || !std::isfinite(frame.metadata.requestedGain))
        throw std::runtime_error("采集图像或帧元数据无效");
    if (manifest_.deviceId.empty())
    {
        manifest_.deviceId = frame.metadata.deviceId;
        manifest_.imageSize = frame.image.size();
    }
    if (manifest_.deviceId != frame.metadata.deviceId || manifest_.imageSize != frame.image.size())
        throw std::runtime_error("相机或分辨率已改变，请新建采集会话");
}
void CaptureSessionService::commit(const CaptureManifest& manifest)
{
    commit_(directory_ / "capture_manifest.yml", captureManifestText(manifest));
}
CaptureSaveResult CaptureSessionService::capture(const CameraFrame& frame, const CaptureRequest& request)
{
    if (!active())
        throw std::runtime_error("请先新建采集会话");
    bindFrame(frame);
    for (const auto& record : manifest_.records)
        if (record.frame.streamId == frame.metadata.streamId &&
            record.frame.frameNumber == frame.metadata.frameNumber)
            throw std::runtime_error("当前相机帧已保存，请等待下一帧");
    if (awaitingLaserOn() && request.category != CaptureCategory::LaserOn)
        throw std::runtime_error("当前组等待激光开启图，请保持棋盘静止或放弃当前组");
    if (request.category == CaptureCategory::LaserOff)
    {
        const auto file = directory_ / "pending" / ("pair_" + std::to_string(nextPair_)) / "off.png";
        atomicImage(file, frame.image);
        try
        {
            commit(manifest_);
        }
        catch (...)
        {
            QFile::remove(qtPath(file));
            throw;
        }
        pendingOff_ = file;
        pendingMetadata_ = frame.metadata;
        return {file, directory_ / "pending", true, nextPair_};
    }
    if (request.category == CaptureCategory::LaserOn)
    {
        if (!awaitingLaserOn())
            throw std::runtime_error("请先采集本组激光关闭图");
        if (frame.metadata.streamId != pendingMetadata_.streamId ||
            frame.metadata.hostTimeMs < pendingMetadata_.hostTimeMs ||
            frame.metadata.frameNumber == pendingMetadata_.frameNumber ||
            frame.metadata.requestedExposureUs != pendingMetadata_.requestedExposureUs ||
            frame.metadata.requestedGain != pendingMetadata_.requestedGain)
            throw std::runtime_error("开关图像不是不同帧，或曝光/增益已改变，请重新采集本组");
        const auto stage = pendingOff_.parent_path() / ("on_attempt_" + std::to_string(++attempt_) + ".png");
        atomicImage(stage, frame.image);
        const auto name = numbered(nextPair_);
        const auto offFile = directory_ / "off" / name, onFile = directory_ / "on" / name;
        std::filesystem::create_directories(offFile.parent_path());
        std::filesystem::create_directories(onFile.parent_path());
        auto next = manifest_;
        CaptureRecord off;
        off.file = "off/" + name;
        off.category = CaptureCategory::LaserOff;
        off.captureIndex = nextPair_;
        off.pairId = nextPair_;
        off.frame = pendingMetadata_;
        auto on = off;
        on.file = "on/" + name;
        on.category = CaptureCategory::LaserOn;
        on.frame = frame.metadata;
        next.records.push_back(off);
        next.records.push_back(on);
        bool offPublished = false, onPublished = false;
        try
        {
            if (std::filesystem::exists(offFile) || std::filesystem::exists(onFile))
                throw std::runtime_error("配对目标文件已存在");
            std::filesystem::rename(pendingOff_, offFile);
            offPublished = true;
            std::filesystem::rename(stage, onFile);
            onPublished = true;
            commit(next);
        }
        catch (...)
        {
            std::error_code error;
            if (onPublished)
            {
                std::filesystem::rename(onFile, stage, error);
                if (error)
                    failed_ = true;
            }
            if (offPublished)
            {
                error.clear();
                std::filesystem::rename(offFile, pendingOff_, error);
                if (error)
                    failed_ = true;
            }
            throw;
        }
        manifest_ = std::move(next);
        pendingOff_.clear();
        const auto pair = nextPair_++;
        return {onFile, directory_ / "on", false, pair};
    }
    const auto count = static_cast<std::size_t>(std::count_if(
                           manifest_.records.begin(), manifest_.records.end(),
                           [&](const auto& item) { return item.category == request.category; })) +
                       1;
    const auto name = numbered(count);
    CaptureRecord record;
    record.category = request.category;
    record.file = std::string(captureFolder(request.category)) + '/' + name;
    record.captureIndex = count;
    record.frame = frame.metadata;
    if (request.category == CaptureCategory::Reconstruction)
    {
        const auto& geometry = request.geometry;
        if (geometry.mode < 0 || geometry.mode > 2 || geometry.axis < 0 || geometry.axis > 2 ||
            !std::isfinite(geometry.stepMm) || !std::isfinite(geometry.positionStart) ||
            !std::isfinite(geometry.positionStep) ||
            (geometry.mode == 1 && !request.manualPosition && std::abs(geometry.stepMm) < 1e-12))
            throw std::runtime_error("采集扫描参数无效");
        record.scanMode = geometry.mode;
        record.axis = geometry.axis;
        record.planePosition =
            geometry.positionStart +
            (geometry.mode == 2 ? static_cast<double>(count - 1) * geometry.positionStep : 0);
        record.translationMm = geometry.mode == 1 ? static_cast<double>(count - 1) * geometry.stepMm : 0;
        if (request.manualPosition)
        {
            if (!std::isfinite(*request.manualPosition))
                throw std::runtime_error("人工扫描位置无效");
            if (geometry.mode == 1)
                record.translationMm = *request.manualPosition;
            else
                record.planePosition = *request.manualPosition;
            record.positionSource = "manual";
        }
    }
    const auto file = directory_ / std::filesystem::u8path(record.file);
    atomicImage(file, frame.image);
    auto next = manifest_;
    next.records.push_back(record);
    try
    {
        commit(next);
    }
    catch (...)
    {
        if (!QFile::remove(qtPath(file)))
            failed_ = true;
        throw;
    }
    manifest_ = std::move(next);
    return {file, file.parent_path(), false, 0};
}
void CaptureSessionService::abandonPair()
{
    if (!awaitingLaserOn())
        return;
    atomicText(pendingOff_.parent_path() / "abandoned.txt", "Incomplete pair retained for inspection.\n");
    pendingOff_.clear();
    ++nextPair_;
}
bool CaptureSessionService::awaitingLaserOn() const
{
    return !pendingOff_.empty();
}
bool CaptureSessionService::active() const
{
    return !directory_.empty();
}
Path CaptureSessionService::directory() const
{
    return directory_;
}
std::size_t CaptureSessionService::nextPairId() const
{
    return nextPair_;
}
} // namespace mllm
