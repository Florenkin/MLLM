#pragma once
#include "core/Types.h"
#include <cstdint>
#include <optional>

namespace mllm
{
enum class CaptureCategory
{
    Camera,
    LaserOff,
    LaserOn,
    Reconstruction
};
struct FrameMetadata
{
    std::string deviceId;
    std::uint64_t frameNumber = 0;
    std::int64_t hostTimeMs = 0;
    bool deviceFrameNumber = false;
    unsigned int lostPackets = 0;
    double requestedExposureUs = 0, requestedGain = 0;
    std::string streamId;
};
struct CameraFrame
{
    cv::Mat image;
    FrameMetadata metadata;
};
struct CaptureRecord
{
    std::string file;
    CaptureCategory category = CaptureCategory::Camera;
    std::size_t captureIndex = 0, pairId = 0;
    FrameMetadata frame;
    int scanMode = 0, axis = 1;
    double planePosition = 0, translationMm = 0;
    std::string positionSource = "planned";
};
struct CaptureManifest
{
    std::string sessionId, deviceId;
    cv::Size imageSize;
    std::vector<CaptureRecord> records;
};
const char* captureFolder(CaptureCategory category);
std::string captureManifestText(const CaptureManifest& manifest);
CaptureManifest loadCaptureManifest(const Path& file);
Path findCaptureManifest(const Path& images);
struct FramePlan
{
    Path file;
    std::size_t sourceIndex = 0;
    double planePosition = 0, translationMm = 0;
};
std::vector<FramePlan> reconstructionFrames(const Path& images, const ReconstructionConfig& config,
                                            std::vector<std::string>& warnings, const TaskContext& task);
void validateLaserPairManifest(const Path& off, const Path& on);
} // namespace mllm
