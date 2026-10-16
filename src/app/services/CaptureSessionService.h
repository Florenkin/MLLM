#pragma once
#include "core/CaptureManifest.h"

namespace mllm
{
struct CaptureRequest
{
    CaptureCategory category = CaptureCategory::Camera;
    ReconstructionConfig geometry;
    std::optional<double> manualPosition;
};
struct CaptureSaveResult
{
    Path file, directory;
    bool pairPending = false;
    std::size_t pairId = 0;
};
class CaptureSessionService final
{
  public:
    using ManifestCommit = std::function<void(const Path&, const std::string&)>;
    explicit CaptureSessionService(ManifestCommit commit = {});
    Path start(const Path& output);
    CaptureSaveResult capture(const CameraFrame& frame, const CaptureRequest& request);
    void abandonPair();
    bool awaitingLaserOn() const;
    bool active() const;
    Path directory() const;
    std::size_t nextPairId() const;

  private:
    void bindFrame(const CameraFrame& frame);
    void commit(const CaptureManifest& manifest);
    Path directory_, pendingOff_;
    FrameMetadata pendingMetadata_;
    CaptureManifest manifest_;
    ManifestCommit commit_;
    std::size_t nextPair_ = 1, attempt_ = 0;
    bool failed_ = false;
};
} // namespace mllm
