#pragma once
#include "core/CaptureManifest.h"
#include <QString>
#include <QThread>
#include <atomic>
#include <opencv2/core.hpp>
#include <vector>

Q_DECLARE_METATYPE(mllm::CameraFrame)
namespace mllm
{
struct CameraInfo
{
    QString id, name;
};
std::vector<CameraInfo> cameraDevices();
class CameraThread final : public QThread
{
    Q_OBJECT
  public:
    CameraThread(QString id, double exposure, double gain, QObject* parent = nullptr);
    ~CameraThread() override;
    void stop();
  signals:
    void frameReady(const mllm::CameraFrame& frame);
    void errorOccurred(const QString& error);
    void opened(const QString& name);

  protected:
    void run() override;

  private:
    void stream();
    void publish(const cv::Mat& image, std::uint64_t frameNumber, bool deviceNumber,
                 unsigned int lostPackets = 0);
    QString id_;
    std::string streamId_;
    double exposure_, gain_;
    std::atomic_bool stopping_{false};
};
} // namespace mllm
