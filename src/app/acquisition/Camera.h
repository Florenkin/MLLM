#pragma once
#include <QString>
#include <QThread>
#include <atomic>
#include <opencv2/core.hpp>
#include <vector>

Q_DECLARE_METATYPE(cv::Mat)
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
    void frameReady(const cv::Mat& frame);
    void errorOccurred(const QString& error);
    void opened(const QString& name);

  protected:
    void run() override;

  private:
    QString id_;
    double exposure_, gain_;
    std::atomic_bool stopping_{false};
};
} // namespace mllm
