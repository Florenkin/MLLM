#include "app/acquisition/Camera.h"
#include "app/services/CaptureSessionService.h"
#include "app/services/TaskRunner.h"
#include "app/ui/MainWindow.h"
#include "app/ui/MainWindowHelpers.h"
#include "app/ui/Views.h"
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTabWidget>
#include <chrono>

namespace mllm
{
using namespace ui_detail;
void MainWindow::refreshDevices()
{
    try
    {
        const auto selected = devices_->currentData().toString();
        devices_->clear();
        for (const auto& info : cameraDevices())
            devices_->addItem(info.name, info.id);
        const int index = devices_->findData(selected);
        if (index >= 0)
            devices_->setCurrentIndex(index);
        log(QString("设备列表已刷新，共 %1 项（含模拟与 USB 入口）。").arg(devices_->count()), false,
            "Camera");
    }
    catch (const std::exception& e)
    {
        log(q(e.what()), true, "Camera");
    }
}

void MainWindow::toggleCamera()
{
    if (camera_)
    {
        camera_->stop();
        cameraButton_->setEnabled(false);
        cameraStatus_->setText("正在停止预览…");
        captureButton_->setEnabled(false);
        return;
    }
    if (closing_ || !syncConfig())
        return;
    camera_ = new CameraThread(devices_->currentData().toString(), realValue("acquisition/exposure_us"),
                               realValue("acquisition/gain"), this);
    latestFrame_ = {};
    devices_->setEnabled(false);
    cameraButton_->setText("停止预览");
    connect(camera_, &CameraThread::opened, this,
            [this](const QString& name) { cameraStatus_->setText("已连接：" + name); });
    connect(camera_, &CameraThread::frameReady, this,
            [this](const CameraFrame& frame)
            {
                if (closing_)
                    return;
                latestFrame_ = frame;
                liveView_->setImage(frame.image);
                updateCaptureState();
            });
    connect(camera_, &CameraThread::errorOccurred, this,
            [this](const QString& text) { cameraStatus_->setText("连接失败：" + text); });
    connect(camera_, &QThread::finished, this,
            [this]
            {
                camera_->deleteLater();
                camera_ = nullptr;
                latestFrame_ = {};
                cameraStatus_->setText("未连接");
                cameraButton_->setText("连接 / 预览");
                cameraButton_->setEnabled(true);
                devices_->setEnabled(true);
                captureButton_->setEnabled(false);
                updateCaptureState();
            });
    camera_->start();
}

void MainWindow::newCaptureSession()
{
    try
    {
        const auto directory = captureSession_->start(fsPath("paths/output"));
        updateCaptureState();
        log("新建采集会话：" + display(directory), false, "Capture");
    }
    catch (const std::exception& error)
    {
        log(q(error.what()), true, "Capture");
    }
}

void MainWindow::captureFrame()
{
    try
    {
        if (latestFrame_.image.empty() || !camera_ || !camera_->isRunning())
            throw std::runtime_error("没有活动相机图像");
        const auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::system_clock::now().time_since_epoch())
                             .count();
        if (now - latestFrame_.metadata.hostTimeMs > 2000)
            throw std::runtime_error("相机图像已超过 2 秒未更新，请检查连接");
        if (!captureSession_->active())
            captureSession_->start(fsPath("paths/output"));
        CaptureRequest request;
        request.category = static_cast<CaptureCategory>(captureCategory_->currentIndex());
        if (request.category == CaptureCategory::Reconstruction)
        {
            request.geometry = reconstructionConfig();
            if (manualPositionEnabled_->isChecked())
                request.manualPosition = manualPosition_->value();
        }
        const auto saved = captureSession_->capture(latestFrame_, request);
        if (saved.pairPending)
        {
            captureCategory_->setCurrentIndex(2);
            captureView_->setImage(latestFrame_.image);
            centralTabs_->setCurrentIndex(2);
        }
        else
        {
            if (request.category == CaptureCategory::LaserOn)
            {
                qobject_cast<QLineEdit*>(controls_.value("paths/laser_off"))
                    ->setText(display(captureSession_->directory() / "off"));
                qobject_cast<QLineEdit*>(controls_.value("paths/laser_on"))
                    ->setText(display(captureSession_->directory() / "on"));
                captureCategory_->setCurrentIndex(1);
            }
            else
            {
                const auto key = request.category == CaptureCategory::Camera ? "paths/camera_images"
                                                                             : "paths/reconstruction";
                qobject_cast<QLineEdit*>(controls_.value(key))->setText(display(saved.directory));
            }
            loadImages(display(saved.directory));
        }
        updateCaptureState();
        log(saved.pairPending ? "关闭图已暂存，请开启激光并保持棋盘静止。"
                              : "已采集并登记：" + display(saved.file),
            false, "Capture");
    }
    catch (const std::exception& error)
    {
        log(q(error.what()), true, "Capture");
    }
}

void MainWindow::updateCaptureState()
{
    if (!captureCategory_)
        return;
    const bool pending = captureSession_->awaitingLaserOn();
    captureCategory_->setEnabled(!pending);
    abandonPairButton_->setEnabled(pending && !busy_);
    const bool reconstruction = captureCategory_->currentIndex() == 3;
    manualPositionEnabled_->setEnabled(reconstruction);
    manualPosition_->setEnabled(reconstruction && manualPositionEnabled_->isChecked());
    if (pending)
    {
        captureButton_->setText("采集开启图");
        capturePairStatus_->setText(
            QString("第 %1 组：等待开启图，保持棋盘静止。").arg(captureSession_->nextPairId()));
    }
    else if (captureCategory_->currentIndex() == 1)
    {
        captureButton_->setText("采集关闭图");
        capturePairStatus_->setText(
            QString("第 %1 组：关闭激光后采集，随后保持棋盘静止。 ").arg(captureSession_->nextPairId()));
    }
    else
    {
        captureButton_->setText("采集一帧");
        capturePairStatus_->setText(reconstruction ? "位置来自扫描参数计划值；勾选上方选项可记录人工位置。"
                                                   : "激光标定需依次采集同一姿态的关闭图与开启图。");
    }
    captureButton_->setEnabled(camera_ && camera_->isRunning() && !latestFrame_.image.empty() && !busy_ &&
                               !closing_);
}
} // namespace mllm
