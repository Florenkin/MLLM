#include "app/ui/MainWindow.h"
#include "app/acquisition/Camera.h"
#include "app/services/AppConfigService.h"
#include "app/services/AppPaths.h"
#include "app/services/CaptureSessionService.h"
#include "app/services/QtLogSink.h"
#include "app/services/TaskRunner.h"
#include "app/ui/LogPanel.h"
#include "app/ui/MainWindowHelpers.h"
#include "core/IO.h"
#include "core/Logger.h"
#include <QCloseEvent>
#include <QFileInfo>
#include <QLineEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QStatusBar>
#include <QTabWidget>
#include <QTableWidget>
#include <QTimer>

namespace mllm
{
using namespace ui_detail;
MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent)
{
    configDir_ = applicationConfigDirectory();
    baseDir_ = QFileInfo(configDir_).absolutePath();
    config_ = std::make_unique<AppConfigService>(configDir_);
    config_->load();
    captureSession_ = std::make_unique<CaptureSessionService>();
    runner_ = new TaskRunner(this);
    qRegisterMetaType<CameraFrame>("mllm::CameraFrame");
    setWindowTitle("MLLM — 单目线激光测量");
    resize(1440, 920);
    setStyleSheet(
        "QMainWindow {background:#d9d9d9;} QDockWidget::title {background:#d6d6d6;padding:4px;border:1px "
        "solid #b8b8b8;} QTabWidget::pane {border:1px solid #b8b8b8;background:#f4f4f4;} QTabBar::tab "
        "{background:#e8e8e8;padding:4px 12px;border:1px solid #b8b8b8;} QTabBar::tab:selected "
        "{background:white;} QLineEdit,QSpinBox,QDoubleSpinBox,QComboBox {background:white;border:1px solid "
        "#c0c0c0;padding:2px;} QPushButton {padding:4px 8px;}");
    settingsTimer_ = new QTimer(this);
    settingsTimer_->setSingleShot(true);
    settingsTimer_->setInterval(500);
    connect(settingsTimer_, &QTimer::timeout, this, &MainWindow::saveSettings);
    buildUi();
    logSink_ = std::make_unique<QtLogSink>();
    connect(logSink_.get(), &QtLogSink::messageReceived, messages_, &LogPanel::appendMessage);
    connect(logSink_.get(), &QtLogSink::messageReceived, this,
            [this](const LogMessage& message) { statusBar()->showMessage(q(message.text)); });
    connect(runner_, &TaskRunner::started, this,
            [this]
            {
                busy_ = true;
                panel_->setEnabled(false);
                centralTabs_->setEnabled(false);
                cancelButton_->setEnabled(true);
                progress_->setValue(0);
                saveSettings();
            });
    connect(runner_, &TaskRunner::progress, this,
            [this](int percent, const QString& text)
            {
                progress_->setValue(percent);
                statusBar()->showMessage(text);
            });
    connect(runner_, &TaskRunner::finished, this, &MainWindow::finishJob);
    for (const auto& warning : config_->warnings())
        Logger::instance().warning("Config", warning.toUtf8().toStdString());
    refreshDevices();
    if (!pathValue("paths/calibration_file").isEmpty())
        try
        {
            calibration_ = loadCalibration(fsPath("paths/calibration_file"));
        }
        catch (const std::exception& error)
        {
            log(q(error.what()), true, "Calibration");
        }
    updateCalibration();
    updateCaptureState();
    log("MLLM 已启动。流程：相机标定 → 激光平面标定 → 重建。坐标单位 mm，介质为空气。", false, "App");
}

MainWindow::~MainWindow()
{
    runner_->cancel();
    if (camera_)
    {
        camera_->stop();
        camera_->wait();
    }
    runner_->wait();
}

void MainWindow::log(const QString& text, bool error, const char* module)
{
    Logger::instance().log(error ? LogLevel::Error : LogLevel::Info, module, text.toUtf8().toStdString());
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    closing_ = true;
    runner_->cancel();
    if (camera_)
        camera_->stop();
    if (runner_->isRunning() || (camera_ && camera_->isRunning()))
    {
        event->ignore();
        statusBar()->showMessage("正在取消任务并释放相机…");
        QTimer::singleShot(100, this, [this] { close(); });
        return;
    }
    saveSettings();
    QMainWindow::closeEvent(event);
}

bool MainWindow::smokeCheck() const
{
    return centralTabs_->count() == 4 && findChild<QPushButton*>("calibrateCameraButton") &&
           findChild<QPushButton*>("calibrateLaserButton") && findChild<QPushButton*>("reconstructButton") &&
           planeTable_->columnCount() == 3 && messages_->findChild<QTableWidget*>("logTable");
}
} // namespace mllm
