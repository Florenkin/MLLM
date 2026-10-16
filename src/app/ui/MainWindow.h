#pragma once
#include "core/CaptureManifest.h"
#include <QMainWindow>
#include <QMap>
#include <functional>

class QComboBox;
class QCheckBox;
class QDoubleSpinBox;
class QSpinBox;
class QLineEdit;
class QFormLayout;
class QLabel;
class QProgressBar;
class QTabWidget;
class QListWidget;
class QTableWidget;
class QPushButton;
class QTimer;
namespace mllm
{
class ImageView;
class CloudView;
class LogPanel;
class QtLogSink;
class AppConfigService;
class CaptureSessionService;
class TaskRunner;
class PointCloudPanel;
class CameraThread;
enum class JobKind;
struct JobResult;
class MainWindow final : public QMainWindow
{
    Q_OBJECT
  public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;
    bool smokeCheck() const;
    void loadDemo(const QString& directory, bool reconstruct = false);
    bool isBusy() const
    {
        return busy_;
    }

  protected:
    void closeEvent(QCloseEvent*) override;

  private:
    void buildUi();
    void refreshDevices();
    void toggleCamera();
    void captureFrame();
    void newCaptureSession();
    void startCameraCalibration();
    void startLaserCalibration();
    void startReconstruction(bool single = false);
    void startJob(JobKind kind, std::function<JobResult(const TaskContext&)> work);
    void finishJob(const JobResult& result);
    void updateCalibration();
    void loadCalibrationFile();
    void exportCalibrationFile();
    void updateCaptureState();
    void loadImages(const QString& directory, bool diagnostics = false);
    void log(const QString& text, bool error = false, const char* module = "UI");
    void saveSettings();
    bool syncConfig();
    QVariant saved(const QString& key, const QVariant& fallback) const;
    QLineEdit* pathRow(QFormLayout* form, const QString& title, const QString& key, bool file = false);
    QSpinBox* integer(QFormLayout* form, const QString& title, const QString& key, int value, int lo, int hi);
    QDoubleSpinBox* real(QFormLayout* form, const QString& title, const QString& key, double value, double lo,
                         double hi, int decimals = 3);
    QComboBox* choice(QFormLayout* form, const QString& title, const QString& key, const QStringList& names);
    double realValue(const QString& key) const;
    QString pathValue(const QString& key) const;
    Path fsPath(const QString& key) const;
    BoardConfig boardConfig();
    LaserConfig laserConfig();
    ReconstructionConfig reconstructionConfig();
    QMap<QString, QWidget*> controls_;
    QString configDir_, baseDir_;
    std::unique_ptr<AppConfigService> config_;
    std::unique_ptr<CaptureSessionService> captureSession_;
    std::unique_ptr<QtLogSink> logSink_;
    QWidget* panel_ = nullptr;
    QTabWidget* centralTabs_ = nullptr;
    ImageView *liveView_ = nullptr, *captureView_ = nullptr, *laserView_ = nullptr;
    PointCloudPanel* cloudPanel_ = nullptr;
    QListWidget *captureList_ = nullptr, *laserList_ = nullptr;
    QLabel *calibrationStatus_ = nullptr, *cameraStatus_ = nullptr, *capturePairStatus_ = nullptr;
    QTableWidget* planeTable_ = nullptr;
    LogPanel* messages_ = nullptr;
    QProgressBar* progress_ = nullptr;
    QPushButton *cameraButton_ = nullptr, *captureButton_ = nullptr, *cancelButton_ = nullptr;
    QComboBox *devices_ = nullptr, *captureCategory_ = nullptr;
    QCheckBox* manualPositionEnabled_ = nullptr;
    QDoubleSpinBox* manualPosition_ = nullptr;
    QPushButton* abandonPairButton_ = nullptr;
    QTimer* settingsTimer_ = nullptr;
    CameraThread* camera_ = nullptr;
    CameraFrame latestFrame_;
    Calibration calibration_;
    TaskRunner* runner_ = nullptr;
    bool busy_ = false, closing_ = false;
};
} // namespace mllm
