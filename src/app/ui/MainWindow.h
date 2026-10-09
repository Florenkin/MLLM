#pragma once
#include "app/acquisition/Camera.h"
#include "app/services/Workflow.h"
#include <QFutureWatcher>
#include <QMainWindow>
#include <QMap>
#include <QSettings>
#include <functional>

class QComboBox;
class QDoubleSpinBox;
class QSpinBox;
class QLineEdit;
class QFormLayout;
class QLabel;
class QPlainTextEdit;
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
    void startJob(std::function<JobResult(const TaskContext&)> work);
    void finishJob();
    void updateCalibration();
    void loadCalibrationFile();
    void exportCalibrationFile();
    void exportPointCloud();
    void updateCloud();
    void loadImages(const QString& directory, bool diagnostics = false);
    void log(const QString& text, bool error = false);
    void saveSettings();
    QVariant saved(const QString& key, const QVariant& fallback) const;
    QLineEdit* pathRow(QFormLayout* form, const QString& title, const QString& key, bool file = false);
    QSpinBox* integer(QFormLayout* form, const QString& title, const QString& key, int value, int lo, int hi);
    QDoubleSpinBox* real(QFormLayout* form, const QString& title, const QString& key, double value, double lo,
                         double hi, int decimals = 3);
    QComboBox* choice(QFormLayout* form, const QString& title, const QString& key, const QStringList& names);
    int intValue(const QString& key) const;
    double realValue(const QString& key) const;
    QString pathValue(const QString& key) const;
    Path fsPath(const QString& key) const;
    BoardConfig boardConfig() const;
    LaserConfig laserConfig() const;
    ReconstructionConfig reconstructionConfig() const;
    QMap<QString, QWidget*> controls_;
    QString configDir_, baseDir_, logPath_;
    std::unique_ptr<QSettings> settings_;
    QWidget* panel_ = nullptr;
    QTabWidget* centralTabs_ = nullptr;
    ImageView *liveView_ = nullptr, *captureView_ = nullptr, *laserView_ = nullptr;
    CloudView* cloudView_ = nullptr;
    QListWidget *captureList_ = nullptr, *laserList_ = nullptr;
    QLabel *calibrationStatus_ = nullptr, *cloudStatus_ = nullptr, *cameraStatus_ = nullptr;
    QTableWidget* planeTable_ = nullptr;
    QPlainTextEdit* messages_ = nullptr;
    QProgressBar* progress_ = nullptr;
    QPushButton *cameraButton_ = nullptr, *captureButton_ = nullptr, *cancelButton_ = nullptr;
    QComboBox *devices_ = nullptr, *captureCategory_ = nullptr;
    QTimer* settingsTimer_ = nullptr;
    CameraThread* camera_ = nullptr;
    cv::Mat latestFrame_;
    Path captureSession_;
    int captureCounts_[4] = {0, 0, 0, 0};
    Calibration calibration_;
    Cloud cloud_;
    QFutureWatcher<JobResult> watcher_;
    TaskContext task_;
    bool busy_ = false, closing_ = false;
};
} // namespace mllm
