#include "app/services/TaskRunner.h"
#include "app/ui/MainWindow.h"
#include "app/ui/PointCloudPanel.h"
#include "app/ui/Views.h"
#include "core/CaptureManifest.h"
#include "core/IO.h"
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDir>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QFont>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMouseEvent>
#include <QProcess>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QTemporaryDir>
#include <QThread>
#include <QTimer>
#include <functional>
#include <iostream>

using namespace mllm;
namespace
{
void require(bool value, const char* message)
{
    if (!value)
        throw std::runtime_error(message);
}
template <class T> T* control(MainWindow& window, const char* name)
{
    auto* result = window.findChild<T*>(name);
    require(result != nullptr, name);
    return result;
}
void waitUntil(const std::function<bool()>& predicate, int timeoutMs = 30000)
{
    QElapsedTimer timer;
    timer.start();
    while (!predicate() && timer.elapsed() < timeoutMs)
    {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
        QThread::msleep(5);
    }
    require(predicate(), "UI operation timed out");
}
void clickAndWait(MainWindow& window, const char* name)
{
    auto* b = control<QPushButton>(window, name);
    b->click();
    waitUntil([&] { return b->isEnabled(); });
}
bool hasLog(QTableWidget* table, const QString& text)
{
    for (int row = 0; row < table->rowCount(); ++row)
        if (table->item(row, 3)->text().contains(text))
            return true;
    return false;
}
void waitNextFrame()
{
    QElapsedTimer timer;
    timer.start();
    waitUntil([&] { return timer.elapsed() >= 150; }, 1000);
}
void cloudPanelWorkflow()
{
    TaskRunner runner;
    PointCloudPanel panel(&runner);
    panel.resize(1250, 750);
    panel.show();
    Cloud raw;
    for (int i = 0; i < 200003; ++i)
        raw.emplace_back(i % 500, i / 500, 5);
    auto original = std::make_shared<const Cloud>(std::move(raw));
    panel.addCloud("original", original);
    const auto originalId = panel.workspace().current()->id;
    QCoreApplication::processEvents();
    auto* view = panel.findChild<CloudView*>("cloudView");
    panel.findChild<QPushButton*>("cloudSelect")->click();
    QMouseEvent press(QEvent::MouseButtonPress, QPointF(3, 3), Qt::LeftButton, Qt::LeftButton,
                      Qt::NoModifier);
    QMouseEvent release(QEvent::MouseButtonRelease, QPointF(view->width() - 3, view->height() - 3),
                        Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
    QCoreApplication::sendEvent(view, &press);
    QCoreApplication::sendEvent(view, &release);
    waitUntil([&] { return !runner.isRunning(); });
    require(panel.workspace().current()->selection.size() == original->size(),
            "UI rectangle selects all raw points beyond display sample");
    panel.findChild<QCheckBox*>("cloudSelectedOnly")->setChecked(true);
    panel.findChild<QPushButton*>("cloudFitPlane")->click();
    waitUntil([&] { return !runner.isRunning(); });
    require(panel.findChild<QLabel*>("cloudMeasurement")->text().contains("RMS"),
            "UI selected plane fit displayed");
    panel.findChild<QPushButton*>("cloudExtractSelection")->click();
    waitUntil([&] { return !runner.isRunning(); });
    require(panel.workspace().documents().size() == 2 &&
                panel.workspace().current()->points->size() == original->size(),
            "UI selection creates derived entry");
    require(panel.findChild<QLabel*>("cloudMeasurement")->text().isEmpty(),
            "measurement cannot remain stale on another dataset");
    panel.findChild<QCheckBox*>("cloudSelectedOnly")->setChecked(false);
    QTimer::singleShot(0, &panel,
                       [&]
                       {
                           auto* dialog = panel.findChild<QDialog*>("cloudCropDialog");
                           if (!dialog)
                               return;
                           dialog->findChild<QComboBox*>("cloudCropAxis")->setCurrentIndex(0);
                           dialog->findChild<QDoubleSpinBox*>("cloudCropMinimum")->setValue(0);
                           dialog->findChild<QDoubleSpinBox*>("cloudCropMaximum")->setValue(10);
                           dialog->accept();
                       });
    panel.findChild<QPushButton*>("cloudCrop")->click();
    waitUntil([&] { return !runner.isRunning(); });
    require(panel.workspace().documents().size() == 3 &&
                panel.workspace().current()->points->size() < original->size(),
            "UI crop creates smaller independent dataset");
    panel.findChild<QComboBox*>("cloudDocuments")->setCurrentIndex(0);
    require(panel.workspace().current()->id == originalId && panel.workspace().current()->points == original,
            "UI can return to original cloud");
    panel.findChild<QPushButton*>("cloudClearSelection")->click();
    require(panel.workspace().current()->selection.empty(), "UI clear selection");
    panel.close();
}
} // namespace
int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    app.setStyle("Fusion");
    app.setFont(QFont("Microsoft YaHei UI", 9));
    try
    {
        QTemporaryDir tmp;
        require(tmp.isValid(), "temporary directory");
        const QString root = tmp.path();
        qputenv("MLLM_CONFIG_DIR", (root + "/config").toUtf8());
        cloudPanelWorkflow();
        QProcess generator;
        generator.start(QCoreApplication::applicationDirPath() + "/mllm_tests.exe",
                        {"--generate-demo", root + "/data"});
        require(generator.waitForFinished(30000) && generator.exitCode() == 0,
                "synthetic dataset generation");
        MainWindow window;
        window.show();
        require(window.smokeCheck(), "UI structure");
        const auto setPath = [&](const char* key, const QString& relative)
        { control<QLineEdit>(window, key)->setText(root + relative); };
        setPath("paths/camera_images", "/data/camera");
        setPath("paths/laser_off", "/data/off");
        setPath("paths/laser_on", "/data/on");
        setPath("paths/reconstruction", "/data/scan");
        setPath("paths/output", "/results");
        control<QSpinBox>(window, "calibration/columns")->setValue(9);
        control<QSpinBox>(window, "calibration/rows")->setValue(6);
        control<QDoubleSpinBox>(window, "calibration/square_mm")->setValue(20);
        control<QSpinBox>(window, "laser/threshold")->setValue(60);
        clickAndWait(window, "calibrateCameraButton");
        auto calibrationFile = control<QLineEdit>(window, "paths/calibration_file")->text();
        require(QFileInfo::exists(calibrationFile), "UI camera calibration output");
        auto calibration = loadCalibration(Path(calibrationFile.toStdWString()));
        require(calibration.camera.valid() && calibration.planes.empty(), "camera result displayed");
        clickAndWait(window, "calibrateLaserButton");
        calibrationFile = control<QLineEdit>(window, "paths/calibration_file")->text();
        calibration = loadCalibration(Path(calibrationFile.toStdWString()));
        require(calibration.planes.size() == 1, "UI laser calibration output");
        clickAndWait(window, "reconstructButton");
        const auto resultDirs =
            QDir(root + "/results").entryList({"reconstruction_*"}, QDir::Dirs | QDir::NoDotAndDotDot);
        require(resultDirs.size() == 1, "one UI reconstruction session");
        require(QFileInfo::exists(root + "/results/" + resultDirs[0] + "/completed.txt"),
                "UI reconstruction completion");
        auto* logs = control<QTableWidget>(window, "logTable");
        waitUntil([&] { return hasLog(logs, "重建完成"); });
        require(logs->columnCount() == 4, "structured UI log columns");
        // A bad input must release the busy lock and keep the application usable.
        setPath("paths/reconstruction", "/missing-images");
        clickAndWait(window, "reconstructButton");
        waitUntil([&] { return hasLog(logs, "不存在"); });
        setPath("paths/reconstruction", "/data/scan");
        control<QPushButton>(window, "reconstructButton")->click();
        control<QPushButton>(window, "cancelTaskButton")->click();
        waitUntil([&] { return control<QPushButton>(window, "reconstructButton")->isEnabled(); });
        waitUntil([&] { return hasLog(logs, "任务已取消"); });
        control<QPushButton>(window, "cameraPreviewButton")->click();
        waitUntil([&] { return control<QPushButton>(window, "captureFrameButton")->isEnabled(); }, 5000);
        control<QPushButton>(window, "captureFrameButton")->click();
        const auto sessions =
            QDir(root + "/results").entryList({"capture_*"}, QDir::Dirs | QDir::NoDotAndDotDot);
        require(sessions.size() == 1 &&
                    QFileInfo::exists(root + "/results/" + sessions[0] + "/camera/000001.png"),
                "UI simulated frame saved");
        auto* category = control<QComboBox>(window, "captureCategory");
        category->setCurrentIndex(1);
        waitNextFrame();
        control<QPushButton>(window, "captureFrameButton")->click();
        const auto sessionRoot = Path((root + "/results/" + sessions[0]).toStdWString());
        require(category->currentIndex() == 2 && !category->isEnabled() &&
                    std::filesystem::exists(sessionRoot / "pending/pair_1/off.png") &&
                    !std::filesystem::exists(sessionRoot / "off/000001.png"),
                "UI off image staged and next phase locked");
        waitNextFrame();
        control<QPushButton>(window, "captureFrameButton")->click();
        require(category->currentIndex() == 1 && category->isEnabled(), "UI completed pair resets phase");
        validateLaserPairManifest(sessionRoot / "off", sessionRoot / "on");
        require(std::filesystem::weakly_canonical(
                    Path(control<QLineEdit>(window, "paths/laser_off")->text().toStdWString())) ==
                        std::filesystem::weakly_canonical(sessionRoot / "off") &&
                    std::filesystem::weakly_canonical(
                        Path(control<QLineEdit>(window, "paths/laser_on")->text().toStdWString())) ==
                        std::filesystem::weakly_canonical(sessionRoot / "on"),
                "UI updates both paired input paths");
        waitNextFrame();
        control<QPushButton>(window, "captureFrameButton")->click();
        control<QPushButton>(window, "abandonLaserPairButton")->click();
        require(category->isEnabled() &&
                    std::filesystem::exists(sessionRoot / "pending/pair_2/abandoned.txt"),
                "UI abandons incomplete group with audit retained");
        category->setCurrentIndex(3);
        control<QComboBox>(window, "reconstruction/mode")->setCurrentIndex(1);
        control<QDoubleSpinBox>(window, "reconstruction/step_mm")->setValue(2);
        control<QCheckBox>(window, "captureManualPositionEnabled")->setChecked(true);
        control<QDoubleSpinBox>(window, "captureManualPosition")->setValue(12);
        waitNextFrame();
        control<QPushButton>(window, "captureFrameButton")->click();
        const auto captureAudit = loadCaptureManifest(sessionRoot / "capture_manifest.yml");
        require(captureAudit.records.back().category == CaptureCategory::Reconstruction &&
                    captureAudit.records.back().translationMm == 12 &&
                    captureAudit.records.back().positionSource == "manual",
                "UI manual frame position recorded");
        control<QPushButton>(window, "reconstructButton")->click();
        window.close();
        waitUntil([&] { return !window.isVisible(); }, 5000);
        MainWindow reopened;
        require(control<QSpinBox>(reopened, "calibration/columns")->value() == 9, "settings restored");
        require(control<QLineEdit>(reopened, "paths/calibration_file")->text() == calibrationFile,
                "calibration path restored");
        reopened.close();
        std::cout << "PASS: UI camera calibration -> laser plane -> reconstruction, error recovery, "
                     "cancellation, paired capture, audited positions, raw cloud selection/crop/fit, close "
                     "during work and settings restore\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "FAIL: " << e.what() << "\n";
        return 1;
    }
}
