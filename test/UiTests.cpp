#include "app/ui/MainWindow.h"
#include "core/IO.h"
#include <QApplication>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QFont>
#include <QLineEdit>
#include <QListWidget>
#include <QPlainTextEdit>
#include <QProcess>
#include <QPushButton>
#include <QSpinBox>
#include <QTemporaryDir>
#include <QThread>
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
        auto* logs = window.findChild<QPlainTextEdit*>();
        require(logs->toPlainText().contains("重建完成"), "UI completion log");
        // A bad input must release the busy lock and keep the application usable.
        setPath("paths/reconstruction", "/missing-images");
        clickAndWait(window, "reconstructButton");
        require(logs->toPlainText().contains("不存在"), "UI invalid directory feedback");
        setPath("paths/reconstruction", "/data/scan");
        control<QPushButton>(window, "reconstructButton")->click();
        control<QPushButton>(window, "cancelTaskButton")->click();
        waitUntil([&] { return control<QPushButton>(window, "reconstructButton")->isEnabled(); });
        require(logs->toPlainText().contains("任务已取消"), "UI cancellation");
        control<QPushButton>(window, "cameraPreviewButton")->click();
        waitUntil([&] { return control<QPushButton>(window, "captureFrameButton")->isEnabled(); }, 5000);
        control<QPushButton>(window, "captureFrameButton")->click();
        const auto sessions =
            QDir(root + "/results").entryList({"capture_*"}, QDir::Dirs | QDir::NoDotAndDotDot);
        require(sessions.size() == 1 &&
                    QFileInfo::exists(root + "/results/" + sessions[0] + "/camera/000001.png"),
                "UI simulated frame saved");
        window.close();
        waitUntil([&] { return !window.isVisible(); }, 5000);
        MainWindow reopened;
        require(control<QSpinBox>(reopened, "calibration/columns")->value() == 9, "settings restored");
        require(control<QLineEdit>(reopened, "paths/calibration_file")->text() == calibrationFile,
                "calibration path restored");
        reopened.close();
        std::cout << "PASS: UI camera calibration -> laser plane -> reconstruction, error recovery, "
                     "cancellation, simulated capture, close and settings restore\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "FAIL: " << e.what() << "\n";
        return 1;
    }
}
