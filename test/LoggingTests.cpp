#include "app/services/AppPaths.h"
#include "app/services/FileLogSink.h"
#include "app/services/QtLogSink.h"
#include "app/services/Workflow.h"
#include "app/ui/LogPanel.h"
#include <QApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QFont>
#include <QHeaderView>
#include <QScrollBar>
#include <QSettings>
#include <QTableWidget>
#include <QTemporaryDir>
#include <QThread>
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace mllm;
namespace
{
void require(bool condition, const char* message)
{
    if (!condition)
        throw std::runtime_error(message);
}
struct ScopedSink
{
    Logger::SinkId id;
    explicit ScopedSink(Logger::Sink callback) : id(Logger::instance().addSink(std::move(callback))) {}
    ~ScopedSink()
    {
        Logger::instance().removeSink(id);
    }
    ScopedSink(const ScopedSink&) = delete;
    ScopedSink& operator=(const ScopedSink&) = delete;
};
void fixture(const QString& path, const QDateTime& modified)
{
    QFile file(path);
    require(file.open(QIODevice::WriteOnly), "fixture creation");
    require(file.write("fixture\n") == 8 && file.flush(), "fixture writing");
    require(file.setFileTime(modified, QFileDevice::FileModificationTime), "fixture modification time");
}
QString contents(const QString& path)
{
    QFile file(path);
    require(file.open(QIODevice::ReadOnly), "readable log file");
    return QString::fromUtf8(file.readAll());
}
void testFileLogging()
{
    QTemporaryDir temp;
    require(temp.isValid(), "temporary log directory");
    const QDateTime now(QDate(2026, 10, 12), QTime(12, 0), Qt::UTC);
    const auto path = [&](const QString& name) { return QDir(temp.path()).filePath(name); };
    const auto expired = path("mllm_20261011_110000_000.log");
    const auto recent = path("mllm_20261011_130000_000.log");
    const auto boundary = path("mllm_20261011_120000_000.log");
    const auto unrelated = path("user_notes.log");
    const auto legacy = path("MLLM_20261001.log");
    const auto invalid = path("mllm_20269999_120000_000.log");
    const auto directory = path("mllm_20261001_120000_000.log");
    fixture(expired, now.addSecs(-25 * 3600));
    fixture(recent, now.addSecs(-23 * 3600));
    fixture(boundary, now.addSecs(-24 * 3600));
    fixture(unrelated, now.addSecs(-48 * 3600));
    fixture(legacy, now.addSecs(-48 * 3600));
    fixture(invalid, now.addSecs(-48 * 3600));
    require(QDir().mkpath(directory), "directory named like a log");
    auto& logger = Logger::instance();
    std::atomic_int observed{0};
    ScopedSink observer([&](const LogMessage&) { ++observed; });
    QString firstFile;
    {
        FileLogSink sink(temp.path(), now);
        require(sink.isActive() && sink.removedFileCount() == 1 && sink.cleanupFailures().isEmpty(),
                "cleanup removes exactly one expired owned file");
        require(!QFileInfo::exists(expired) && QFileInfo::exists(recent) && QFileInfo::exists(boundary) &&
                    QFileInfo::exists(unrelated) && QFileInfo::exists(legacy) && QFileInfo::exists(invalid) &&
                    QFileInfo(directory).isDir(),
                "cleanup preserves boundary, unrelated, legacy and invalid files and directories");
        firstFile = sink.filePath();
        require(QFileInfo(firstFile).fileName() == "mllm_20261012_120000_000.log", "per-run file name");
        logger.debug("LaserExtraction", "hidden debug details");
        logger.info("Capture", "采集完成\n文件已保存");
        logger.warning("Camera", "关键警告");
        logger.error("Capture", "关键错误");
        auto written = contents(firstFile);
        require(!written.contains("hidden debug details") && written.contains("采集完成\\n文件已保存") &&
                    written.contains("[Warning] [Camera] 关键警告") &&
                    written.contains("[Error] [Capture] 关键错误") && written.count('\n') == 3,
                "UTF-8 records are immediately flushed, multiline text escaped, debug omitted");
        {
            QtLogSink ui;
            logger.info("App", "界面日志");
        }
        logger.info("App", "窗口销毁后的关闭日志");
        require(contents(firstFile).contains("窗口销毁后的关闭日志"), "UI sink lifetime is independent");
        {
            FileLogSink second(temp.path(), now);
            require(second.isActive() && second.filePath() != firstFile &&
                        QFileInfo(second.filePath()).fileName() == "mllm_20261012_120000_000_1.log",
                    "identical timestamps cannot overwrite existing logs");
        }
        std::vector<std::thread> workers;
        for (int thread = 0; thread < 4; ++thread)
            workers.emplace_back(
                [&, thread]
                {
                    for (int record = 0; record < 50; ++record)
                        logger.info("Worker", "worker=" + std::to_string(thread) +
                                                  ", record=" + std::to_string(record));
                });
        for (auto& worker : workers)
            worker.join();
        written = contents(firstFile);
        require(written.count("[Info] [Worker]") == 200, "concurrent records are not lost");
        for (int thread = 0; thread < 4; ++thread)
            for (int record = 0; record < 50; ++record)
                require(written.contains("worker=" + QString::number(thread) +
                                         ", record=" + QString::number(record) + '\n'),
                        "concurrent records do not interleave");
    }
    const auto finalText = contents(firstFile);
    const auto previousObserved = observed.load();
    logger.info("App", "after file sink closes");
    require(contents(firstFile) == finalText && observed == previousObserved + 1,
            "closing a file sink preserves other registered sinks");
    const auto blockedDirectory = path("not_a_directory");
    fixture(blockedDirectory, now);
    FileLogSink failed(blockedDirectory, now);
    require(!failed.isActive() && !failed.lastError().isEmpty(), "unavailable directory reports an error");
    std::cout << "PASS: UTF-8, retention boundaries, no overwrite, concurrent files and sink lifetime\n";
}
void testLoggerAndQtBridge(QApplication& app)
{
    auto& logger = Logger::instance();
    int observed = 0;
    {
        ScopedSink throwing([](const LogMessage&) { throw std::runtime_error("broken sink"); });
        ScopedSink observer([&](const LogMessage&) { ++observed; });
        logger.info("App", "other sinks still work");
        require(observed == 1, "one failed sink does not interrupt other sinks");
    }
    int selfRemoved = 0;
    Logger::SinkId self = 0;
    self = logger.addSink(
        [&](const LogMessage&)
        {
            ++selfRemoved;
            logger.removeSink(self);
            logger.info("App", "nested record after removal");
        });
    logger.info("App", "remove inside callback");
    logger.info("App", "after removal");
    require(selfRemoved == 1, "callbacks can remove themselves and log without deadlocking");

    QtLogSink ui;
    int received = 0;
    bool guiThread = true;
    LogMessage delivered;
    QObject::connect(&ui, &QtLogSink::messageReceived, &app,
                     [&](const LogMessage& message)
                     {
                         ++received;
                         guiThread = guiThread && QThread::currentThread() == app.thread();
                         delivered = message;
                     });
    const auto before = std::chrono::system_clock::now();
    std::thread worker(
        [&]
        {
            logger.debug("Camera", "filtered UI details");
            logger.warning("Camera", "后台线程警告");
        });
    worker.join();
    const auto after = std::chrono::system_clock::now();
    require(received == 0, "worker logs never call widgets directly");
    app.processEvents();
    require(received == 1 && guiThread && delivered.text == "后台线程警告" && delivered.timestamp >= before &&
                delivered.timestamp <= after,
            "GUI delivery preserves original timestamp and filters debug messages");

    std::mutex mutex;
    std::condition_variable condition;
    bool entered = false, released = false;
    ScopedSink blocker(
        [&](const LogMessage& message)
        {
            if (message.text != "destroy-race")
                return;
            std::unique_lock<std::mutex> lock(mutex);
            entered = true;
            condition.notify_all();
            condition.wait(lock, [&] { return released; });
        });
    auto disposable = std::make_unique<QtLogSink>();
    int lateMessages = 0;
    QObject::connect(disposable.get(), &QtLogSink::messageReceived, &app,
                     [&](const LogMessage&) { ++lateMessages; });
    logger.info("App", "queued-before-destruction");
    std::thread racingWorker([&] { logger.info("App", "destroy-race"); });
    {
        std::unique_lock<std::mutex> lock(mutex);
        condition.wait(lock, [&] { return entered; });
    }
    disposable.reset();
    {
        std::lock_guard<std::mutex> lock(mutex);
        released = true;
    }
    condition.notify_all();
    racingWorker.join();
    app.processEvents();
    require(lateMessages == 0, "destroyed GUI sink drops queued and already-snapshotted callbacks safely");
    std::cout << "PASS: independent callbacks, queued GUI delivery and concurrent destruction\n";
}
void testLogPanel(QApplication& app)
{
    LogPanel panel;
    auto* table = panel.findChild<QTableWidget*>("logTable");
    require(table && table->columnCount() == 4, "four-column log panel");
    const QDateTime eventTime(QDate(2026, 10, 12), QTime(8, 15, 10, 123));
    panel.appendMessage({LogLevel::Debug, "Camera", "hidden details"});
    panel.appendMessage(
        {LogLevel::Info, "Capture", "采集完成\n文件已保存",
         std::chrono::system_clock::time_point(std::chrono::milliseconds(eventTime.toMSecsSinceEpoch()))});
    panel.appendMessage({LogLevel::Warning, "Camera", "相机警告"});
    panel.appendMessage({LogLevel::Error, "Capture", "采集失败"});
    require(table->rowCount() == 3 && table->item(0, 0)->text() == "08:15:10.123" &&
                table->item(0, 3)->text() == "采集完成 文件已保存" &&
                table->item(0, 3)->toolTip() == "采集完成\n文件已保存",
            "table uses event time, readable single-line text and original tooltip");
    panel.resize(1180, panel.sizeHint().height());
    panel.show();
    app.processEvents();
    const auto rowHeight = table->verticalHeader()->defaultSectionSize();
    require(table->viewport()->height() >= 2 * rowHeight && table->viewport()->height() < 3 * rowHeight,
            "two rows are visible at the default panel height");
    require(table->verticalScrollBar()->value() == table->verticalScrollBar()->maximum(),
            "auto-scroll to latest logs");
    const auto arguments = app.arguments();
    if (arguments.size() == 3 && arguments[1] == "--render-preview")
    {
        QDir().mkpath(QFileInfo(arguments[2]).absolutePath());
        require(panel.grab().save(arguments[2]), "save log panel preview");
    }
    panel.resize(1180, 300);
    app.processEvents();
    require(table->viewport()->height() > 2 * rowHeight, "log history panel can be expanded");
    for (int record = 0; record < 2005; ++record)
        panel.appendMessage({LogLevel::Info, "Workflow", std::to_string(record)});
    require(table->rowCount() == 2000 && table->item(1999, 3)->text() == "2004", "bounded UI history");
    panel.clear();
    require(table->rowCount() == 0, "clear UI history");
    std::cout << "PASS: timestamped table, compact height, expandable and bounded history\n";
}
void testPathsAndWorkflow()
{
    QTemporaryDir temp;
    require(temp.isValid(), "temporary configuration directory");
    const auto previous = qgetenv("MLLM_CONFIG_DIR");
    const auto wasSet = qEnvironmentVariableIsSet("MLLM_CONFIG_DIR");
    const auto config = temp.path() + "/config";
    QDir().mkpath(config);
    qputenv("MLLM_CONFIG_DIR", config.toUtf8());
    require(applicationLogDirectory() == temp.path() + "/log", "default log path follows config parent");
    {
        QSettings settings(config + "/settings.ini", QSettings::IniFormat);
        settings.setIniCodec("UTF-8");
        settings.setValue("paths/logDirectory", "自定义日志");
        settings.sync();
    }
    require(applicationLogDirectory() == temp.path() + "/自定义日志", "custom relative UTF-8 log directory");
    {
        QSettings settings(config + "/settings.ini", QSettings::IniFormat);
        settings.setValue("paths/logDirectory", temp.path() + "/absolute-logs");
        settings.sync();
    }
    require(applicationLogDirectory() == temp.path() + "/absolute-logs", "custom absolute log directory");
    if (wasSet)
        qputenv("MLLM_CONFIG_DIR", previous);
    else
        qunsetenv("MLLM_CONFIG_DIR");

    std::vector<LogMessage> messages;
    ScopedSink observer([&](const LogMessage& message) { messages.push_back(message); });
    const Path root(temp.path().toStdWString());
    const auto failure = cameraWorkflow(root / "missing", root / "output", BoardConfig{}, TaskContext{});
    require(!failure.succeeded() && !failure.isCancelled() && messages.back().level == LogLevel::Error &&
                messages.back().module == "CameraCalibration",
            "service failures have error level and module");
    TaskContext cancelled;
    cancelled.cancel->store(true);
    const auto result = singleFrameWorkflow(root / "image.png", root / "output", Calibration{},
                                            ReconstructionConfig{}, cancelled);
    require(result.isCancelled() && !result.succeeded() && messages.back().level == LogLevel::Warning &&
                messages.back().module == "Reconstruction" && messages.back().text == "任务已取消",
            "cancellation is a warning recorded by the worker service");
    std::cout << "PASS: configurable paths, service errors and cancellation levels\n";
}
} // namespace
int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    app.setFont(QFont("Microsoft YaHei UI", 9));
    try
    {
        testFileLogging();
        testLoggerAndQtBridge(app);
        testLogPanel(app);
        testPathsAndWorkflow();
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
