#include "app/services/AppPaths.h"
#include "app/services/FileLogSink.h"
#include "app/ui/MainWindow.h"
#include <QApplication>
#include <QFont>
#include <QTimer>
int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    app.setApplicationName("MLLM");
    app.setOrganizationName("MLLM");
    app.setStyle("Fusion");
    app.setFont(QFont("Microsoft YaHei UI", 9));
    // 文件接收器先于窗口创建，持续到窗口和后台任务全部销毁。
    mllm::FileLogSink fileLog(mllm::applicationLogDirectory());
    auto& logger = mllm::Logger::instance();
    int exitCode = 0;
    try
    {
        mllm::MainWindow window;
        window.show();
        if (fileLog.isActive())
            logger.info("Logging", "运行日志：" + fileLog.filePath().toUtf8().toStdString());
        else
            logger.error("Logging", "无法创建运行日志：" + fileLog.lastError().toUtf8().toStdString());
        if (fileLog.removedFileCount() > 0)
            logger.info("Logging",
                        "已清理超过 24 小时的日志：" + std::to_string(fileLog.removedFileCount()) + " 个");
        if (!fileLog.cleanupFailures().isEmpty())
            logger.warning("Logging", "以下过期日志无法清理：" +
                                          fileLog.cleanupFailures().join(", ").toUtf8().toStdString());
        const auto args = app.arguments();
        if (args.contains("--demo") && args.indexOf("--demo") + 1 < args.size())
            window.loadDemo(args[args.indexOf("--demo") + 1], args.contains("--demo-run"));
        if (args.contains("--smoke-test"))
            QTimer::singleShot(100, &window, [&] { app.exit(window.smokeCheck() ? 0 : 2); });
        if (args.contains("--screenshot") && args.indexOf("--screenshot") + 1 < args.size())
        {
            auto* screenshotTimer = new QTimer(&window);
            screenshotTimer->setInterval(500);
            QObject::connect(screenshotTimer, &QTimer::timeout, &window,
                             [&]
                             {
                                 if (!window.isBusy())
                                 {
                                     const bool ok =
                                         window.grab().save(args[args.indexOf("--screenshot") + 1]);
                                     app.exit(ok ? 0 : 3);
                                 }
                             });
            screenshotTimer->start();
        }
        exitCode = app.exec();
    }
    catch (const std::exception& error)
    {
        logger.error("App", "应用异常：" + std::string(error.what()));
        exitCode = 1;
    }
    logger.info("App", "MLLM 已退出，退出码 " + std::to_string(exitCode));
    return exitCode;
}
