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
    mllm::MainWindow window;
    window.show();
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
                                 const bool ok = window.grab().save(args[args.indexOf("--screenshot") + 1]);
                                 app.exit(ok ? 0 : 3);
                             }
                         });
        screenshotTimer->start();
    }
    return app.exec();
}
