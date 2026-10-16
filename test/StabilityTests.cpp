#include "app/services/AppConfigService.h"
#include "app/services/AppPaths.h"
#include "app/services/CaptureSessionService.h"
#include "app/services/ConfigFile.h"
#include "app/services/PointCloudWorkflow.h"
#include "app/services/TaskRunner.h"
#include "core/Calibration.h"
#include "core/IO.h"
#include "core/Reconstruction.h"
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QSaveFile>
#include <QSettings>
#include <QTemporaryDir>
#include <QThread>
#include <atomic>
#include <cmath>
#include <iostream>
#include <limits>

using namespace mllm;
namespace
{
int checks = 0;
void writeFixture(const Path& file, const std::string& text)
{
    std::filesystem::create_directories(file.parent_path());
    QSaveFile output(QString::fromStdWString(file.wstring()));
    if (!output.open(QIODevice::WriteOnly) ||
        output.write(text.data(), static_cast<qint64>(text.size())) != static_cast<qint64>(text.size()) ||
        !output.commit())
        throw std::runtime_error("test fixture write failed");
}
void require(bool value, const char* message)
{
    ++checks;
    if (!value)
        throw std::runtime_error(message);
}
void fails(const std::function<void()>& action, const char* message)
{
    bool rejected = false;
    try
    {
        action();
    }
    catch (const std::exception&)
    {
        rejected = true;
    }
    require(rejected, message);
}
void waitUntil(const std::function<bool()>& predicate)
{
    QElapsedTimer timer;
    timer.start();
    while (!predicate() && timer.elapsed() < 5000)
    {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
        QThread::msleep(1);
    }
    require(predicate(), "background job timed out");
}
CameraFrame frame(std::uint64_t number, std::string stream = "stream-A")
{
    CameraFrame result;
    result.image = cv::Mat(24, 32, CV_8UC1, cv::Scalar(0));
    result.image.col(16).setTo(255);
    result.metadata = {"demo", number,           static_cast<std::int64_t>(1000 + number), false, 0, 3000,
                       0,      std::move(stream)};
    return result;
}
void configurations(const Path& root)
{
    const auto directory = root / "config";
    writeFixture(directory / "settings.ini", "[calibration]\ncolumns=9\nrows=6\nsquare_mm=20\n[paths]"
                                             "\nlogDirectory=C:/Project/MLLM/log\noutput=results\n");
    AppConfigService config(QString::fromStdWString(directory.wstring()));
    config.load();
    require(config.board().corners == cv::Size(9, 6) && config.board().squareMm == 20,
            "legacy config migration");
    require(std::filesystem::exists(directory / "calibration.ini") &&
                std::filesystem::exists(directory / "paths.ini"),
            "categorized files initialized");
    const auto legacy = readText(directory / "settings.ini");
    const std::string customized = "; user comment\n[laser]\nthreshold=75\ncustom_future=keep\n";
    writeFixture(directory / "laser.ini", customized);
    config.load();
    require(config.laser().threshold == 75, "categorized config overrides legacy");
    require(!config.update({{"laser/threshold", 999}}) && config.laser().threshold == 75,
            "invalid update preserves cached values");
    require(!config.update({{"laser/width", std::numeric_limits<double>::quiet_NaN()}}),
            "nonfinite config rejected");
    require(config.update({{"laser/threshold", 80}}) && config.save(), "validated config save");
    const auto saved = readText(directory / "laser.ini");
    require(saved.find("; user comment") != std::string::npos &&
                saved.find("custom_future=keep") != std::string::npos,
            "comments and unknown fields retained");
    require(readText(directory / "settings.ini") == legacy, "legacy source preserved");
    const auto quotedRoot = root / "quoted-legacy";
    std::filesystem::create_directories(quotedRoot);
    const QString quotedPath = QString::fromStdWString((root / "中文,folder").wstring());
    {
        QSettings old(QString::fromStdWString((quotedRoot / "settings.ini").wstring()), QSettings::IniFormat);
        old.setIniCodec("UTF-8");
        old.setValue("paths/output", quotedPath);
        old.sync();
    }
    AppConfigService quoted(QString::fromStdWString(quotedRoot.wstring()));
    quoted.load();
    require(quoted.value("paths/output").toString() == quotedPath, "legacy quoted Unicode path decoded");
    const auto manualRoot = root / "manual-legacy";
    const auto manualPath = (root / "Windows path").wstring();
    writeFixture(manualRoot / "settings.ini", "[paths]\noutput=" + Path(manualPath).u8string() +
                                                  "\nlogDirectory=C:\\Project\\MLLM\\log\n");
    const auto previousConfig = qgetenv("MLLM_CONFIG_DIR");
    qputenv("MLLM_CONFIG_DIR", QString::fromStdWString(manualRoot.wstring()).toUtf8());
    require(applicationLogDirectory().replace('\\', '/') == "C:/Project/MLLM/log",
            "log bootstrap preserves handwritten Windows path");
    qputenv("MLLM_CONFIG_DIR", previousConfig);
    AppConfigService manual(QString::fromStdWString(manualRoot.wstring()));
    manual.load();
    require(manual.value("paths/output").toString() == QString::fromStdWString(manualPath),
            "handwritten Windows separators preserved in migration");
    writeFixture(directory / "laser.ini", "[laser]\nthreshold=broken\nwidth=nan\n");
    config.load();
    require(config.laser().threshold == 100 && config.laser().width == 3 && !config.warnings().empty(),
            "invalid values have validated defaults and warnings");
    const std::string corrupt = "[laser]\nthreshold=1\nthreshold=2\n";
    writeFixture(directory / "laser.ini", corrupt);
    config.load();
    require(!config.save() && readText(directory / "laser.ini") == corrupt,
            "malformed config protected from overwrite");
    const auto brokenRoot = root / "broken-config";
    writeFixture(brokenRoot / "settings.ini", "not an INI document\n");
    AppConfigService broken(QString::fromStdWString(brokenRoot.wstring()));
    broken.load();
    require(!broken.save() && !std::filesystem::exists(brokenRoot / "paths.ini"),
            "malformed legacy prevents migration overwrite");
    ConfigFile duplicate(QString::fromStdWString((directory / "laser.ini").wstring()));
    require(!duplicate.error().isEmpty(), "duplicate config key detected");
    ConfigFile reusable(QString::fromStdWString((root / "reusable.ini").wstring()));
    require(reusable.save({{"paths/output", "first", "comment"}}) &&
                reusable.save({{"paths/logDirectory", "second", {}}}) &&
                reusable.value("paths/output") == "first",
            "repeated config writes preserve previous edits");
    require(!reusable.save({{"paths/output", "bad\nvalue", {}}}) &&
                reusable.save({{"paths/output", "recovered", {}}}) &&
                reusable.value("paths/output") == "recovered",
            "config save recovers after invalid input");
    const auto conflict = root / "not-a-directory";
    writeFixture(conflict, "original");
    ConfigFile unwritable(QString::fromStdWString((conflict / "paths.ini").wstring()));
    require(!unwritable.save({{"paths/output", "results", {}}}) && readText(conflict) == "original",
            "config storage failure preserves unrelated file");
}
void captures(const Path& root)
{
    CaptureSessionService session;
    const auto dir = session.start(root / "capture-tests");
    CaptureRequest off;
    off.category = CaptureCategory::LaserOff;
    CaptureRequest on;
    on.category = CaptureCategory::LaserOn;
    fails([&] { session.capture(frame(1), on); }, "laser on requires laser off");
    const auto pending = session.capture(frame(1), off);
    require(pending.pairPending && session.awaitingLaserOn() &&
                !std::filesystem::exists(dir / "off/000001.png"),
            "incomplete pair remains staged");
    fails([&] { session.start(root); }, "new session cannot discard pending pair");
    fails([&] { session.capture(frame(2), off); }, "pending pair category protected");
    fails([&] { session.capture(frame(1), on); }, "same frame pair rejected");
    fails([&] { session.capture(frame(2, "stream-B"), on); },
          "reconnected stream cannot complete previous pair");
    auto changed = frame(2);
    changed.metadata.requestedGain = 1;
    fails([&] { session.capture(changed, on); }, "changed gain pair rejected");
    const auto completed = session.capture(frame(2), on);
    require(!completed.pairPending && !session.awaitingLaserOn(), "complete pair returns to ready state");
    validateLaserPairManifest(dir / "off", dir / "on");
    fails([&] { validateLaserPairManifest(dir / "on", dir / "off"); }, "swapped pair directories rejected");
    const auto manifest = loadCaptureManifest(dir / "capture_manifest.yml");
    require(manifest.records.size() == 2 && manifest.records[1].frame.streamId == "stream-A" &&
                manifest.records[1].frame.frameNumber == 2,
            "capture metadata persisted");
    session.capture(frame(3), off);
    session.abandonPair();
    require(!session.awaitingLaserOn() && session.nextPairId() == 3 &&
                std::filesystem::exists(dir / "pending/pair_2/off.png"),
            "abandon retains staged audit image");
    bool failCommit = false;
    CaptureSessionService rollback(
        [&](const Path& path, const std::string& text)
        {
            if (failCommit)
                throw std::runtime_error("injected disk failure");
            writeFixture(path, text);
        });
    const auto rollbackDir = rollback.start(root / "rollback-tests");
    rollback.capture(frame(1), off);
    failCommit = true;
    fails([&] { rollback.capture(frame(2), on); }, "pair manifest storage failure surfaced");
    require(rollback.awaitingLaserOn() && std::filesystem::exists(rollbackDir / "pending/pair_1/off.png") &&
                !std::filesystem::exists(rollbackDir / "off/000001.png") &&
                !std::filesystem::exists(rollbackDir / "on/000001.png") &&
                loadCaptureManifest(rollbackDir / "capture_manifest.yml").records.empty(),
            "pair storage failure rolls back both published files");
    failCommit = false;
    rollback.capture(frame(3), on);
    validateLaserPairManifest(rollbackDir / "off", rollbackDir / "on");
    CaptureRequest camera;
    failCommit = true;
    fails([&] { rollback.capture(frame(4), camera); }, "single manifest storage failure surfaced");
    require(!std::filesystem::exists(rollbackDir / "camera/000001.png"),
            "failed single capture does not publish image");
    failCommit = false;
    rollback.capture(frame(5), camera);
    auto wrong = frame(6);
    wrong.metadata.deviceId = "other";
    fails([&] { rollback.capture(wrong, camera); }, "session device ownership protected");
    wrong = frame(6);
    wrong.metadata.lostPackets = 1;
    fails([&] { rollback.capture(wrong, camera); }, "packet loss capture rejected");
    wrong = frame(6);
    wrong.metadata.requestedExposureUs = std::numeric_limits<double>::infinity();
    fails([&] { rollback.capture(wrong, camera); }, "invalid frame metadata rejected");
}
void reconstructionAudit(const Path& root)
{
    CaptureSessionService session;
    const auto dir = session.start(root / "reconstruction-capture");
    CaptureRequest request;
    request.category = CaptureCategory::Reconstruction;
    request.geometry.mode = 1;
    request.geometry.axis = 2;
    request.geometry.stepMm = 2;
    request.manualPosition = 7;
    session.capture(frame(1), request);
    request.manualPosition.reset();
    session.capture(frame(2), request);
    ReconstructionConfig config = request.geometry;
    config.stepMm = 999;
    config.saveDiagnostics = false;
    std::vector<std::string> warnings;
    const auto plans = reconstructionFrames(dir / "reconstruction", config, warnings, {});
    require(plans.size() == 2 && plans[0].translationMm == 7 && plans[1].translationMm == 2 &&
                warnings.empty(),
            "recorded positions override current step");
    require(reconstructionFrames(dir / "reconstruction" / "", config, warnings, {}).size() == 2,
            "trailing directory separator preserves audit checks");
    config.stepMm = 0;
    Calibration calibration;
    calibration.camera.imageSize = {32, 24};
    calibration.camera.K = (cv::Mat_<double>(3, 3) << 100, 0, 16, 0, 100, 12, 0, 0, 1);
    calibration.camera.D = cv::Mat::zeros(1, 5, CV_64F);
    LaserPlane plane;
    plane.coefficients = {0, 0, 1, -100};
    calibration.planes.push_back(plane);
    const auto result =
        reconstructBatch(dir / "reconstruction", root / "audited-results", calibration, config, {});
    require(result.frames == 2 && result.points.size() == 48 && result.points[0].z == 107 &&
                result.points[24].z == 102,
            "actual reconstruction uses per-frame positions");
    const auto single = singleFrameWorkflow(plans[0].file, root / "audited-single", calibration, config, {});
    require(single.succeeded() && single.batch->points[0].z == 107 &&
                std::filesystem::exists(single.directory / "capture_manifest.yml"),
            "single reconstruction uses recorded manual position and retains audit");
    require(std::filesystem::exists(result.directory / "capture_manifest.yml") &&
                readText(result.directory / "frames.csv").find("translation_mm") != std::string::npos,
            "result retains frame audit source");
    config.axis = 0;
    require(!singleFrameWorkflow(plans[0].file, root / "invalid-single", calibration, config, {}).succeeded(),
            "single reconstruction rejects mismatched recorded scan axis");
    fails([&] { reconstructionFrames(dir / "reconstruction", config, warnings, {}); },
          "scan model axis mismatch rejected");
    config.axis = 2;
    std::filesystem::rename(dir / "reconstruction/000002.png", dir / "second.saved");
    fails([&] { reconstructionFrames(dir / "reconstruction", config, warnings, {}); },
          "missing recorded frame rejected");
    std::filesystem::rename(dir / "second.saved", dir / "reconstruction/000002.png");
    writeImage(dir / "reconstruction/extra.png", frame(3).image);
    fails([&] { reconstructionFrames(dir / "reconstruction", config, warnings, {}); },
          "unregistered input rejected");
    fails([&] { reconstructionFrames(dir / "on", config, warnings, {}); },
          "wrong manifest directory rejected");
    const auto legacy = root / "legacy-scan";
    writeImage(legacy / "scan_1.png", frame(1).image);
    writeImage(legacy / "scan_3.png", frame(3).image);
    config.stepMm = 2;
    fails([&] { reconstructionFrames(legacy, config, warnings, {}); }, "legacy numeric gap rejected");
    config.mode = 0;
    require(reconstructionFrames(legacy, config, warnings, {}).size() == 2,
            "fixed legacy images remain usable");
    TaskContext cancel;
    cancel.progress = [&](int percent, const std::string&)
    {
        if (percent >= 95)
            cancel.cancel->store(true);
    };
    const auto cancelled =
        reconstructionWorkflow(legacy, root / "cancelled-results", calibration, config, cancel);
    require(cancelled.isCancelled(), "reconstruction save cancellation distinguished");
    bool marker = false;
    for (const auto& item : std::filesystem::directory_iterator(root / "cancelled-results"))
        marker |= std::filesystem::exists(item.path() / "completed.txt");
    require(!marker, "cancelled reconstruction has no completion marker");
}
void pointClouds(const Path& root)
{
    Cloud cloud;
    for (int i = 0; i < 200000; ++i)
        cloud.emplace_back(i % 500, i / 500, 5);
    const auto original = std::make_shared<const Cloud>(std::move(cloud));
    PointCloudWorkspace workspace;
    const auto originalId = workspace.add("original", original);
    workspace.select({5, 3, 5});
    const auto snapshot = workspace.snapshot(true);
    workspace.select({7});
    require(snapshot.selection == std::vector<std::size_t>({3, 5}) && snapshot.points == original,
            "worker gets immutable original and selection snapshot");
    fails([&] { workspace.select({original->size()}); }, "out of range selection rejected");
    CloudRequest request;
    request.input = snapshot;
    request.operation = CloudOperation::ExtractSelection;
    auto extracted = pointCloudWorkflow(request, {});
    require(extracted.succeeded() && extracted.cloud->size() == 2 && extracted.sourceDocumentId == originalId,
            "selection derived result");
    const auto derivedId = workspace.add(extracted.documentName, extracted.cloud, extracted.sourceDocumentId);
    require(derivedId != originalId && workspace.documents().front().points->size() == 200000,
            "derived document preserves original");
    workspace.activate(originalId);
    request.input = workspace.snapshot(false);
    request.operation = CloudOperation::SelectProjected;
    request.rectangle = {0, -0.25, 10, 0.5};
    auto selected = pointCloudWorkflow(request, {});
    require(selected.succeeded() &&
                *selected.selection == std::vector<std::size_t>({0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10}),
            "projection selection includes unsampled raw points");
    request.operation = CloudOperation::Crop;
    request.axis = 0;
    request.minimum = 0;
    request.maximum = 1;
    auto cropped = pointCloudWorkflow(request, {});
    require(cropped.succeeded() && cropped.cloud->size() == 800 && original->size() == 200000,
            "crop produces independent cloud");
    request.operation = CloudOperation::Voxel;
    request.voxelMm = 2;
    TaskContext cancel;
    bool underway = false;
    cancel.progress = [&](int percent, const std::string&)
    {
        if (percent >= 5)
        {
            underway = true;
            cancel.cancel->store(true);
        }
    };
    require(pointCloudWorkflow(request, cancel).isCancelled() && underway,
            "voxel cancellation inside processing");
    request.operation = CloudOperation::PlaneFit;
    cancel = {};
    underway = false;
    cancel.progress = [&](int, const std::string& text)
    {
        if (text.find("RANSAC") != std::string::npos)
        {
            underway = true;
            cancel.cancel->store(true);
        }
    };
    require(pointCloudWorkflow(request, cancel).isCancelled() && underway,
            "plane fit cancellation inside processing");
    request.operation = CloudOperation::Export;
    request.exportPath = root / "preserved.xyz";
    writeFixture(request.exportPath, "original destination\n");
    cancel = {};
    underway = false;
    cancel.progress = [&](int percent, const std::string&)
    {
        if (percent >= 5)
        {
            underway = true;
            cancel.cancel->store(true);
        }
    };
    require(pointCloudWorkflow(request, cancel).isCancelled() && underway &&
                readText(request.exportPath) == "original destination\n",
            "cancelled export preserves destination");
    cancel = {};
    underway = false;
    cancel.progress = [&](int percent, const std::string& message)
    {
        if (percent >= 95 && message == "写入点云")
        {
            underway = true;
            cancel.cancel->store(true);
        }
    };
    require(pointCloudWorkflow(request, cancel).isCancelled() && underway &&
                readText(request.exportPath) == "original destination\n",
            "cancelled file write preserves destination");
    const auto input = root / "import.xyz";
    exportCloud(input, *original);
    cancel = {};
    underway = false;
    cancel.progress = [&](int percent, const std::string&)
    {
        if (percent >= 5)
        {
            underway = true;
            cancel.cancel->store(true);
        }
    };
    require(loadCloudWorkflow(input, cancel).isCancelled() && underway,
            "stream import cancellation inside reading");
    const auto success = pointCloudWorkflow(request, {});
    require(success.succeeded() && importCloud(request.exportPath).size() == original->size(),
            "atomic export round trip after cancellation");
    request.operation = CloudOperation::SelectProjected;
    request.projection.scale = std::numeric_limits<double>::quiet_NaN();
    require(!pointCloudWorkflow(request, {}).succeeded(), "invalid projection reported as failure");
}
void runners()
{
    TaskRunner runner;
    int finished = 0;
    JobResult result;
    std::atomic_bool entered{false};
    bool guiProgress = false;
    QObject::connect(&runner, &TaskRunner::finished, &runner,
                     [&](JobResult value)
                     {
                         result = std::move(value);
                         ++finished;
                     });
    QObject::connect(&runner, &TaskRunner::progress, &runner, [&](int, const QString&)
                     { guiProgress = QThread::currentThread() == QCoreApplication::instance()->thread(); });
    require(runner.start(JobKind::CloudDerived,
                         [&](const TaskContext& task)
                         {
                             return executeJob(JobKind::CloudDerived, task,
                                               [&](JobResult&, const TaskContext& current)
                                               {
                                                   entered.store(true);
                                                   while (true)
                                                   {
                                                       current.report(20, "running");
                                                       QThread::msleep(1);
                                                   }
                                               });
                         }),
            "runner starts work");
    require(!runner.start(JobKind::CloudFit, [](const TaskContext&) { return JobResult{}; }),
            "runner refuses overlapping jobs");
    waitUntil([&] { return entered.load() && guiProgress; });
    runner.cancel();
    waitUntil([&] { return finished == 1; });
    require(result.isCancelled() && !runner.isRunning(), "runner cancellation releases busy state");
    require(runner.start(JobKind::CloudFit, [](const TaskContext&) -> JobResult { throw 42; }),
            "runner reusable after cancellation");
    waitUntil([&] { return finished == 2; });
    require(!result.succeeded() && !result.isCancelled() && result.kind == JobKind::CloudFit &&
                !result.message.empty(),
            "unknown worker exception contained");
    require(runner.start(JobKind::CloudLoad, [](const TaskContext&) -> JobResult
                         { throw std::runtime_error("expected exception"); }),
            "runner starts after exception");
    waitUntil([&] { return finished == 3; });
    require(result.message == "expected exception", "worker exception message preserved");
    require(runner.start(JobKind::CloudExport,
                         [](const TaskContext& task)
                         {
                             return executeJob(JobKind::CloudExport, task,
                                               [](JobResult& value, const TaskContext&)
                                               { value.message = "ok"; });
                         }),
            "runner reusable after failure");
    waitUntil([&] { return finished == 4; });
    require(result.succeeded(), "subsequent job succeeds");
    std::atomic_bool destroyedJobEntered{false}, stopped{false};
    {
        TaskRunner closing;
        closing.start(JobKind::CloudDerived,
                      [&](const TaskContext& task)
                      {
                          auto value = executeJob(JobKind::CloudDerived, task,
                                                  [&](JobResult&, const TaskContext& current)
                                                  {
                                                      destroyedJobEntered = true;
                                                      while (true)
                                                      {
                                                          current.report(10, "closing");
                                                          QThread::msleep(1);
                                                      }
                                                  });
                          stopped = true;
                          return value;
                      });
        waitUntil([&] { return destroyedJobEntered.load(); });
    }
    QCoreApplication::processEvents();
    require(stopped, "runner destruction cancels and joins worker safely");
}
} // namespace
int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    try
    {
        QTemporaryDir temporary;
        require(temporary.isValid(), "temporary workspace");
        const Path root(temporary.path().toStdWString());
        configurations(root);
        captures(root);
        reconstructionAudit(root);
        pointClouds(root);
        runners();
        std::cout << "PASS: " << checks
                  << " stability assertions (config, capture rollback, frame audit, cloud cancellation, "
                     "worker lifecycle)\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
