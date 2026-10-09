#include "app/acquisition/Camera.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#ifdef MLLM_WITH_HIK
#include <MvCameraControl.h>
#endif
#include <cmath>
#include <stdexcept>

namespace mllm
{
#ifdef MLLM_WITH_HIK
namespace
{
QString hikSerial(const MV_CC_DEVICE_INFO& info)
{
    if (info.nTLayerType == MV_GIGE_DEVICE)
        return QString::fromLatin1(reinterpret_cast<const char*>(info.SpecialInfo.stGigEInfo.chSerialNumber));
    return QString::fromLatin1(reinterpret_cast<const char*>(info.SpecialInfo.stUsb3VInfo.chSerialNumber));
}
void sdkCheck(int code, const char* action)
{
    if (code != 0)
        throw std::runtime_error(std::string(action) + ": 0x" +
                                 QString::number(static_cast<unsigned int>(code), 16).toStdString());
}
struct HikHandle
{
    void* handle = nullptr;
    bool open = false, grabbing = false;
    ~HikHandle()
    {
        if (handle)
        {
            if (grabbing)
                MV_CC_StopGrabbing(handle);
            if (open)
                MV_CC_CloseDevice(handle);
            MV_CC_DestroyHandle(handle);
        }
    }
};
} // namespace
#endif
std::vector<CameraInfo> cameraDevices()
{
    std::vector<CameraInfo> devices{{"demo", "模拟线激光（验证软件）"}};
#ifdef MLLM_WITH_HIK
    MV_CC_DEVICE_INFO_LIST list{};
    sdkCheck(MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &list), "枚举海康相机");
    for (unsigned int i = 0; i < list.nDeviceNum; ++i)
    {
        const auto serial = hikSerial(*list.pDeviceInfo[i]);
        devices.push_back({"hik:" + serial, "海康 " + serial});
    }
#endif
    // Generic indices are offered explicitly; no auto-open or slow probing.
    for (int i = 0; i < 4; ++i)
        devices.push_back({"usb:" + QString::number(i), "USB / DirectShow 相机 " + QString::number(i)});
    return devices;
}
CameraThread::CameraThread(QString id, double exposure, double gain, QObject* parent)
    : QThread(parent), id_(id), exposure_(exposure), gain_(gain)
{
}
CameraThread::~CameraThread()
{
    stop();
    wait();
}
void CameraThread::stop()
{
    stopping_.store(true);
}
void CameraThread::run()
{
    try
    {
        if (id_ == "demo")
        {
            emit opened("模拟线激光");
            int count = 0;
            while (!stopping_.load())
            {
                cv::Mat frame(480, 640, CV_8UC1, cv::Scalar(8));
                for (int y = 0; y < frame.rows; ++y)
                {
                    const double cx = 320 + 35 * std::sin(y / 60.0 + count / 35.0);
                    for (int x = 0; x < frame.cols; ++x)
                        frame.at<uchar>(y, x) =
                            static_cast<uchar>(8 + 235 * std::exp(-std::pow(x - cx, 2) / 8));
                }
                emit frameReady(frame);
                ++count;
                msleep(50);
            }
            return;
        }
#ifdef MLLM_WITH_HIK
        if (id_.startsWith("hik:"))
        {
            MV_CC_DEVICE_INFO_LIST list{};
            sdkCheck(MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &list), "枚举设备");
            MV_CC_DEVICE_INFO* device = nullptr;
            for (unsigned int i = 0; i < list.nDeviceNum; ++i)
                if (hikSerial(*list.pDeviceInfo[i]) == id_.mid(4))
                    device = list.pDeviceInfo[i];
            if (!device)
                throw std::runtime_error("所选海康相机已断开");
            HikHandle h;
            sdkCheck(MV_CC_CreateHandle(&h.handle, device), "创建设备句柄");
            sdkCheck(MV_CC_OpenDevice(h.handle), "连接设备");
            h.open = true;
            if (device->nTLayerType == MV_GIGE_DEVICE)
            {
                const int packet = MV_CC_GetOptimalPacketSize(h.handle);
                if (packet > 0)
                    sdkCheck(MV_CC_SetIntValue(h.handle, "GevSCPSPacketSize", packet), "设置 GigE 包大小");
            }
            sdkCheck(MV_CC_SetEnumValue(h.handle, "TriggerMode", 0), "设置自由取流");
            sdkCheck(MV_CC_SetEnumValue(h.handle, "ExposureAuto", 0), "关闭自动曝光");
            sdkCheck(MV_CC_SetFloatValue(h.handle, "ExposureTime", static_cast<float>(exposure_)),
                     "设置曝光");
            sdkCheck(MV_CC_SetEnumValue(h.handle, "GainAuto", 0), "关闭自动增益");
            sdkCheck(MV_CC_SetFloatValue(h.handle, "Gain", static_cast<float>(gain_)), "设置增益");
            sdkCheck(MV_CC_StartGrabbing(h.handle), "启动取流");
            h.grabbing = true;
            emit opened(id_);
            int timeouts = 0;
            while (!stopping_.load())
            {
                MV_FRAME_OUT buffer{};
                const int code = MV_CC_GetImageBuffer(h.handle, &buffer, 150);
                if (code != 0)
                {
                    if (++timeouts > 20)
                        sdkCheck(code, "连续取流超时或断线");
                    continue;
                }
                timeouts = 0;
                cv::Mat frame;
                try
                {
                    const auto& info = buffer.stFrameInfo;
                    if (info.enPixelType == PixelType_Gvsp_Mono8)
                        frame = cv::Mat(info.nHeight, info.nWidth, CV_8UC1, buffer.pBufAddr).clone();
                    else
                    {
                        frame = cv::Mat(info.nHeight, info.nWidth, CV_8UC3);
                        MV_CC_PIXEL_CONVERT_PARAM param{};
                        param.nWidth = info.nWidth;
                        param.nHeight = info.nHeight;
                        param.pSrcData = buffer.pBufAddr;
                        param.nSrcDataLen = info.nFrameLen;
                        param.enSrcPixelType = info.enPixelType;
                        param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
                        param.pDstBuffer = frame.data;
                        param.nDstBufferSize = static_cast<unsigned int>(frame.total() * 3);
                        sdkCheck(MV_CC_ConvertPixelType(h.handle, &param), "像素转换");
                    }
                }
                catch (...)
                {
                    MV_CC_FreeImageBuffer(h.handle, &buffer);
                    throw;
                }
                MV_CC_FreeImageBuffer(h.handle, &buffer);
                emit frameReady(frame);
                msleep(25); // bounded UI delivery rate, independent of camera FPS.
            }
            return;
        }
#endif
        if (!id_.startsWith("usb:"))
            throw std::runtime_error("未知相机类型");
        cv::VideoCapture capture(id_.mid(4).toInt(), cv::CAP_DSHOW);
        if (!capture.isOpened())
            throw std::runtime_error("无法连接所选 USB 相机");
        emit opened(id_);
        while (!stopping_.load())
        {
            cv::Mat frame;
            if (!capture.read(frame) || frame.empty())
                throw std::runtime_error("USB 相机取流失败或设备断开");
            emit frameReady(frame);
            msleep(40);
        }
    }
    catch (const std::exception& error)
    {
        emit errorOccurred(QString::fromUtf8(error.what()));
    }
}
} // namespace mllm
