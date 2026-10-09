#include "app/ui/Views.h"
#include "core/PointCloud.h"
#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>

namespace mllm
{
ImageView::ImageView(QWidget* p) : QWidget(p)
{
    setMinimumSize(300, 240);
}
void ImageView::setImage(const cv::Mat& image)
{
    if (image.empty())
    {
        image_ = {};
        update();
        return;
    }
    cv::Mat rgb;
    if (image.channels() == 1)
        cv::cvtColor(image, rgb, cv::COLOR_GRAY2RGB);
    else
        cv::cvtColor(image, rgb, cv::COLOR_BGR2RGB);
    const bool changed = image_.size() != QSize(image.cols, image.rows);
    image_ = QImage(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step), QImage::Format_RGB888).copy();
    if (changed)
        fit();
    else
        update();
}
void ImageView::fit()
{
    offset_ = {};
    if (!image_.isNull())
        scale_ = std::min(static_cast<double>(width()) / image_.width(),
                          static_cast<double>(height()) / image_.height());
    update();
}
void ImageView::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.fillRect(rect(), QColor("#f5f5f5"));
    if (image_.isNull())
    {
        p.setPen(QColor("#777777"));
        p.drawText(rect(), Qt::AlignCenter, "无图像\n滚轮缩放 · 左键平移 · 双击适应窗口");
        return;
    }
    const QSizeF s(image_.width() * scale_, image_.height() * scale_);
    p.setRenderHint(QPainter::SmoothPixmapTransform);
    p.drawImage(
        QRectF(QPointF(width() / 2.0, height() / 2.0) + offset_ - QPointF(s.width() / 2, s.height() / 2), s),
        image_);
}
void ImageView::resizeEvent(QResizeEvent*)
{
    fit();
}
void ImageView::wheelEvent(QWheelEvent* e)
{
    if (image_.isNull())
        return;
    const double old = scale_;
    scale_ = std::clamp(scale_ * (e->angleDelta().y() > 0 ? 1.15 : 1 / 1.15), 0.01, 30.0);
    const QPointF delta = e->position() - QPointF(width() / 2.0, height() / 2.0);
    offset_ = delta - (delta - offset_) * scale_ / old;
    update();
}
void ImageView::mousePressEvent(QMouseEvent* e)
{
    last_ = e->pos();
}
void ImageView::mouseMoveEvent(QMouseEvent* e)
{
    if (e->buttons() & Qt::LeftButton)
    {
        offset_ += e->pos() - last_;
        last_ = e->pos();
        update();
    }
}
void ImageView::mouseDoubleClickEvent(QMouseEvent*)
{
    fit();
}
CloudView::CloudView(QWidget* p) : QWidget(p)
{
    setMinimumSize(300, 240);
}
void CloudView::setCloud(const Cloud& cloud)
{
    points_.clear();
    const std::size_t stride = std::max<std::size_t>(1, (cloud.size() + 149999) / 150000);
    for (std::size_t i = 0; i < cloud.size(); i += stride)
        points_.push_back(cloud[i]);
    if (!cloud.empty())
    {
        const auto [lo, hi] = cloudBounds(cloud);
        center_ = (lo + hi) * 0.5;
        extent_ = std::max({hi.x - lo.x, hi.y - lo.y, hi.z - lo.z, 1.0});
    }
    reset();
}
void CloudView::reset()
{
    yaw_ = -0.4;
    pitch_ = 0.4;
    zoom_ = 1;
    pan_ = {};
    update();
}
void CloudView::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.fillRect(rect(), QColor("#242a30"));
    if (points_.empty())
    {
        p.setPen(Qt::lightGray);
        p.drawText(rect(), Qt::AlignCenter, "无点云\n左键旋转 · 右键平移 · 滚轮缩放 · 双击复位");
        return;
    }
    const double scale = std::min(width(), height()) * 0.75 * zoom_ / extent_;
    const auto project = [&](cv::Point3d v)
    {
        const double x = std::cos(yaw_) * v.x + std::sin(yaw_) * v.z,
                     z = -std::sin(yaw_) * v.x + std::cos(yaw_) * v.z;
        const double y = std::cos(pitch_) * v.y - std::sin(pitch_) * z;
        return QPointF(width() / 2.0 + x * scale, height() / 2.0 - y * scale) + pan_;
    };
    double minZ = points_[0].z, maxZ = minZ;
    for (const auto& v : points_)
    {
        minZ = std::min(minZ, v.z);
        maxZ = std::max(maxZ, v.z);
    }
    for (const auto& point : points_)
    {
        const double ratio = (point.z - minZ) / std::max(maxZ - minZ, 1e-9);
        p.setPen(QColor::fromHsvF((1 - ratio) * 0.65, 0.8, 0.95));
        p.drawPoint(project(point - center_));
    }
    const auto origin = project({0, 0, 0});
    const double l = extent_ * 0.2;
    const cv::Point3d vectors[] = {{l, 0, 0}, {0, l, 0}, {0, 0, l}};
    const QColor colors[] = {Qt::red, Qt::green, QColor("#4488ff")};
    const QString names[] = {"X", "Y", "Z"};
    for (int i = 0; i < 3; ++i)
    {
        p.setPen(QPen(colors[i], 2));
        p.drawLine(origin, project(vectors[i]));
        p.drawText(project(vectors[i]), names[i]);
    }
    p.setPen(Qt::lightGray);
    p.drawText(
        12, 22,
        QString("显示 %1 点 · 深度 %2–%3 mm").arg(points_.size()).arg(minZ, 0, 'f', 2).arg(maxZ, 0, 'f', 2));
}
void CloudView::wheelEvent(QWheelEvent* e)
{
    zoom_ = std::clamp(zoom_ * (e->angleDelta().y() > 0 ? 1.15 : 1 / 1.15), 0.05, 50.0);
    update();
}
void CloudView::mousePressEvent(QMouseEvent* e)
{
    last_ = e->pos();
}
void CloudView::mouseMoveEvent(QMouseEvent* e)
{
    const QPoint delta = e->pos() - last_;
    last_ = e->pos();
    if (e->buttons() & Qt::LeftButton)
    {
        yaw_ += delta.x() * 0.01;
        pitch_ += delta.y() * 0.01;
    }
    if (e->buttons() & Qt::RightButton)
        pan_ += delta;
    update();
}
void CloudView::mouseDoubleClickEvent(QMouseEvent*)
{
    reset();
}
} // namespace mllm
