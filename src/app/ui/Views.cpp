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
CloudView::CloudView(QWidget* parent) : QWidget(parent)
{
    setMinimumSize(300, 240);
}
void CloudView::setCloud(std::shared_ptr<const Cloud> cloud, std::vector<std::size_t> selection)
{
    selection_ = std::move(selection);
    std::sort(selection_.begin(), selection_.end());
    if (cloud_ == cloud)
    {
        update();
        return;
    }
    cloud_ = std::move(cloud);
    displayedIndices_.clear();
    if (cloud_ && !cloud_->empty())
    {
        const std::size_t stride = std::max<std::size_t>(1, (cloud_->size() + 149999) / 150000);
        for (std::size_t i = 0; i < cloud_->size(); i += stride)
            displayedIndices_.push_back(i);
        const auto bounds = cloudBounds(*cloud_);
        center_ = (bounds.first + bounds.second) * 0.5;
        extent_ = std::max({bounds.second.x - bounds.first.x, bounds.second.y - bounds.first.y,
                            bounds.second.z - bounds.first.z, 1.0});
    }
    reset();
}
void CloudView::setSelection(std::vector<std::size_t> selection)
{
    selection_ = std::move(selection);
    std::sort(selection_.begin(), selection_.end());
    update();
}
void CloudView::setSelectionMode(bool enabled)
{
    selectionMode_ = enabled;
    selecting_ = false;
    setCursor(enabled ? Qt::CrossCursor : Qt::ArrowCursor);
    update();
}
CloudProjection CloudView::projection() const
{
    return {center_,
            {std::cos(yaw_), 0, std::sin(yaw_)},
            {std::sin(pitch_) * std::sin(yaw_), std::cos(pitch_), -std::sin(pitch_) * std::cos(yaw_)},
            std::min(width(), height()) * 0.75 * zoom_ / extent_,
            {width() / 2.0 + pan_.x(), height() / 2.0 + pan_.y()}};
}
void CloudView::reset()
{
    yaw_ = -0.4;
    pitch_ = 0.4;
    zoom_ = 1;
    pan_ = {};
    selecting_ = false;
    update();
}
void CloudView::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    painter.fillRect(rect(), QColor("#242a30"));
    if (!cloud_ || cloud_->empty())
    {
        painter.setPen(Qt::lightGray);
        painter.drawText(rect(), Qt::AlignCenter, "无点云\n左键旋转 · 右键平移 · 滚轮缩放");
        return;
    }
    const auto transform = projection();
    const auto project = [&](const cv::Point3d& point)
    {
        const auto p = projectCloudPoint(point, transform);
        return QPointF(p.x, p.y);
    };
    double minimum = cloud_->front().z, maximum = minimum;
    for (auto index : displayedIndices_)
    {
        minimum = std::min(minimum, (*cloud_)[index].z);
        maximum = std::max(maximum, (*cloud_)[index].z);
    }
    for (auto index : displayedIndices_)
    {
        const auto& point = (*cloud_)[index];
        const bool selected = std::binary_search(selection_.begin(), selection_.end(), index);
        painter.setPen(
            selected ? QColor("#ffcc33")
                     : QColor::fromHsvF((1 - (point.z - minimum) / std::max(maximum - minimum, 1e-9)) * 0.65,
                                        0.8, 0.95));
        painter.drawPoint(project(point));
    }
    const cv::Point3d vectors[] = {{extent_ * 0.2, 0, 0}, {0, extent_ * 0.2, 0}, {0, 0, extent_ * 0.2}};
    const QColor colors[] = {Qt::red, Qt::green, QColor("#4488ff")};
    const QString names[] = {"X", "Y", "Z"};
    for (int i = 0; i < 3; ++i)
    {
        painter.setPen(QPen(colors[i], 2));
        painter.drawLine(project(center_), project(center_ + vectors[i]));
        painter.drawText(project(center_ + vectors[i]), names[i]);
    }
    painter.setPen(Qt::lightGray);
    painter.drawText(12, 22,
                     QString("显示 %1 / %2 点 · 选中 %3 点")
                         .arg(displayedIndices_.size())
                         .arg(cloud_->size())
                         .arg(selection_.size()));
    if (selecting_)
    {
        painter.setPen(QPen(QColor("#ffcc33"), 1, Qt::DashLine));
        painter.setBrush(QColor(255, 204, 51, 30));
        painter.drawRect(QRect(selectionStart_, selectionEnd_).normalized());
    }
}
void CloudView::wheelEvent(QWheelEvent* event)
{
    if (selecting_)
        return;
    zoom_ = std::clamp(zoom_ * (event->angleDelta().y() > 0 ? 1.15 : 1 / 1.15), 0.05, 50.0);
    update();
}
void CloudView::mousePressEvent(QMouseEvent* event)
{
    last_ = event->pos();
    if (selectionMode_ && event->button() == Qt::LeftButton)
    {
        selecting_ = true;
        selectionStart_ = selectionEnd_ = event->pos();
        update();
    }
}
void CloudView::mouseMoveEvent(QMouseEvent* event)
{
    if (selecting_)
    {
        selectionEnd_ = event->pos();
        update();
        return;
    }
    const auto delta = event->pos() - last_;
    last_ = event->pos();
    if (event->buttons() & Qt::LeftButton)
    {
        yaw_ += delta.x() * 0.01;
        pitch_ += delta.y() * 0.01;
    }
    if (event->buttons() & Qt::RightButton)
        pan_ += delta;
    update();
}
void CloudView::mouseReleaseEvent(QMouseEvent* event)
{
    if (!selecting_ || event->button() != Qt::LeftButton)
        return;
    selectionEnd_ = event->pos();
    selecting_ = false;
    const auto rectangle = QRectF(selectionStart_, selectionEnd_).normalized();
    if (rectangle.width() > 2 && rectangle.height() > 2)
        emit selectionRequested(projection(),
                                {rectangle.x(), rectangle.y(), rectangle.width(), rectangle.height()});
    update();
}
void CloudView::mouseDoubleClickEvent(QMouseEvent*)
{
    if (!selectionMode_)
        reset();
}
} // namespace mllm
