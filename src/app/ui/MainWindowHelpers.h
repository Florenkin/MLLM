#pragma once
#include "core/Types.h"
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>
namespace mllm::ui_detail
{
inline QString q(const std::string& value)
{
    return QString::fromUtf8(value.data(), static_cast<int>(value.size()));
}
inline Path path(const QString& value)
{
    return Path(value.toStdWString());
}
inline QString display(const Path& value)
{
    return QString::fromStdWString(value.wstring());
}
inline QScrollArea* wrapScroll(QWidget* widget)
{
    auto* area = new QScrollArea;
    area->setWidgetResizable(true);
    area->setWidget(widget);
    area->setFrameShape(QFrame::NoFrame);
    return area;
}
inline QPushButton* button(QVBoxLayout* layout, const QString& text, const QString& name)
{
    auto* value = new QPushButton(text);
    value->setObjectName(name);
    layout->addWidget(value);
    return value;
}
} // namespace mllm::ui_detail
