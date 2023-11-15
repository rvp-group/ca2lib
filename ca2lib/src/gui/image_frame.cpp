// clang-format off

// Copyright (c) 2023, S(apienza) R(obust) R(obotics) G(roup)

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// clang-format on

#include "ca2lib/gui/image_frame.h"
#include <QMouseEvent>
#include <QPainter>
#include <QImage>
#include <iostream>
#include <QHBoxLayout>

namespace ca2lib {

ImageFrame::ImageFrame(QWidget* parent, Qt::WindowFlags flags)
    : QWidget(parent, flags),
      _aspect_ratio(4, 3),
      _outer_layout(nullptr),
      _image(),
      _mutex(),
      _roi(nullptr),
      _roi_origin() {
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setMinimumSize(80, 60);
  connect(this, SIGNAL(delayed_update()), this, SLOT(update()),
          Qt::QueuedConnection);
}

void ImageFrame::setOuterLayout(QHBoxLayout* outer_layout) {
  _outer_layout = outer_layout;
}

void ImageFrame::setImage(QImage image_) {
  _mutex.lock();
  _image = image_.copy();
  setAspectRatio(_image.width(), _image.height());
  _mutex.unlock();
  emit delayed_update();
}

void ImageFrame::paintEvent(QPaintEvent*) {
  QPainter painter(this);
  _mutex.lock();
  if (!_image.isNull()) {
    resizeToFitAspectRatio();
    QImage image =
        _image.scaled(contentsRect().width(), contentsRect().height(),
                      Qt::KeepAspectRatio, Qt::SmoothTransformation);
    painter.drawImage(contentsRect(), image);
  }
  _mutex.unlock();
}

void ImageFrame::mousePressEvent(QMouseEvent* e_) {
  if (_enable_selection) {
    _roi_origin = e_->pos();
    if (!_roi) _roi = new QRubberBand(QRubberBand::Rectangle, this);
    _roi->setGeometry(QRect(_roi_origin, QSize()));
    _roi->show();
    emit delayed_update();
  }
}

void ImageFrame::mouseMoveEvent(QMouseEvent* e_) {
  if (_enable_selection) {
    _roi->setGeometry(QRect(_roi_origin, e_->pos()).normalized());
    emit delayed_update();
  }
}
void ImageFrame::mouseReleaseEvent(QMouseEvent* e_) {
  if (_enable_selection) {
    _roi->hide();
    int x1, x2, y1, y2;
    _roi->geometry().getCoords(&x1, &y1, &x2, &y2);
    emit delayed_update();
  }
}

void ImageFrame::resizeToFitAspectRatio() {
  QRect rect = contentsRect();

  // reduce longer edge to aspect ration
  double width;
  double height;

  if (_outer_layout) {
    width = _outer_layout->contentsRect().width();
    height = _outer_layout->contentsRect().height();
  } else {
    // if outer layout isn't available, this will use the old
    // width and height, but this can shrink the display image if the
    // aspect ratio changes.
    width = rect.width();
    height = rect.height();
  }

  double layout_ar = width / height;
  const double image_ar =
      double(_aspect_ratio.width()) / double(_aspect_ratio.height());
  if (layout_ar > image_ar) {
    // too large width
    width = height * image_ar;
  } else {
    // too large height
    height = width / image_ar;
  }
  rect.setWidth(int(width + 0.5));
  rect.setHeight(int(height + 0.5));

  // resize taking the border line into account
  resize(rect.width(), rect.height());
}

void ImageFrame::setInnerFrameMinimumSize(const QSize& size) {
  QSize new_size = size;
  setMinimumSize(new_size);
  emit delayed_update();
}

void ImageFrame::setInnerFrameMaximumSize(const QSize& size) {
  QSize new_size = size;
  setMaximumSize(new_size);
  emit delayed_update();
}

void ImageFrame::setInnerFrameFixedSize(const QSize& size) {
  setInnerFrameMinimumSize(size);
  setInnerFrameMaximumSize(size);
}

void ImageFrame::setAspectRatio(unsigned short width, unsigned short height) {
  int divisor = greatestCommonDivisor(width, height);
  if (divisor != 0) {
    _aspect_ratio.setWidth(width / divisor);
    _aspect_ratio.setHeight(height / divisor);
  }
}

int ImageFrame::greatestCommonDivisor(int a, int b) {
  if (b == 0) {
    return a;
  }
  return greatestCommonDivisor(b, a % b);
}

}  // namespace ca2lib