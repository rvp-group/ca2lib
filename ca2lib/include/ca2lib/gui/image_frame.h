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

#pragma once

#include <QMutex>
#include <QWidget>
#include <QImage>
#include <QRubberBand>
#include <QHBoxLayout>

namespace ca2lib {

class ImageFrame : public QWidget {
  Q_OBJECT

 public:
  ImageFrame(QWidget* parent = nullptr, Qt::WindowFlags flags = 0);

  void resizeToFitAspectRatio();

  void setInnerFrameMinimumSize(const QSize& size);

  void setInnerFrameMaximumSize(const QSize& size);

  void setInnerFrameFixedSize(const QSize& size);

  inline void enableSelection(bool flag_) { _enable_selection = flag_; }

  void setOuterLayout(QHBoxLayout* outer_layout);

 public slots:
  void setImage(QImage);

 signals:
  void imageChanged(QImage);

  void delayed_update();

 protected:
  void paintEvent(QPaintEvent* event);
  void mousePressEvent(QMouseEvent*);
  void mouseMoveEvent(QMouseEvent*);
  void mouseReleaseEvent(QMouseEvent*);
  void setAspectRatio(unsigned short width, unsigned short height);

 private:
  static int greatestCommonDivisor(int a, int b);
  QSize _aspect_ratio;

  QHBoxLayout* _outer_layout;

  QImage _image;
  QMutex _mutex;

  // Drawing rectangle stuff
  QRubberBand* _roi;
  QPoint _roi_origin;

  bool _enable_selection = false;
};
}  // namespace ca2lib