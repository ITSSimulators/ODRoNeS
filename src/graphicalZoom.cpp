//
//   This file is part of ODRoNeS (OpenDRIVE Road Network System).
//
//   Copyright (c) 2019-2026 Albert Solernou, University of Leeds.
//
//   The ODRoNeS package is free software; you can redistribute it and/or
//   modify it under the terms of the GNU Lesser General Public
//   License as published by the Free Software Foundation; either
//   version 3 of the License, or (at your option) any later version.
//
//   The ODRoNeS package is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
//   Lesser General Public License for more details.
//
//   You should have received a copy of the GNU Lesser General Public
//   License along with the ODRoNeS package; if not, see
//   <https://www.gnu.org/licenses/>.
//



#include "graphicalZoom.h"

#ifdef QT_CORE_LIB

#include <QMouseEvent>
#include <QApplication>
#include <QScrollBar>
#include <qmath.h>

GraphicalZoom::GraphicalZoom(QGraphicsView* view)
  : QObject(view), _view(view)
{
  _view->viewport()->installEventFilter(this);
  _view->setMouseTracking(true);
  _modifiers = Qt::ControlModifier;
  _zoom_factor_base = 1.0015;
}

void GraphicalZoom::gentle_zoom(double factor) {
  _view->scale(factor, factor);
  _view->centerOn(target_scene_pos);

  // That's the bit that solves things out!!
  QRectF rect(target_scene_pos.x() - _view->viewport()->width() / 2.0,
              target_scene_pos.y() - _view->viewport()->height() / 2.0,
              _view->viewport()->width(), _view->viewport()->height());
  _view->setSceneRect(rect);

  // The following until zoomed() won't do anything useful...
  QPointF delta_viewport_pos = target_viewport_pos - QPointF(_view->viewport()->width() / 2.0,
                                                             _view->viewport()->height() / 2.0);
  QPointF viewport_center = _view->mapFromScene(target_scene_pos) - delta_viewport_pos;
  _view->centerOn(_view->mapToScene(viewport_center.toPoint()));
}

void GraphicalZoom::set_modifiers(Qt::KeyboardModifiers modifiers) {
  _modifiers = modifiers;

}

void GraphicalZoom::set_zoom_factor_base(double value) {
  _zoom_factor_base = value;
}

bool GraphicalZoom::eventFilter(QObject *object, QEvent *event) {
  if (event->type() == QEvent::MouseMove) {
    QMouseEvent* mouse_event = static_cast<QMouseEvent*>(event);
    QPointF delta = target_viewport_pos - mouse_event->pos();
    if (qAbs(delta.x()) > 5 || qAbs(delta.y()) > 5) {
      target_viewport_pos = mouse_event->pos();
      target_scene_pos = _view->mapToScene(mouse_event->pos());
    }
  } else if (event->type() == QEvent::Wheel) {
    QWheelEvent* wheel_event = static_cast<QWheelEvent*>(event);
    if (QApplication::keyboardModifiers() == _modifiers) {
      double angle = wheel_event->angleDelta().y();
      if (angle != 0) {
        double factor = qPow(_zoom_factor_base, angle);
        gentle_zoom(factor);
        return true;
      }
    }
  }
  Q_UNUSED(object)
  return false;
}

#endif // QT_CORE_LIB
