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



#ifndef ODRONES_RNSWINDOW_H
#define ODRONES_RNSWINDOW_H

#ifdef QT_CORE_LIB

#include <QMainWindow>
#include <QGraphicsScene>
#include "rns.h"
#include "graphicalrns.h"
#include "graphicalZoom.h"

namespace Ui {
class RNSWindow;
}

namespace odrones
{

class RNSWindow : public QMainWindow
{
    Q_OBJECT

public:
    // explicit RNSWindow(std::string iFile, bool identifyLanes, QWidget *parent = nullptr);
    explicit RNSWindow(RNS *rns, const graphicalSettings &gSettings, QWidget *parent = nullptr);
    ~RNSWindow();

private:
    Ui::RNSWindow *ui;

    // RNS *_rns;
    graphicalRNS *_grns;
    GraphicalZoom *_gzoom;
    QGraphicsScene *_scene;
};

}
#endif // QT_CORE_LIB

#endif // ODRONES_RNSWINDOW_H
