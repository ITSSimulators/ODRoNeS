// 
//  This file is part of the ODRoNeS (OpenDRIVE Road Network System) package.
//  
//  Copyright (c) 2023 Albert Solernou, University of Leeds.
// 
//  GTSmartActors is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
// 
//  GTSmartActors is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
// 
//  You should have received a copy of the GNU General Public License
//  along with ODRoNeS. If not, see <http://www.gnu.org/licenses/>.
// 
//  We would appreciate that if you use this software for work leading 
//  to publications you cite the package and its related publications. 
//

#ifndef RNSWINDOW_H
#define RNSWINDOW_H

#ifdef QT_CORE_LIB

#include <QMainWindow>
#include <QGraphicsScene>
#include "rns.h"
#include "graphicalrns.h"
#include "graphicalZoom.h"

namespace Ui {
class RNSWindow;
}

class RNSWindow : public QMainWindow
{
    Q_OBJECT

public:
    // explicit RNSWindow(std::string iFile, bool identifyLanes, QWidget *parent = nullptr);
    explicit RNSWindow(RNS *rns, bool identifyLanes, QWidget *parent = nullptr);
    ~RNSWindow();

private:
    Ui::RNSWindow *ui;

    // RNS *_rns;
    graphicalRNS *_grns;
    GraphicalZoom *_gzoom;
    QGraphicsScene *_scene;
};

#endif // QT_CORE_LIB

#endif // RNSWINDOW_H
