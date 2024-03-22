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

#ifndef ODRONES_MAINWINDOW_H
#define ODRONES_MAINWINDOW_H

#include <QMainWindow>
#include <Qt3DCore/Qt3DCore>
#include <Qt3DExtras/Qt3DExtras>

#include "rns.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

namespace odrones
{

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(std::string iFile, bool identifyLanes, QWidget *parent = nullptr);
    ~MainWindow();

    void setup();


private:
    Ui::MainWindow *ui;
    Qt3DExtras::Qt3DWindow *view;
    Qt3DExtras::QFirstPersonCameraController *cameraController;
    Qt3DCore::QEntity *root;

    RNS *_rns;

};

} // namespace odrones;
#endif // ODRONES_MAINWINDOW_H
