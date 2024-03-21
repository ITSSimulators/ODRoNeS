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

#ifdef QT_CORE_LIB

#include "rnswindow.h"
#include "ui_rnswindow.h"

// RNSWindow::RNSWindow(std::string iFile, bool identifyLanes, QWidget *parent) :
RNSWindow::RNSWindow(RNS *rns, bool identifyLanes, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::RNSWindow),
    _grns(nullptr),
    _gzoom(nullptr),
    _scene(nullptr)
{
    ui->setupUi(this);
    setWindowTitle(tr("ODRoNeS"));
    ui->centralwidget->setLayout(ui->verticalLayout);

    // rns = new RNS(iFile, concepts::drivingSide::leftHand, true);
    if (!rns->ready())
    {
        std::cout << "[ Error ] Something went wrong loading the Road Network System" << std::endl;
        return;
    }
    rns->printLanes();
    int minX, minY, maxX, maxY;
    rns->getDimensions(minX, minY, maxX, maxY);

    // Create a Scene, in a rectangle given by (x, y, width, height)
    _scene = new QGraphicsScene();
    _scene->setSceneRect(minX, maxY, maxX - minX, maxY - minY);
    // _scene->setSceneRect(-100, -300, 600, 500);

    // This is the appropriate indexing method if you're adding and removing objects.
    _scene->setItemIndexMethod(QGraphicsScene::NoIndex); // CHANGE THIS

    _grns = new graphicalRNS(*rns, identifyLanes);
    _scene->addItem(_grns);


    ui->graphicsView->setScene(_scene);
    ui->graphicsView->setRenderHint(QPainter::Antialiasing);
    ui->graphicsView->setCacheMode(QGraphicsView::CacheBackground);
    ui->graphicsView->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
    ui->graphicsView->setDragMode(QGraphicsView::ScrollHandDrag);


    // Add some zoom capabilities:
    _gzoom = new GraphicalZoom(ui->graphicsView);
    _gzoom->set_modifiers(Qt::NoModifier);

    if ((rns->sectionsSize()) && (rns[0].lanesSize()))
    {
        arr2 blc, trc;
        rns[0][0].getBoundingBox(blc, trc);
        ui->graphicsView->setSceneRect(blc[0], blc[1], trc[0] - blc[0], trc[1] - blc[1]);
    }



    // And finally...
    // show();


    return;
}

RNSWindow::~RNSWindow()
{
    if (_gzoom) delete _gzoom;
    delete ui;
    if (_grns) delete _grns;
    // if (rns) delete rns;
    if (_scene) delete _scene;
}

#endif // QT_CORE_LIB
