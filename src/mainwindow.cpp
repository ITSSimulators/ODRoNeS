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

#include <iostream>

#include "mainwindow.h"
#include "./ui_mainwindow.h"
using namespace odrones;

MainWindow::MainWindow(std::string iFile, bool identifyLanes, QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{

    setup();

    _rns = new RNS(iFile, concepts::drivingSide::leftHand, true);
    if (!_rns->ready())
    {
        std::cout << "[ Error ] Something went wrong loading the Road Network System" << std::endl;
        return;
    }

    _rns->printLanes();


    int minX, minY, maxX, maxY;
    _rns->getDimensions(minX, minY, maxX, maxY);

    scalar step = 0.1; // approx step size in metres
    for (uint i=0; i < _rns->sectionsSize(); ++i)
    {
        for (uint j=0; j<_rns->sections(i).size(); ++j)
        {
            lane *l = _rns->sections(i)[j];
            std::vector<QByteArray> indexBytes, vertexBytes;
            std::vector<int> indexSize, vertexSize;
            l->fillInVerticesAndIndices(step, indexBytes, vertexBytes, indexSize, vertexSize);
        }
    }



    /* // Add plane */
    Qt3DCore::QEntity *planeEntity = new Qt3DCore::QEntity(root);
    Qt3DExtras::QPlaneMesh *planeMesh = new Qt3DExtras::QPlaneMesh(planeEntity);
    planeMesh->setWidth(20);
    planeMesh->setHeight(20);
    Qt3DExtras::QPhongMaterial *planeMaterial = new Qt3DExtras::QPhongMaterial(planeEntity);
    planeMaterial->setAmbient(QColor(0, 0, 0.7 * 255, 0.1 * 255));
    planeEntity->addComponent(planeMesh);
    planeEntity->addComponent(planeMaterial);

    // Add sphere
    Qt3DCore::QEntity *sphereEntity = new Qt3DCore::QEntity(root);
    Qt3DExtras::QSphereMesh *sphereMesh = new Qt3DExtras::QSphereMesh(sphereEntity);
    Qt3DExtras::QPhongMaterial *sphereMaterial = new Qt3DExtras::QPhongMaterial(sphereEntity);
    sphereMaterial->setAmbient(Qt::red);
    Qt3DCore::QTransform *sphereTransform = new Qt3DCore::QTransform(sphereEntity);
    sphereTransform->setTranslation(QVector3D(0., 5., 0.));
    sphereEntity->addComponent(sphereMesh);
    sphereEntity->addComponent(sphereMaterial);
    sphereEntity->addComponent(sphereTransform);




    view->show();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setup()
{

    ui->setupUi(this);

    ui->centralwidget->setLayout(ui->verticalLayout);
    QWidget *tmp = ui->threeD;

    ui->listWidget->setMinimumHeight(100);
    ui->listWidget->setMaximumHeight(250);

    // store the old values, since  will be used by the new window container:
    QSize thisSize = ui->threeD->size();
    QSize Msize = ui->threeD->maximumSize();
    QSize msize = ui->threeD->minimumSize();
    QRect g = ui->threeD->geometry();
    QSizePolicy sp = ui->threeD->sizePolicy();

    view = new Qt3DExtras::Qt3DWindow();
    view->resize(thisSize);
    view->setMaximumSize(Msize);
    view->setMinimumSize(msize);
    view->setGeometry(g);

    ui->threeD = QWidget::createWindowContainer(view, this);
    ui->threeD->resize(thisSize.width(), 0.4 * thisSize.height());
    ui->threeD->setSizePolicy(sp);
    ui->threeD->setGeometry(g);
    ui->threeD->setMaximumSize(Msize);
    ui->threeD->setMinimumSize(msize);
    ui->verticalLayout->replaceWidget(tmp, ui->threeD);

    Qt3DExtras::QForwardRenderer *renderer = (Qt3DExtras::QForwardRenderer *)view->activeFrameGraph();
    renderer->setClearColor(QColor(240, 240, 240));

    root = new Qt3DCore::QEntity();
    view->setRootEntity(root);

    Qt3DRender::QCamera *camera = view->camera();
    camera->setProjectionType(Qt3DRender::QCameraLens::PerspectiveProjection);
    camera->setFieldOfView(45);
    // Cast to float to ensure float division
    // camera->setAspectRatio(windowWidth / (float) windowHeight);
    camera->setNearPlane(0.1f);
    camera->setFarPlane(100.f);
    camera->setPosition(QVector3D(0., 10., 20.));
    camera->setViewCenter(QVector3D(0., 0., 0.));
    camera->setUpVector(QVector3D(0., 1., 0.));


    cameraController = new Qt3DExtras::QFirstPersonCameraController(root);
    cameraController->setCamera(camera);

}
