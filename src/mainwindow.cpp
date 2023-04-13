#include <iostream>

#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(std::string iFile, bool identifyLanes, QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{

    setup();

    // Add plane
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

    lastCWSize = ui->centralwidget->size();

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
    renderer->setClearColor("black");

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
