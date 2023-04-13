#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <Qt3DCore>
#include <Qt3DExtras>

#include "rns.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

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
#endif // MAINWINDOW_H
