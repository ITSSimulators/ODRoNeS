#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <Qt3DCore>
#include <Qt3DExtras>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();


private:
    Ui::MainWindow *ui;
    Qt3DExtras::Qt3DWindow *view;
    Qt3DExtras::QFirstPersonCameraController *cameraController;
    QSize lastCWSize;

};
#endif // MAINWINDOW_H
