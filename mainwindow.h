#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QProcess>
#include <QFileDialog>
#include <QMessageBox>
#include <QCoreApplication>
#include <QThread>
#include <myprocess.h>
QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_selectVideo_clicked();

    void on_selectOutputPath_clicked();

    void on_startCut_clicked();

    void on_selectCloudOutPath_clicked();

    void on_startProcess_clicked();

    void on_FPS_textChanged(const QString &arg1);

    void receiveMSG2Main(const QString &);
    void onProgressUpdated(QString progress);

private:
    Ui::MainWindow *ui;
    QThread* myProcessThread;
    myProcess *myprocess;
 signals:
    void start2Cut();

};
#endif // MAINWINDOW_H
