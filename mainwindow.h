#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QProcess>
#include <QFileDialog>
#include <QMessageBox>
#include <QCoreApplication>
#include <QThread>
#include <myprocess.h>
#include <reconstruction.h>
#include <QVector3D>
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
    void onProgressUpdated(float progress);

    void on_comboBox_pushDirction_currentIndexChanged(int index);

    void on_speed_textChanged(const QString &arg1);

    void on_PZAngle_textChanged(const QString &arg1);


    void on_checkBox_stateChanged(int arg1);

    void on_selectLocalTxt_clicked();

    void on_startGetOut_clicked();
    void extractData(const QString &inputFilePath, const QString &outputFilePath) ;
    void on_pushButton_clicked();

    void on_readCloudFile_clicked();

    void on_findPoint_clicked();

private:
    Ui::MainWindow *ui;
    QThread* myProcessThread;
    myProcess *myprocess;
    QThread *myReconstructionThread;
    reconstruction *myreconstruction;
    QString LocalDatainputFileName;
    QString LocalDataoutputFileName;

    QVector3D parsePointFromLine(const QString& line);
    QVector3D cloudPoint;
    double getCloudPointPosition(const QString& filePath,
                                        const QVector3D& target,
                                        float eps = 1e-4f) ;  //

 signals:
    void start2Cut();
    void startProcess();
    void sendGD2Process(QVector<double> hdgs );
};
#endif // MAINWINDOW_H
