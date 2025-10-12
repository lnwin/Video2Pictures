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
#include <QProgressDialog>
#include <QApplication>
#include <QElapsedTimer>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE
struct YZOffsetGroup {
    double startFrac;      // [0,1)
    double endFrac;        // [0,1)
    double yStartMm;       // 该段 Y 起点偏移(mm)
    double yEndMm;         // 该段 Y 终点偏移(mm)
    double zStartMm;       // 该段 Z 起点偏移(mm)
    double zEndMm;         // 该段 Z 终点偏移(mm)
    bool   interpolate;    // Y/Z 是否线性插值
    bool   useSpeedOverride; // 是否覆盖速度
    double speedOverride;    // 覆盖速度 (m/s)
};
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


    void extractData(const QString &inputFilePath, const QString &outputFilePath) ;


    void on_readCloudFile_clicked();



    void on_CurveStretching_toggled(bool checked);

    void on_Swing_toggled(bool checked);

    void on_selectCloudOutPath_2_clicked();

    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_radioButton_toggled(bool checked);



    void on_radioButton_2_toggled(bool checked);

    void on_radioButton_3_toggled(bool checked);
    void on_pushButton_3_clicked();


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
    bool processCloudWithYZGroups(const QString& inPath,
                                              const QString& outPath,
                                              const std::vector<YZOffsetGroup>& groups,
                                              bool applyXShift,
                                              double fps,
                                  double defaultSpeedMps);

 signals:
    void start2Cut();
    void startProcess();
    void sendGD2Process(QVector<double> hdgs );
};
#endif // MAINWINDOW_H
