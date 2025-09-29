#ifndef RECONSTRUCTION_H
#define RECONSTRUCTION_H

#include <QObject>
#include <QThread>
#include <opencv2/core/core.hpp>
#include <QMessageBox>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <QDir>
#include <QDebug>
struct INSHeading { double t; double hdg; }; // t: 当天UTC秒，hdg: [0,360)
class reconstruction:public QThread
{
    Q_OBJECT
public:
    reconstruction();
    QString imagePath="C:/Users/40582/Desktop/CSRE/PICTUREcUT-11.3-4-29";
    QString outPutCloudPath="C:/Users/40582/Desktop/CSRE/CLOUD-11.3-4-29";
    int mydir=1;
    std::vector<cv::Vec4f> myPushReconstruction(int dir);   
    cv::Point3f mypixelToUnitRay(const cv::Point2f& pixel, const cv::Mat& intrinsic);
    float myspeed=0.1;
    float PZAngle=0;
    cv::Mat intrinsic_linshi;
    cv::Mat intrinsic = (cv::Mat_<double>(3, 3) << 2579.454124121254, 0, 1300.363638421887,
                                     0, 2577.912309294391,  1027.11205883085,
                                     0, 0, 1);
    void saveCloud( QString outPutCloudPath);
    bool useChaZhi=false;
    std::vector<cv::Vec4f> interpolateFrames(const std::vector<cv::Vec4f>& frame1, const std::vector<cv::Vec4f>& frame2);
    QVector<double> myhdgs;
public slots:

    void start2process();
    void getGD2Process(QVector<double> hdgs );
signals:
    void sendmySG2Main(QString);
    void myprogressUpdated(float progress);
};

#endif // RECONSTRUCTION_H
