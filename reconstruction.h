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

class reconstruction:public QThread
{
    Q_OBJECT
public:
    reconstruction();
    QString imagePath="C:/Users/40582/Desktop/pushLaser";
    QString outPutCloudPath="C:/Users/40582/Desktop/pushCloud";
    int mydir=1;
    std::vector<cv::Vec4f> myPushReconstruction(int dir);
    cv::Point3f mypixelToUnitRay(const cv::Point2f& pixel, const cv::Mat& intrinsic);
    float myspeed=0.05;
    float PZAngle=0;
    cv::Mat intrinsic_linshi;
    cv::Mat intrinsic = (cv::Mat_<double>(3, 3) << 3203.558782465405, 0, 1235.548825997014,
                                     0, 3202.812554528705, 1035.272924976913,
                                     0, 0, 1);
    void saveCloud( QString outPutCloudPath);
    bool useChaZhi=false;
    std::vector<cv::Vec4f> interpolateFrames(const std::vector<cv::Vec4f>& frame1, const std::vector<cv::Vec4f>& frame2);
public slots:

    void start2process();
signals:
    void sendmySG2Main(QString);
    void myprogressUpdated(float progress);
};

#endif // RECONSTRUCTION_H
