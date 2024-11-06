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
    QString imagePath;
    QString outPutCloudPath;
    int mydir=1;
    std::vector<cv::Vec4f> myPushReconstruction(int dir);
    cv::Point3f mypixelToUnitRay(const cv::Point2f& pixel, const cv::Mat& intrinsic);
    float myspeed=0.05;
    float PZAngle=0;
    cv::Mat intrinsic_linshi;
    cv::Mat intrinsic = (cv::Mat_<double>(3, 3) << 2372.360457159646, 0, 1243.845172805239,
                                     0, 2371.692271094078, 1033.589378054626,
                                     0, 0, 1);
    void saveCloud( QString outPutCloudPath);
public slots:

    void start2process();
signals:
    void sendmySG2Main(QString);
    void myprogressUpdated(float progress);
};

#endif // RECONSTRUCTION_H
