#ifndef MYSTRUCT_H
#define MYSTRUCT_H
#include <QtGlobal>        // 或者 <QtCore/QtGlobal>
#include <QMetaType>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
// 1) 定义点结构（POD，便于序列化）

// cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); /* 摄像机内参数矩阵 */
// cv::Mat distCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */

struct PcdPoint {
    float   x;
    float   y;
    float   z;
    float   intensity;    
    quint64 ts_ms;   // 64位毫秒时间戳
    // 新增：真彩（范围 0~1）与标志（旧数据可不填）
    float   r{0.f}, g{0.f}, b{0.f};
    bool    hasColor{false};
};
Q_DECLARE_METATYPE(PcdPoint)
struct  myProjectInformation
{
    QString projectName="无";
    QString dateTime="无";
    QString projectPath="无";
    QString waterDeepth="无";
    QString longitude="无";
    QString latitude="无";
    QString scanTarget="无";
    QString orther="无";

    QString LaserLevel="0";
    QString ScanThreshold="150";
    QString ScanFPS="60Hz";
    QString ScanMode="转扫";
    QString Diretion="向右 -Y";

    QString CameraExposure="10000";
    QString CameraGain="1";

};
Q_DECLARE_METATYPE(myProjectInformation)

struct planeParameter
{
    float p_a,p_b,p_c,p_d;
    float roateAngle_SLOW,roateAngle_FAST;
    float X_Coef,Y_Coef,Z_Coef;
};Q_DECLARE_METATYPE(planeParameter)

struct calibrateParameter
{

    int calibrateBordCircleRows;
    int calibrateBordCircleCols;
    double calibrateBordCircleO2Odistance_mm;
    std::string img_nolase_path;
    std::string img_lase_path;
    std::string img_origin_path;
    std::string calibrateFile_output_path;

};
#endif // MYSTRUCT_H
