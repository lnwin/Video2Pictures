#include "reconstruction.h"
#include <QDateTime>
void rotatePointAroundZ(float x, float y, float &x_new, float &y_new, float angle) {
    // 将角度转换为弧度
    float theta = angle * 3.14 / 180.0;

    // 计算旋转后的坐标
    x_new = x * std::cos(theta) - y * std::sin(theta);
    y_new = x * std::sin(theta) + y * std::cos(theta);
}
std::vector<cv::Vec4f> reconstruction::interpolateFrames(const std::vector<cv::Vec4f>& frame1, const std::vector<cv::Vec4f>& frame2) {
    std::vector<cv::Vec4f> interpolatedData;
    if (frame1.empty() || frame2.empty()) {
        // 如果其中一个为空，直接跳过插值
        return interpolatedData;
    }

    size_t minSize = std::min(frame1.size(), frame2.size());


    // 假设 frame1 和 frame2 的大小相同并且点一一对应
    for (size_t i = 0; i < minSize; ++i) {
        const cv::Vec4f& point1 = frame1[i];
        const cv::Vec4f& point2 = frame2[i];

        // 插入点 1：三分之一位置
        cv::Vec4f interp1(
            point1[0] + (point2[0] - point1[0]) / 3.0f,  // 插值 X
            point1[1],                                   // 保持 Y 不变
            point1[2] + (point2[2] - point1[2]) / 3.0f,  // 插值 Z
            (point1[3] + point2[3]) / 2.0f               // 灰度值取平均
            );

        // 插入点 2：三分之二位置
        cv::Vec4f interp2(
            point1[0] + 2 * (point2[0] - point1[0]) / 3.0f,  // 插值 X
            point1[1],                                       // 保持 Y 不变
            point1[2] + 2 * (point2[2] - point1[2]) / 3.0f,  // 插值 Z
            (point1[3] + point2[3]) / 2.0f                   // 灰度值取平均
            );

        // 将原始点和插值点依次加入
        interpolatedData.push_back(point1);
        interpolatedData.push_back(interp1);
        interpolatedData.push_back(interp2);
    }

    // 最后添加 frame2 的所有点
   // interpolatedData.insert(interpolatedData.end(), frame2.begin(), frame2.end());
    return interpolatedData;
}
reconstruction::reconstruction()
{

}

void reconstruction::start2process()
{
    myPushReconstruction(mydir);
};

cv::Point3f reconstruction::mypixelToUnitRay(const cv::Point2f& pixel, const cv::Mat& intrinsic)
{

    /*
             |u|   |fx 0 cx||Xc/Zc|
             |v| = |0 fy cy||Yc/Zc|    //s mean=(1+s)
             |1|   |0  0  1||  1  |
    */
    const double xd = pixel.x - intrinsic.at<double>(0, 2), yd = pixel.y - intrinsic.at<double>(1, 2);
    cv::Point3f point(xd / intrinsic.at<double>(0, 0), yd / intrinsic.at<double>(1, 1), 1.);
    const double norm = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    return cv::Point3f(point.x / norm, point.y / norm, point.z / norm);

};
std::vector<cv::Point> myextractLine(const cv::Mat &img,int threshold)
{
    cv::Mat dst = img.clone();
    cv::Mat colordst ;   
    cv::cvtColor(dst, colordst, cv::COLOR_GRAY2BGR);

   //  cv::namedWindow("myextractLine", cv::WINDOW_NORMAL);
    // cv::imshow("myextractLine",dst);
  //  qDebug() << " cv::cvtColor(dst, colordst, cv::COLOR_GRAY2BGR);";

    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            if (img.at<uchar>(i, j) < threshold) { dst.at<uchar>(i, j) = 0; }
            else { dst.at<uchar>(i, j) = img.at<uchar>(i, j); }
        }
    }
    std::vector<cv::Point> pixels;

    for (int i = 0; i < dst.rows; i++)
    {
        int sum = 0;
        float x = 0;
        for (int j = 0; j < dst.cols; j++) {
            int g = dst.at<uchar>(i, j);
            if (g) {
                sum += g;
                x += g * j;
            }
        }
        if (sum) {
            x /= sum;
            pixels.push_back(cv::Point(x, i));
        }
    }
    const bool isClosed = false;
    // 设置线的颜色和厚度
    const cv::Scalar lineColor = cv::Scalar(0, 255, 0); // 绿色
    const int lineThickness = 3;    // 画线

    cv::polylines(colordst, pixels, isClosed, lineColor, lineThickness);
    cv::namedWindow("test", cv::WINDOW_NORMAL);
    cv::resizeWindow("test", 720, 480);
    // 计算窗口的位置，使其在屏幕左中间（假设屏幕宽度为1920x1080）
    int windowX = 0;                 // 屏幕左边
    int windowY = 30;  // 屏幕高度中间 - 半个窗口高度
    // 移动窗口到指定位置
    cv::moveWindow("test", windowX, windowY);
    cv::imshow("test",colordst);
    cv::waitKey(3);
    return pixels;
}
cv::Mat rotateImage(const cv::Mat &src, double angle) {
    // 获取图像中心
    cv::Point2f center(src.cols / 2.0, src.rows / 2.0);

    // 计算旋转矩阵
    cv::Mat rotMat = cv::getRotationMatrix2D(center, angle, 1.0);

    // 计算旋转后图像的边界
    cv::Rect bbox = cv::RotatedRect(center, src.size(), angle).boundingRect();

    // 调整旋转矩阵，平移中心
    rotMat.at<double>(0, 2) += bbox.width / 2.0 - center.x;
    rotMat.at<double>(1, 2) += bbox.height / 2.0 - center.y;

    // 执行仿射变换
    cv::Mat dst;
    cv::warpAffine(src, dst, rotMat, bbox.size());

    return dst;
}
std::vector<cv::Vec4f> mycloudResult;
float disInter=0;
std::vector<cv::Vec4f> reconstruction::myPushReconstruction(int dir)
{
    mycloudResult.clear();
    std::vector<cv::Vec4f> previousFrameData;
    // 设置支持的图片格式
    QStringList filters;
    filters << "*.png" << "*.jpg" << "*.jpeg" << "*.bmp" << "*.gif";  // 根据需要添加图片格式
    // 获取目录
    QDir directory(imagePath);
    // 按名称或大小排序（这里按名称排序）
    directory.setSorting(QDir::Name);
    // 过滤出图片文件
    directory.setNameFilters(filters);
    // 获取文件列表
    QFileInfoList fileList = directory.entryInfoList();
    // 循环读取文件

    int count=0;
    foreach (QFileInfo fileInfo, fileList) {
        QString filePath = fileInfo.absoluteFilePath();
        cv::Mat laserIMG = cv::imread(filePath.toLocal8Bit().constData(), cv::IMREAD_GRAYSCALE);

        std::vector<cv::Point> laserPointInPixel = myextractLine(laserIMG, 200);
        std::vector<cv::Vec4f> mycloudOnceIMG;
        intrinsic.convertTo(intrinsic_linshi, CV_64F);
        Eigen::Vector3f P_A;
        Eigen::Vector3f finalPoint;

        float angleInRadians = static_cast<float>(45 * 3.14) / 180.0f;
        float sinValue = std::sin(angleInRadians) * disInter;
        float cosValue = std::cos(angleInRadians) * disInter;

        for (int i = 0; i < laserPointInPixel.size(); i++) {
            cv::Point3f unit_ray = mypixelToUnitRay(laserPointInPixel.at(i), intrinsic_linshi);
            double denominator = unit_ray.x *(-9.62883)+ unit_ray.y * (5.76027) + unit_ray.z * 1;
            double s = (-2810.19) / denominator;

            if (dir == 1) {
                P_A.x() = (s * unit_ray.x) +disInter;
                P_A.y() = (s * unit_ray.y) -1.5*disInter;
                P_A.z() = (s * unit_ray.z);
            } else if (dir == 2) {
                P_A.x() = (s * unit_ray.x) + cosValue;
                P_A.y() = (s * unit_ray.y) + sinValue;
                P_A.z() = (s * unit_ray.z);
            } else if (dir == 3) {
                P_A.x() = (s * unit_ray.x);
                P_A.y() = (s * unit_ray.y);
                P_A.z() = (s * unit_ray.z) + disInter;
            }

            finalPoint = P_A;
            float grayscaleValue = static_cast<float>(laserIMG.at<uchar>(laserPointInPixel.at(i).y, laserPointInPixel.at(i).x));
            // 旋转角度
            float angle = 30.0;
            float x_new, y_new;

            // 旋转点
            rotatePointAroundZ(finalPoint[0],  finalPoint[1], x_new, y_new, angle);
            cv::Vec4f linshiPoint(x_new, y_new, finalPoint[2], grayscaleValue);
            mycloudOnceIMG.push_back(linshiPoint);
        }

        if(!useChaZhi)
        {
           mycloudResult.insert(mycloudResult.end(), mycloudOnceIMG.begin(), mycloudOnceIMG.end());
        }
        else
        {
           // 若 previousFrameData 不为空，插入当前帧和前一帧之间的插值点
           if (!previousFrameData.empty()) {
                std::vector<cv::Vec4f> interpolatedData = interpolateFrames(previousFrameData, mycloudOnceIMG);
                // 将插值结果添加到点云数据中（假设 mycloudResult 存储所有点云数据）
                mycloudResult.insert(mycloudResult.end(), interpolatedData.begin(), interpolatedData.end());
           }

           // 更新 previousFrameData
           previousFrameData = mycloudOnceIMG;
        }

        disInter += myspeed * 50;

        float progress = (static_cast<float>(count) / fileList.size()) * 100.0f;
        emit myprogressUpdated(progress);
        count += 1;
    }


    saveCloud(outPutCloudPath);

    return mycloudResult;
};
void reconstruction::saveCloud( QString outPutCloudPath)
{
    QDateTime filedate;
        //  emit sendItems2Table(filedate.currentDateTime().toString("hh_mm_ss"),filedate.currentDateTime().toString("yyyy.MM.dd&&hh_mm_ss")+".txt","Rotate");
    QString filepath=outPutCloudPath+"/"+filedate.currentDateTime().toString("yyyy_MM_dd")+"/"+filedate.currentDateTime().toString("yyyy.MM.dd&&hh_mm_ss")+".txt";

    QFile cloudfile(filepath);
    QTextStream stream(&cloudfile);
    if(!cloudfile.exists())

    {
        QDir *folder = new QDir;
        folder->mkdir(outPutCloudPath+"/"+filedate.currentDateTime().toString("yyyy_MM_dd"));
        cloudfile.open(QIODevice::WriteOnly | QIODevice::Text);
        for (const auto& point : mycloudResult)
        {
            stream << QString::number(point[0]) + "," + QString::number(point[1]) + "," + QString::number(point[2]) + "," + QString::number(point[3]) + "\n";
        }

    }
    else
    {
        cloudfile.open(QIODevice::WriteOnly | QIODevice::Text|QIODevice::Append);
        for (const auto& point : mycloudResult)
        {
            stream << QString::number(point[0]) + "," + QString::number(point[1]) + "," + QString::number(point[2]) + "," + QString::number(point[3]) + "\n";
        }

    }
    cloudfile.close();

    emit  sendmySG2Main("saved Cloud success");
};
