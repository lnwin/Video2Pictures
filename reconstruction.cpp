#include "reconstruction.h"
#include <QDateTime>

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
  //  qDebug() << "colordst";
    cv::cvtColor(dst, colordst, cv::COLOR_GRAY2BGR);


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

    return pixels;
}
std::vector<cv::Vec4f>mycloudOnceIMG;
float disInter=0;
std::vector<cv::Vec4f> reconstruction::myPushReconstruction(int dir)
{
    mycloudOnceIMG.clear();
    disInter=0;
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
    foreach (QFileInfo fileInfo, fileList)
    {
        QString filePath = fileInfo.absoluteFilePath();
        // 使用 OpenCV 读取图像
        cv::Mat laserIMG = cv::imread(filePath.toLocal8Bit().constData(), cv::IMREAD_GRAYSCALE);
        cv::Vec4f linshiPoint;
        std::vector<cv::Point> laserPointInPixel = myextractLine(laserIMG,100);
        // std::vector<Eigen::Vector3f>mycloudOnceIMG;
        intrinsic.convertTo(intrinsic_linshi, CV_64F);
        Eigen::Vector3f P_A;
        Eigen::Vector3f finalPoint;
        // std::cout<<mp.A;
        for(int i=0;i<laserPointInPixel.size();i++)
        {
            cv::Point3f unit_ray =mypixelToUnitRay(laserPointInPixel.at(i), intrinsic_linshi);

            double denominator = unit_ray.x *  1.96705+ unit_ray.y *  0.0382633 + unit_ray.z *( 1);

            double s = 1691.32/ denominator;

            float angleInRadians =float (PZAngle * 3.14) / 180.0f;
            float sinValue = std::sin(angleInRadians)*disInter;
            float cosValue = std::cos(angleInRadians)*disInter;
            if(dir==1)
            {
                P_A.x()=(s * unit_ray.x)+sinValue;
                P_A.y()=(s * unit_ray.y)+cosValue;
                P_A.z()=(s * unit_ray.z);
            }
            else if (dir==2)
            {
                P_A.x()=(s * unit_ray.x)+cosValue;
                P_A.y()=(s * unit_ray.y)+sinValue;
                P_A.z()=(s * unit_ray.z);
            }
            else if (dir==3)
            {
                P_A.x()=(s * unit_ray.x);
                P_A.y()=(s * unit_ray.y);
                P_A.z()=(s * unit_ray.z)+disInter;
            }


            finalPoint=P_A;

            float grayscaleValue = static_cast<float>(laserIMG.at<uchar>(laserPointInPixel.at(i).y, laserPointInPixel.at(i).x));

            linshiPoint = cv::Vec4f(finalPoint[0], finalPoint[1], finalPoint[2], grayscaleValue);
            //linshiPoint=cv::Point3f(finalPoint[0], finalPoint[1], finalPoint[2]);
            mycloudOnceIMG.push_back(linshiPoint);


        }
        disInter+=myspeed*50;
         float progress = (static_cast<float>(count) / fileList.size()) * 100.0f;
        emit myprogressUpdated(progress);
        count+=1;

    }
    saveCloud(outPutCloudPath);

    return mycloudOnceIMG;
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
        for (const auto& point : mycloudOnceIMG)
        {
            stream << QString::number(point[0]) + "," + QString::number(point[1]) + "," + QString::number(point[2]) + "," + QString::number(point[3]) + "\n";
        }

    }
    else
    {
        cloudfile.open(QIODevice::WriteOnly | QIODevice::Text|QIODevice::Append);
        for (const auto& point : mycloudOnceIMG)
        {
            stream << QString::number(point[0]) + "," + QString::number(point[1]) + "," + QString::number(point[2]) + "," + QString::number(point[3]) + "\n";
        }

    }
    cloudfile.close();

    emit  sendmySG2Main("saved Cloud success");
};
