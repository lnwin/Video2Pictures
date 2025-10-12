#include "reconstruction.h"
#include <QDateTime>

// 1) 在函数里/文件顶部定义分段结构（新增速度覆盖）
struct YOffsetGroup {
    double startFrac;    // 起始百分比 [0,1)
    double endFrac;      // 结束百分比 [0,1)
    double startVal;     // 起始偏移 (mm)
    double endVal;       // 结束偏移 (mm)
    bool   interpolate;  // 是否线性插值
    // —— 新增：速度覆盖（单位与 myspeed 一致；比如 “单位/毫秒”）——
    bool   useSpeedOverride = false;
    double speedOverride    = 0.0;   // 若 useSpeedOverride=true，则每帧用它*50推进 disInter
};


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

    myPushReconstruction(1);

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
    int windowRows=30;int windColes=30;
    if (img.empty()) {
        std::cerr << "Error: Empty image in extractLaserLine()" << std::endl;
        return {};
    }

    cv::Mat colordst;
    cv::cvtColor(img, colordst, cv::COLOR_GRAY2BGR);

    int constwindowRows = windowRows;
    int width = img.cols;
    int height = img.rows;
    int ColsBoxNumber = ((2 * (width - windColes)) / windColes) + 1;
    int ColsStepNumber = windColes/2;

    int rowsNumber = (height + windowRows - 1) / windowRows; // 确保计算最后一行

    std::vector<cv::Point> pixels;
    pixels.reserve(height);

    std::vector<std::array<float, 4>> allbox;
    allbox.reserve(rowsNumber * ColsBoxNumber);

    // ================== 遍历窗口 ==================
    for (int i = 0; i < height; i += windowRows)
    {
        int currentWindowRows = std::min(windowRows, height - i); // 确保最后一行 box 不会超界

        for (int k = 0; k < ColsBoxNumber; k++) {
            int currentBoxSumValue = 0;
            int overThresholdTime = 0;
            int box_startX = k * ColsStepNumber;
            int box_endX = std::min(k * ColsStepNumber + windColes, width);

            cv::Rect boxRect(box_startX, i, box_endX - box_startX, currentWindowRows);
            cv::Mat box = img(boxRect);

            for (int y = 0; y < box.rows; ++y) {
                const uchar* rowPtr = box.ptr<uchar>(y);
                for (int x = 0; x < box.cols; ++x) {
                    uchar pixelValue = rowPtr[x];
                    currentBoxSumValue += pixelValue;
                    if (pixelValue >= threshold) overThresholdTime++;
                }
            }

            float BoxScore = currentBoxSumValue * 0.1f + overThresholdTime * 200;
            allbox.push_back({BoxScore, (float)box_startX, (float)box_endX, (float)overThresholdTime});
        }
    }

    std::vector<std::array<float, 4>> targetBoxes(rowsNumber);
    // ================== 查找每一行得分最高的 Box ==================
    for (int boxi = 0; boxi < rowsNumber; ++boxi) {
        float maxScore = 0;
        std::array<float, 4> bestBox;
        for (int boxN = boxi * ColsBoxNumber; boxN < boxi * ColsBoxNumber + ColsBoxNumber; ++boxN) {
            if (allbox[boxN][0] > maxScore) {
                maxScore = allbox[boxN][0];
                bestBox = allbox[boxN];
            }
        }
        targetBoxes[boxi] = bestBox;
    }

    // ================== 计算灰度重心 ==================
    for (size_t s = 0; s < targetBoxes.size(); ++s) {
        if (targetBoxes[s].at(3) > 0) {
            int startY = static_cast<int>(s * constwindowRows);
            int endY = std::min(startY + constwindowRows, height); // 修正 endY 确保最后 box 计算

            int leftBand = std::max(static_cast<int>(targetBoxes[s][1]) - 10, 0);
            int rightBand = std::min(static_cast<int>(targetBoxes[s][2]) + 10, width);

            for (int i = startY; i < endY; i++) {
                int sum = 0;
                float weighted_sum_x = 0;
                const uchar* rowPtr = img.ptr<uchar>(i);

                for (int j = leftBand; j < rightBand; j++) {
                    int g = rowPtr[j];
                    if (g >= threshold) {
                        sum += g;
                        weighted_sum_x += g * j;
                    }
                }

                if (sum > 0) {
                    float x = weighted_sum_x / sum;
                    pixels.push_back(cv::Point(static_cast<int>(x), i));
                }
            }
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

    // 1) 文件列表
    QStringList filters;
    filters << "*.png" << "*.jpg" << "*.jpeg" << "*.bmp" << "*.gif";
    QDir directory(imagePath);
    directory.setSorting(QDir::Name);
    directory.setNameFilters(filters);
    QFileInfoList fileList = directory.entryInfoList();

    const int total = fileList.size();
    if (total <= 0) {
        saveCloud(outPutCloudPath);
        return mycloudResult;
    }

    // 20 FPS（如有不同，改这里）
    const double fps     = 20.0;
    const double frameDt = 1.0 / fps;   // 每帧时长（秒）

    // 若点云内部单位是“米”，把下面设为 0.001；若内部就是“毫米”，保持 1.0
    constexpr double mm2unit = 1.0;

    // 2) 分段配置：百分比 + Y/Z 偏移(mm) + 可选速度覆盖(米/秒)
    struct YZOffsetGroup {
        double startFrac;    // 起始百分比 [0,1)
        double endFrac;      // 结束百分比 [0,1)
        double yStartMm;     // 该段 Y 起始偏移 (mm)
        double yEndMm;       // 该段 Y 结束偏移 (mm)
        double zStartMm;     // 该段 Z 起始偏移 (mm) —— 新增
        double zEndMm;       // 该段 Z 结束偏移 (mm) —— 新增
        bool   interpolate;  // Y/Z 是否线性插值（共用）
        bool   useSpeedOverride; // 是否覆盖速度
        double speedOverride;    // 覆盖速度（单位：米/秒）
    };

    // 你给的分段基础上，给 Z 轴默认 0→0（需要时修改数值即可）
    std::vector<YZOffsetGroup> yGroups = {
        //                 start   end      y0     y1      z0    z1   lerp   useV   v(m/s)
        { /*组1*/         0.00 ,  1.00 ,    0.0 ,  0.0 ,   0.0 , 0.0, false,  false,  0.1 },

    };

    int count = 0;

    foreach (const QFileInfo& fileInfo, fileList) {
        const QString filePath = fileInfo.absoluteFilePath();
        cv::Mat laserIMG = cv::imread(filePath.toLocal8Bit().constData(), cv::IMREAD_GRAYSCALE);
        if (laserIMG.empty()) {
            float progress = (static_cast<float>(count) / std::max(1, total)) * 100.0f;
            emit myprogressUpdated(progress);
            ++count;
            continue;
        }

        // ——当前进度百分比（0~1）——
        const double frac = (total > 1)
                                ? static_cast<double>(count) / static_cast<double>(total - 1)
                                : 0.0;

        // ——从分段表得到本帧 Y/Z 偏移 和 速度（m/s）——
        double yOffsetMm = 0.0, zOffsetMm = 0.0;
        double speedThisFrame = myspeed;  // 默认全局 m/s

        for (const auto& g : yGroups) {
            if (frac >= g.startFrac && frac < g.endFrac) {
                if (g.interpolate) {
                    const double a = (g.endFrac > g.startFrac)
                                         ? (frac - g.startFrac) / (g.endFrac - g.startFrac)
                                         : 0.0;
                    yOffsetMm = g.yStartMm + a * (g.yEndMm - g.yStartMm);
                    zOffsetMm = g.zStartMm + a * (g.zEndMm - g.zStartMm);
                } else {
                    yOffsetMm = g.yEndMm;
                    zOffsetMm = g.zEndMm;
                }
                if (g.useSpeedOverride) speedThisFrame = g.speedOverride; // m/s
                break;
            }
        }

        // ——生成当前帧点云（你的原逻辑）——
        std::vector<cv::Point> laserPointInPixel = myextractLine(laserIMG, yuzhi);
        std::vector<cv::Vec4f> mycloudOnceIMG;
        mycloudOnceIMG.reserve(laserPointInPixel.size());

        intrinsic.convertTo(intrinsic_linshi, CV_64F);
        Eigen::Vector3f P_A;
        Eigen::Vector3f finalPoint;

        for (int i = 0; i < static_cast<int>(laserPointInPixel.size()); ++i) {
            cv::Point3f unit_ray = mypixelToUnitRay(laserPointInPixel[i], intrinsic_linshi);
            double denominator = unit_ray.x * (2.36015) + unit_ray.y * (-0.0117948) + unit_ray.z * 1;
            double s = (-901.984) / denominator;

            // 沿 X 推扫（保持）
            P_A.x() = (s * unit_ray.x) + disInter;
            P_A.y() = (s * unit_ray.y);
            P_A.z() = (s * unit_ray.z);

            finalPoint = P_A;

            // ——加 Y/Z 偏移（mm→内部单位）
            finalPoint[1] += static_cast<float>(yOffsetMm);
            finalPoint[2] += static_cast<float>(zOffsetMm);

            float gray = static_cast<float>(
                laserIMG.at<uchar>(laserPointInPixel[i].y, laserPointInPixel[i].x));

            mycloudOnceIMG.emplace_back(finalPoint[0], finalPoint[1], finalPoint[2], gray);
        }

        // ——拼云逻辑（保持）——
        if (!useChaZhi) {
            mycloudResult.insert(mycloudResult.end(), mycloudOnceIMG.begin(), mycloudOnceIMG.end());
        } else {
            if (!previousFrameData.empty()) {
                auto interpolatedData = interpolateFrames(previousFrameData, mycloudOnceIMG);
                mycloudResult.insert(mycloudResult.end(), interpolatedData.begin(), interpolatedData.end());
            }
            previousFrameData = std::move(mycloudOnceIMG);
        }

        // ——位移推进：速度(米/秒) × 帧时长(秒) → 米
        disInter += speedThisFrame * 50;

        // ——进度条（保持）——
        float progress = (static_cast<float>(count) / std::max(1, total)) * 100.0f;
        emit myprogressUpdated(progress);
        ++count;
    }

    saveCloud(outPutCloudPath);
    return mycloudResult;
}


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

// 20FPS，使用“相对偏航（减首样本）”


void reconstruction::getGD2Process(QVector<double> hdgs )
{
    myhdgs=hdgs;
};
