#include "myprocess.h"

myProcess::myProcess()
{
    process = new QProcess(this);
    connect(process, &QProcess::readyReadStandardError, this, &myProcess::updateProgress);
}
void myProcess::myCut()
{

    extractFramesFromVideo(ffmpegPath, videoPath,outputPath,frameNumber);
    // 捕获 ffmpeg 输出

};
void myProcess::extractFramesFromVideo(const QString &ffmpegPath, const QString &videoPath, const QString &outputPath, int frameNumber)
{       // 检查视频文件是否存在
    if (!QFile::exists(videoPath)) {
        qDebug() << "Video file not found at: " << videoPath;
        emit sendMSG2Main("Video file not found!");
        return;
    }

    // 检查输出目录是否存在
    QDir outputDir(outputPath);
    if (!outputDir.exists()) {
        outputDir.mkpath(outputPath);  // 创建输出目录
    }

    // 获取源视频的总帧数
    int totalFrames = getVideoFrameCount(ffmpegPath, videoPath);
    if (totalFrames == -1) {
        emit sendMSG2Main("Failed to get the total frame count of the video.");
        return;
    }

    // 检查输入的帧号是否超出范围
    if (frameNumber >= totalFrames) {
        frameNumber = totalFrames - 1;  // 将帧号设置为最大帧号
        QString message = QString("Frame number exceeds the total frames of the video. Adjusted to the last frame (%1).").arg(frameNumber);
         emit sendMSG2Main(message);
    }
    qDebug()<<"frameNumber=="<<frameNumber;
    // 创建 QProcess 对象
    // 构建 FFmpeg 命令，不使用手动加引号，直接传递路径
    // 构建 FFmpeg 命令，提取视频的所有帧
    QStringList arguments;
    arguments << "-i" << videoPath  // 输入视频路径
              << "-vf" << QString("fps=%1").arg(frameNumber)  // 使用 fps 选项控制帧提取
              << outputPath + "/frame_%0004d.png";  // 输出每一帧的图片

    // 调试：输出命令
    qDebug() << "FFmpeg Command: " << ffmpegPath << arguments.join(" ");

    // 启动 ffmpeg 进程
    process->start(ffmpegPath, arguments);

    // 等待进程完成
    process->waitForFinished(-1);

    // 获取进程的输出
    QString output(process->readAllStandardOutput());
    QString error(process->readAllStandardError());

    // 检查进程退出码
    int exitCode = process->exitCode();
    bool success = (exitCode == 0);  // 通常，退出码为 0 表示成功

    // 判断是否剥离成功
    if (success)
    {
        emit sendMSG2Main("Frame extraction succeeded!");
        qDebug() << "FFmpeg Output: " << output;
    } else {
        // 如果失败，输出错误信息
        QString errorMessage = QString("Frame extraction failed! Exit code: %1\nFFmpeg Error: %2").arg(exitCode).arg(error);
       emit sendMSG2Main(errorMessage);
        qDebug() << "errorMessage="<<errorMessage;
    }
}
int myProcess::getVideoFrameCount(const QString &ffmpegPath, const QString &videoPath) {
    QProcess process;
    QStringList arguments;
    arguments << "-i" << videoPath;

    process.start(ffmpegPath, arguments);
    process.waitForFinished();

    QString output = process.readAllStandardError(); // ffmpeg 输出时长和帧率的信息到错误流
    QRegularExpression durationRegExp("Duration: (\\d+):(\\d+):(\\d+\\.\\d+)");
    QRegularExpression fpsRegExp("(\\d+(\\.\\d+)?) fps");

    int hours = 0, minutes = 0;
    double seconds = 0.0, fps = 0.0;

    // 匹配视频时长
    QRegularExpressionMatch durationMatch = durationRegExp.match(output);
    if (durationMatch.hasMatch()) {
        hours = durationMatch.captured(1).toInt();
        minutes = durationMatch.captured(2).toInt();
        seconds = durationMatch.captured(3).toDouble();
    } else {
        return -1;  // 如果无法解析到时长，返回 -1
    }

    // 匹配帧率
    QRegularExpressionMatch fpsMatch = fpsRegExp.match(output);
    if (fpsMatch.hasMatch()) {
        fps = fpsMatch.captured(1).toDouble();
    } else {
        return -1;  // 如果无法解析到帧率，返回 -1
    }

    // 计算总秒数
    totalSeconds = hours * 3600 + minutes * 60 + seconds;

    // 计算总帧数
    int frameCount = static_cast<int>(totalSeconds * fps);

    return frameCount;
}
void myProcess::updateProgress()
{
    QString output = process->readAllStandardError();
    QRegularExpression timeRegExp("time=(\\d{2}):(\\d{2}):(\\d{2})\\.(\\d{2})");
    QRegularExpressionMatch match = timeRegExp.match(output);

    if (match.hasMatch()) {
        int hours = match.captured(1).toInt();
        int minutes = match.captured(2).toInt();
        int seconds = match.captured(3).toInt();
        int currentTimeInSeconds = hours * 3600 + minutes * 60 + seconds;

        // 确保你已经获取并存储了视频的总时长（以秒为单位）
        if (totalSeconds > 0) {
            // 计算百分比
            float progress = (static_cast<float>(currentTimeInSeconds) / totalSeconds) * 100.0f;

            // 发送进度更新信号，传递百分比数据
            emit progressUpdated(progress);
            qDebug() << "updateProgress: " << progress << "%";
        }
    }
}
