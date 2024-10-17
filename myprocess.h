#ifndef MYPROCESS_H
#define MYPROCESS_H

#include <QObject>
#include <QAccessibleObject>
#include <QProcess>
#include <QFileDialog>
#include <QMessageBox>
#include <QRect>
#include <QRegularExpressionMatch>
#include <QThread>

class myProcess:public QThread
{
    Q_OBJECT
public:
    myProcess();
    void extractFramesFromVideo(const QString &ffmpegPath, const QString &videoPath, const QString &outputPath, int frameNumber);
    QString ffmpegPath=QCoreApplication::applicationDirPath() + "/ffmpeg/bin/ffmpeg.exe";
    QString videoPath;QString outputPath;int frameNumber=20;
    int getVideoFrameCount(const QString &ffmpegPath, const QString &videoPath);
    double totalSeconds;
    QProcess *process;  // 处理 ffmpeg 进程
public slots:
    void myCut();
    void updateProgress();
signals:
    void sendMSG2Main(const QString &);

    void progressUpdated(float progress);
};

#endif // MYPROCESS_H
