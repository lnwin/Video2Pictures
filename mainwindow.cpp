#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);


    myProcessThread=new QThread();
    myprocess=new  myProcess ();
    myprocess->moveToThread(myProcessThread);
    myProcessThread->start();

    connect(this,&MainWindow::start2Cut,myprocess,&myProcess::myCut);
    connect(myprocess, &myProcess::progressUpdated, this, &MainWindow::onProgressUpdated);
}
void MainWindow::onProgressUpdated(QString progress)
{
    // 在 UI 中更新进度信息
    qDebug() << progress;
    ui->progressBar->setValue(progress.toInt());  // 这里假设 progress 是一个百分比值
}
MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_selectVideo_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,QStringLiteral("文件对话框！"));
    ui->videoPath->setText(fileName);
    myprocess->videoPath=fileName;
}


void MainWindow::on_selectOutputPath_clicked()
{
    QString  thisDirPath = QFileDialog::getExistingDirectory( this, "Rec path", "/");
    if (thisDirPath.isEmpty())
    {
        return;
    }
    else
    {
        ui->pictureOutPath ->setText(thisDirPath);
    }
    myprocess->outputPath=thisDirPath;
}


void MainWindow::on_startCut_clicked()
{
    emit start2Cut();
}
void MainWindow::receiveMSG2Main(const QString &msg)
{
    QMessageBox ssk;
    ssk.setText(msg);
    ssk.setIcon(QMessageBox::Information);  // 设置
    ssk.exec();

};

void MainWindow::on_selectCloudOutPath_clicked()
{
    QString  thisDirPath = QFileDialog::getExistingDirectory( this, "Rec path", "/");
    if (thisDirPath.isEmpty())
    {
        return;
    }
    else
    {
        ui->cloudOutPath ->setText(thisDirPath);
    }
}


void MainWindow::on_startProcess_clicked()
{

}


void MainWindow::on_FPS_textChanged(const QString &arg1)
{
    myprocess->frameNumber=arg1.toInt();
}


