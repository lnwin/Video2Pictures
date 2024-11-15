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

    myReconstructionThread= new QThread();
    myreconstruction =new reconstruction() ;
    myreconstruction->moveToThread(myReconstructionThread);
    myReconstructionThread->start();

    connect(this,&MainWindow::start2Cut,myprocess,&myProcess::myCut);
    connect(this,&MainWindow::startProcess,myreconstruction,&reconstruction::start2process);

    connect(myprocess, &myProcess::progressUpdated, this, &MainWindow::onProgressUpdated);
    connect(myprocess,&myProcess::sendMSG2Main,this,&MainWindow::receiveMSG2Main);
    connect(myreconstruction,&reconstruction::sendmySG2Main,this,&MainWindow::receiveMSG2Main);
     connect(myreconstruction,&reconstruction::myprogressUpdated,this,&MainWindow::onProgressUpdated);

}
void MainWindow::onProgressUpdated(float progress)
{

    ui->progressBar->setValue(progress);  // 这里假设 progress 是一个百分比值
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
    myreconstruction->imagePath=thisDirPath;
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

        myreconstruction->outPutCloudPath=thisDirPath;
    }
}


void MainWindow::on_startProcess_clicked()
{
    emit startProcess();
}


void MainWindow::on_FPS_textChanged(const QString &arg1)
{
    myprocess->frameNumber=arg1.toInt();
}



void MainWindow::on_comboBox_pushDirction_currentIndexChanged(int index)
{
    myreconstruction->mydir=index+=1;
}


void MainWindow::on_speed_textChanged(const QString &arg1)
{
    myreconstruction->myspeed=arg1.toFloat();
}


void MainWindow::on_PZAngle_textChanged(const QString &arg1)
{
    myreconstruction->PZAngle=arg1.toFloat();
}





void MainWindow::on_checkBox_stateChanged(int arg1)
{
    qDebug()<<arg1;
    if(arg1==2)
    {
        myreconstruction->useChaZhi=true;
    }
    else
    {
        myreconstruction->useChaZhi=false;
    }
}


void MainWindow::on_selectLocalTxt_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,QStringLiteral("位置文件选择对话框！"));
    ui->localTxt->setText(fileName);
    LocalDatainputFileName=fileName;
    LocalDataoutputFileName=LocalDatainputFileName.chopped(4)+="_out.txt";
}


void MainWindow::on_startGetOut_clicked()
{
    extractData(LocalDatainputFileName,LocalDataoutputFileName);
}
void MainWindow::extractData(const QString &inputFilePath, const QString &outputFilePath) {
    QFile inputFile(inputFilePath);
    if (!inputFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "无法打开输入文件";
        return;
    }

    QFile outputFile(outputFilePath);
    if (!outputFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qDebug() << "无法打开输出文件";
        return;
    }

    QTextStream in(&inputFile);
    QTextStream out(&outputFile);
   out << "dateTime" << "," << "qingJiaoYiZongjiao" << "," << "gaoDuJi" <<"," <<"gaoDu"<< "," <<"zongXiangJiao46"<<","<<"gaodu-50" <<"\n";
    while (!in.atEnd()) {
        QString line = in.readLine();
        QStringList dataList = line.split(",");

        // 确保数据量足够
        if (dataList.size() > 51) {
            // 获取时间和第29、第51个数据
            QString dateTime = dataList.at(dataList.size() - 2).trimmed();  // 假设时间在最后
            QString angle = dataList[12].trimmed();  // 假设时间在最后
            QString data29 = dataList[23].trimmed();
            QString angle2 = dataList[28].trimmed();
            QString data46 = dataList[46].trimmed();
            QString data50 = dataList[50].trimmed();


            // 输出到文件，每行格式：时间, 第29个数据, 第51个数据
            out << dateTime << "," << angle << "," << data29 <<"," <<angle2<< "," <<data46<<","<<data50 <<"\n";
        }
    }

    inputFile.close();
    outputFile.close();
}
