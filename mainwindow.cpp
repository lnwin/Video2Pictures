#include "mainwindow.h"
#include "ui_mainwindow.h"
struct PsonnavReadOptions {
    bool verifyChecksum = true;   // 校验 "*HH"；失败行将被跳过
    bool requireValidFlags = false; // 需要位置/姿态状态为 'A' 才收集
};

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
// 计算 NMEA 异或校验（不含'$'，到'*'前一字节）
static inline int nmeaXorChecksum(QStringView sv)
{
    int cs = 0;
    for (qsizetype i = 0; i < sv.size(); ++i)
        cs ^= sv.at(i).toLatin1();   // 协议为 ASCII
    return cs & 0xFF;
}

// 从一行中提取 heading（度）。成功返回 true。
static bool parseHeadingFromPsonnavLine(const QString& line,
                                        double& headingOut,
                                        const PsonnavReadOptions& opt)
{
    if (!line.startsWith(u"$PSONNAV")) return false;

    // 拆出 “$...*HH” 主体与校验和
    qsizetype starIdx = line.lastIndexOf(u'*');
    if (starIdx <= 0) return false; // 无校验部分
    QStringView body = QStringView{line}.mid(1, starIdx - 1); // 去掉开头的 '$'
    QStringView csStr = QStringView{line}.mid(starIdx + 1).trimmed();
    if (csStr.size() >= 2 && opt.verifyChecksum) {
        bool okHex = false;
        int want = csStr.left(2).toInt(&okHex, 16);
        if (!okHex) return false;
        int got = nmeaXorChecksum(body);
        if (want != got) return false; // 校验失败
    }

    // 用 splitRef 避免复制
    const auto fields = body.split(u',', Qt::KeepEmptyParts);
    // 按你的设备协议：Heading 在第 14 个字段（0-based index = 14）
    // 索引对应：
    // 0:$PSONNAV(已去$)  1:UTC 2:lat 3:N/S 4:lon 5:E/W
    // 6:major 7:minor 8:ellipse_dir 9:pos_status
    // 10:depth 11:depth_std 12:roll 13:pitch 14:heading
    // 15:heading_std 16:orient_status 17:sensor_flags ...
    if (fields.size() < 15) return false;

    if (opt.requireValidFlags) {
        // 位置状态 A、姿态状态 A（字段 9 和 16）
        if (!(fields[9]  == u"A" && fields[16] == u"A"))
            return false;
    }

    bool ok = false;
    double hdg = fields[14].toString().toDouble(&ok);
    if (!ok) return false;

    // 规范化到 [0,360)
    if (hdg < 0.0)       hdg += 360.0 * std::ceil((-hdg)/360.0);
    else if (hdg >= 360) hdg = std::fmod(hdg, 360.0);

    headingOut = hdg;
    return true;
}

QVector<double> loadHeadingsFromPsonnavFile(const QString& path,
                                            const PsonnavReadOptions& opt)
{
    QVector<double> headings;
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qWarning() << "Open file failed:" << path;
        return headings;
    }

    QTextStream ts(&f);
    ts.setCodec("UTF-8");
    headings.reserve(4096); // 预留，按需增长

    QString line;
    while (ts.readLineInto(&line)) {
        double hdg;
        if (parseHeadingFromPsonnavLine(line.trimmed(), hdg, opt))
            headings.push_back(hdg);
    }
    return headings;
}
void MainWindow::on_pushButton_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,QStringLiteral("选择惯导文件!"));
    PsonnavReadOptions opt;
    opt.verifyChecksum   = true;   // 建议开启
    opt.requireValidFlags = false; // 需要时改为 true

    QVector<double> hdgs = loadHeadingsFromPsonnavFile(fileName, opt);
    qDebug() << "Headings parsed:" << hdgs.size();
    if (!hdgs.isEmpty())
        qDebug() << "First..last =" << hdgs.first() << ".." << hdgs.last();
}

