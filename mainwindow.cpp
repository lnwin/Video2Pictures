#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QRegularExpression>
struct PsonnavReadOptions {
    bool verifyChecksum = true;   // 校验 "*HH"；失败行将被跳过
    bool requireValidFlags = false; // 需要位置/姿态状态为 'A' 才收集
};

// 解析一行 x,y,z[,其他]；返回是否成功；自动兼容逗号/空白；输出 tokens 与分隔类型
static bool parseXYZLine(const QString& line,
                         float& x, float& y, float& z,
                         QStringList& tokensOut,
                         QChar& delimOut)
{
    QString s = line.trimmed();
    if (s.isEmpty()) return false;

    QStringList parts;
    QChar delim = s.contains(',') ? QChar(',') : QChar(' ');

    if (delim == QChar(',')) {
        parts = s.split(',', Qt::SkipEmptyParts);
    } else {
        parts = s.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
    }
    if (parts.size() < 3) return false;

    bool okx=false, oky=false, okz=false;
    float xx = parts[0].toFloat(&okx);
    float yy = parts[1].toFloat(&oky);
    float zz = parts[2].toFloat(&okz);
    if (!(okx && oky && okz)) return false;

    x = xx; y = yy; z = zz;
    tokensOut = parts;
    delimOut  = delim;
    return true;
}

// 把 x,y,z + 其余列重新拼回一行；尽量保留原来的分隔风格
static QString rebuildLine(float x, float y, float z,
                           const QStringList& originalTokens,
                           QChar delim)
{
    QStringList out;
    out.reserve(originalTokens.size());
    out << QString::number(x, 'f', 6)
        << QString::number(y, 'f', 6)
        << QString::number(z, 'f', 6);
    for (int i = 3; i < originalTokens.size(); ++i) out << originalTokens[i];

    if (delim == QChar(',')) return out.join(',');
    return out.join(' ');
}

// 给定进度 frac∈[0,1]，从分段表取当帧的 (yMm, zMm, speedMps)
static void sampleYZandSpeed(double frac,
                             const std::vector<YZOffsetGroup>& groups,
                             double& yMm, double& zMm, double& speedMps,
                             double defaultSpeedMps)
{
    yMm = 0.0; zMm = 0.0; speedMps = defaultSpeedMps;
    for (const auto& g : groups) {
        if (frac >= g.startFrac && frac < g.endFrac) {
            if (g.interpolate) {
                const double a = (g.endFrac > g.startFrac)
                                     ? (frac - g.startFrac) / (g.endFrac - g.startFrac)
                                     : 0.0;
                yMm = g.yStartMm + a * (g.yEndMm - g.yStartMm);
                zMm = g.zStartMm + a * (g.zEndMm - g.zStartMm);
            } else {
                yMm = g.yEndMm;
                zMm = g.zEndMm;
            }
            if (g.useSpeedOverride) speedMps = g.speedOverride;
            break;
        }
    }
}

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
    myprocess->outputPath=ui->pictureOutPath->text();
    myreconstruction->imagePath=ui->pictureOutPath->text();
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

        myreconstruction->outPutCloudPath=ui->cloudOutPath->text();
    }
}


void MainWindow::on_startProcess_clicked()
{
    myprocess->outputPath=ui->pictureOutPath->text();

    myreconstruction->imagePath=ui->pictureOutPath->text();

    myreconstruction->outPutCloudPath=ui->cloudOutPath->text();
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
            QString angle =  dataList[12].trimmed();  // 假设时间在最后
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


void MainWindow::on_readCloudFile_clicked()
{
    QString cloudfileName = QFileDialog::getOpenFileName(this,QStringLiteral("选取点云！"));
    ui->targetCloudFilePath->setText(cloudfileName);

}
QVector3D MainWindow::parsePointFromLine(const QString& line)
{
    // 按 “空格 或 逗号” 分隔
    QStringList parts = line.trimmed().split(QRegExp("[,\\s]+"), Qt::SkipEmptyParts);
    if (parts.size() < 3) return QVector3D();

    bool okx=false, oky=false, okz=false;
    float x = parts[0].toFloat(&okx);
    float y = parts[1].toFloat(&oky);
    float z = parts[2].toFloat(&okz);

    if (!(okx && oky && okz)) return QVector3D(); // 返回空点表示解析失败
    return QVector3D(x,y,z);
}

void MainWindow::on_findPoint_clicked()
{

       cloudPoint=parsePointFromLine(ui->targetPoint->text());

        double ratio = getCloudPointPosition(ui->targetCloudFilePath->text(), cloudPoint, 1e-4f);
        if (ratio < 0) {
            ui->textEdit->append("选点未找到或总点数不足。");
        } else {
            ui->textEdit->append("选点位置数据位置占比 ： " + QString::number(ratio, 'f', 3));
        }


}

double MainWindow::getCloudPointPosition(const QString& filePath,const QVector3D& target,float eps)   // 小容差，单位同坐标
{
    QFile f(filePath);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return -1.0;

    QTextStream ts(&f);
    QString line;
    int index = -1;
    int lineId = 0;   // 有效点的顺序编号

    while (ts.readLineInto(&line)) {
        const QString s = line.trimmed();
        if (s.isEmpty()) continue;

        // 同时兼容 “, 分隔” 和 “空白分隔”
        QStringList parts;
        if (s.contains(',')) parts = s.split(',', Qt::SkipEmptyParts);
        else parts = s.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);

        if (parts.size() < 3) continue;

        bool okx=false, oky=false, okz=false;
        float x = parts[0].toFloat(&okx);
        float y = parts[1].toFloat(&oky);
        float z = parts[2].toFloat(&okz);
        if (!(okx && oky && okz)) continue;

        if (index < 0) {
            const QVector3D p(x,y,z);
            if (eps <= 0.0f) {
                // 严格匹配（不推荐）
                if (p.x()==target.x() && p.y()==target.y() && p.z()==target.z())
                    index = lineId;
            } else {
                // 距离平方 <= eps^2 视为命中（推荐）
                if ((p - target).lengthSquared() <= eps*eps)
                    index = lineId;
            }
        }

        ++lineId;  // 统计有效点数量
    }

    if (index < 0 || lineId <= 1) return -1.0;
    return double(index) / double(lineId - 1);  // 转换为 0~1 的相对位置
}

void MainWindow::on_pushButton_2_clicked()
{
    // 选择输出
    QString out = QFileDialog::getSaveFileName(this, QStringLiteral("保存处理后点云"),
                                               QFileInfo(ui->targetCloudFilePath->text()).absolutePath() + "/cloud_processed.txt",
                                               "Text (*.txt);;All (*.*)");
    if (out.isEmpty()) return;

    std::vector<YZOffsetGroup> groups = {
                                         //             start end    y0   y1    z0   z1   lerp useV  v(m/s)
                                         { /*全段*/     0.00, 1.00,  0.0, 0.0,  0.0, 0.0, false,false, 0.10 },
                                         };

    // 想做“沿 Z 逐步抬高 0→200mm，Y 不变；保持速度覆盖 0.10 m/s（若不开 X 推进可无视）”
    // groups = {
    //   {0.00, 1.00, 0.0, 0.0, 0.0, 200.0, true, true, 0.10},
    // };

    // 是否对 X 也做“按速度推进”的位移（默认 false）
    bool applyXShift = false;   // 若想启用，改 true
    double fps = 20.0;          // 帧率仅用于 X 推进：m/s -> mm/点
    double defaultSpeedMps = 0.10;

    bool ok = processCloudWithYZGroups(ui->targetCloudFilePath->text(),
                                       out, groups,
                                       applyXShift, fps, defaultSpeedMps);
    if (ok) {
        QMessageBox mb(this);
        mb.setWindowTitle("完成");
        mb.setText("已输出处理后点云：\n" + out);
        mb.setIcon(QMessageBox::Information);
        mb.setWindowFlag(Qt::WindowStaysOnTopHint, true);
        mb.exec();
    } else {
        QMessageBox::warning(this, "失败", "处理失败，请检查输入文件格式。");
    }
}

// ====== 主处理：把原始点云 -> 应用分段 Y/Z 偏移(+可选X推进) -> 写出 ======
bool MainWindow::processCloudWithYZGroups(const QString& inPath,
                                          const QString& outPath,
                                          const std::vector<YZOffsetGroup>& groups,
                                          bool applyXShift,
                                          double fps,
                                          double defaultSpeedMps)
{
    QFile fin(inPath);
    if (!fin.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "错误", "无法打开输入点云文件");
        return false;
    }
    QTextStream tin(&fin);
    tin.setCodec("UTF-8");

    // 预读统计有效点数（两遍读取最稳妥）
    int validCount = 0;
    {
        QString line;
        while (tin.readLineInto(&line)) {
            float x,y,z; QStringList toks; QChar delim;
            if (parseXYZLine(line, x,y,z, toks, delim)) ++validCount;
        }
        fin.close();
    }
    if (validCount <= 0) {
        QMessageBox::information(this, "提示", "文件中未解析到有效点");
        return false;
    }

    // 重新打开输入，打开输出
    if (!fin.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "错误", "重新打开输入失败");
        return false;
    }
    QFile fout(outPath);
    if (!fout.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "错误", "无法创建输出点云文件");
        return false;
    }
    QTextStream tout(&fout);
    tout.setCodec("UTF-8");

    // 单位：速度是 m/s；点云是 mm；每“点”为一条记录
    // 若把“每条记录当作一帧”，则每“帧”时长：
    const double frameDt = 1.0 / std::max(1.0, fps);   // s
    const double MPS_TO_MM_PER_POINT = 1000.0 * frameDt; // m/s -> mm/点
    double disInterMm = 0.0; // 可选：沿 X 的累计推进(毫米)

    // 第二遍：读取、计算、输出
    QString line;
    int idx = 0;
    int processed = 0;
    while (tin.readLineInto(&line)) {
        float x,y,z; QStringList toks; QChar delim;
        if (!parseXYZLine(line, x,y,z, toks, delim)) {
            // 非点数据（或注释）原样写回
            tout << line << '\n';
            continue;
        }

        // 计算该点在整体中的进度 frac
        const double frac = (validCount > 1) ? double(idx) / double(validCount - 1) : 0.0;

        // 取该进度下的 Y/Z 偏移与速度
        double yMm=0.0, zMm=0.0, speedMps=defaultSpeedMps;
        sampleYZandSpeed(frac, groups, yMm, zMm, speedMps, defaultSpeedMps);

        // 应用偏移（你的点云就是毫米单位，直接相加）
        x += (applyXShift ? static_cast<float>(disInterMm) : 0.0f);
        y += static_cast<float>(yMm);
        z += static_cast<float>(zMm);

        // 写出（保留其他列）
        tout << rebuildLine(x,y,z, toks, delim) << '\n';

        // 更新推进：下一点前进量（若开启 X 推进）
        if (applyXShift) {
            disInterMm += speedMps * MPS_TO_MM_PER_POINT; // mm 累计
        }

        ++idx;
        ++processed;
        if ((processed % 200000) == 0) {
            qApp->processEvents(); // 大文件时避免界面卡死
        }
    }

    fin.close();
    fout.close();
    return true;
}
