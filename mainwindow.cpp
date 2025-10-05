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
// 兼容“逗号 or 任意空白”分隔
static inline QStringList splitFlexible(const QString& s) {
    return s.contains(',') ? s.split(',', Qt::SkipEmptyParts)
                           : s.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
}

// 解析一行：xyz 必须；强度、时间戳可选
// 返回 true 表示成功，并填充 out
static bool parsePcdLine(const QString& line, PcdPoint& p)
{
    QString s = line.trimmed();
    if (s.isEmpty() || s.startsWith('#')) return false;

    // 统一把逗号替换为空格，然后按空白切分
    s.replace(',', ' ');
    const QStringList toks = s.split(QRegularExpression("\\s+"),
                                     Qt::SkipEmptyParts);
    if (toks.size() < 3) return false;

    const QLocale c = QLocale::c(); // C locale，支持科学计数法 & '.' 小数点
    bool okx=false, oky=false, okz=false, oki=true;

    p.x = c.toFloat(toks[0], &okx);
    p.y = c.toFloat(toks[1], &oky);
    p.z = c.toFloat(toks[2], &okz);

    // 强度可选：有就解析，没有置 0
    if (toks.size() >= 4) {
        p.intensity = c.toFloat(toks[3], &oki);
    } else {
        p.intensity = 0.0f;
        oki = true;
    }

    return okx && oky && okz && oki && std::isfinite(p.x) && std::isfinite(p.y)
           && std::isfinite(p.z) && std::isfinite(p.intensity);
}


// 读取文件并按批量调用 viewer->getCloud2Show(batch)
// normalizeIntensity=true 时，会把强度归一化到 [0,1]（若文件无强度，则按 Z 归一化）
static bool loadTxtAndFeedToViewer(const QString& path,
                                   cloudRender* viewer,
                                   bool normalizeIntensity = true,
                                   int batchSize = 200000)
{
    if (!viewer) return false;

    QFile f(path);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qWarning() << "[loadTxt] open failed:" << path;
        return false;
    }
    QTextStream ts(&f);
    ts.setCodec("UTF-8");
    ts.setLocale(QLocale::c());   // 关键！支持科学计数法，使用 '.' 小数点

    // 先一遍扫描：统计 min/max（用于可选归一化）
    bool first = true;
    float minZ=0.f, maxZ=0.f, minI=0.f, maxI=0.f;
    bool hasAnyIntensity = false;

    // 为了不二次读取整文件到内存，这里先简单地重读两遍文件（一次求范围，一次喂数据）。
    // 如果不想两遍读：可以先不归一化，直接喂；或边读边分块缓存做局部归一化（略复杂）。
    {
        QString line;
        while (ts.readLineInto(&line)) {
            PcdPoint p;
            if (!parsePcdLine(line, p)) continue;

            if (first) {
                minZ = maxZ = p.z;
                minI = maxI = p.intensity;
                first = false;
            } else {
                minZ = std::min(minZ, p.z); maxZ = std::max(maxZ, p.z);
                minI = std::min(minI, p.intensity); maxI = std::max(maxI, p.intensity);
            }
            if (std::isfinite(p.intensity) && std::fabs(p.intensity) > 1e-12f)
                hasAnyIntensity = true;
        }
    }
    f.seek(0);  // 回到开头再读一遍
    ts.seek(0);

    std::vector<PcdPoint> batch;
    batch.reserve(std::min(batchSize, 800000)); // 预留一点

    const bool doI = normalizeIntensity && hasAnyIntensity && (std::fabs(maxI - minI) > 1e-6f);
    const bool doZ = normalizeIntensity && !hasAnyIntensity && (std::fabs(maxZ - minZ) > 1e-6f);
    const float invI = doI ? (1.0f / (maxI - minI)) : 1.0f;
    const float invZ = doZ ? (1.0f / (maxZ - minZ)) : 1.0f;

    QString line;
    qint64 total = 0;
    while (ts.readLineInto(&line)) {
        PcdPoint p;
        if (!parsePcdLine(line, p)) continue;

        if (normalizeIntensity) {
            if (doI) {
                p.intensity = std::clamp((p.intensity - minI) * invI, 0.0f, 1.0f);
            } else if (doZ) {
                p.intensity = std::clamp((p.z - minZ) * invZ, 0.0f, 1.0f);
            } else {
                // 没有可归一化的范围，保持原值（可能全 0）
            }
        }

        batch.push_back(p);
        ++total;

        if ((int)batch.size() >= batchSize) {
            viewer->getCloud2Show(batch);
            batch.clear();
            // 可选：让事件循环喘口气（避免 UI 假死）
            qApp->processEvents();
        }
    }

    if (!batch.empty()) {
        viewer->getCloud2Show(batch);
        batch.clear();
    }

    f.close();
    qDebug() << "[loadTxt] fed points =" << total << "file =" << path;
    // 可选：让视角回到中心（如你已有 b2C_openGL）
    viewer->b2C_openGL();

    return (total > 0);
}

void MainWindow::on_readCloudFile_clicked()
{

    QString cloudfileName = QFileDialog::getOpenFileName(this,QStringLiteral("选取点云！"));
    ui->targetCloudFilePath->setText(cloudfileName);
    // 这里最好检查一下返回值
    // 假设你的控件对象名是 openGLWidget，类型是 cloudRender*
    if (!loadTxtAndFeedToViewer(cloudfileName, ui->openGLWidget, /*normalizeIntensity=*/true, /*batchSize=*/200000)) {
        QMessageBox::warning(this, tr("读取失败"), cloudfileName);
    }
    else
    {
        QMessageBox::information(this, tr("完成"), tr("读取点云成功！"));
    }



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



void MainWindow::on_CurveStretching_toggled(bool checked)
{
    ui->openGLWidget->pickingEnabled=checked;
    if(!checked)
    {ui->openGLWidget->clearSelection();}
}


void MainWindow::on_Swing_toggled(bool checked)
{
    ui->openGLWidget->pickingEnabled_swing =checked;
    if(!checked)
    {ui->openGLWidget->clearSelection();}
}


void MainWindow::on_selectCloudOutPath_2_clicked()
{
    QString  thisDirPath = QFileDialog::getExistingDirectory( this, "Rec path", "/");
    if (thisDirPath.isEmpty())
    {
        return;
    }
    else
    {
        ui->lineEdit->setText(thisDirPath);

    }
}


void MainWindow::on_pushButton_clicked()
{
    ui->openGLWidget->saveAfterprocessTxt(ui->lineEdit->text());
}


void MainWindow::on_pushButton_2_clicked()
{
    ui->openGLWidget->b2C_openGL();
}


void MainWindow::on_radioButton_toggled(bool checked)
{
    ui->openGLWidget->pickEnabled_all =checked;
    if(!checked)
    {ui->openGLWidget->clearSelection();}
}





void MainWindow::on_radioButton_2_toggled(bool checked)
{
    ui->openGLWidget->pickEnabled_stretchX =checked;
    if(!checked)
    {ui->openGLWidget->clearSelection();}
}

