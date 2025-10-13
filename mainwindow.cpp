#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QRegularExpression>
struct PsonnavReadOptions {
    bool verifyChecksum = true;   // 校验 "*HH"；失败行将被跳过
    bool requireValidFlags = false; // 需要位置/姿态状态为 'A' 才收集
};
static inline bool parseLine_XYZ_I_or_RGB(const QByteArray& line, PcdPoint& p)
{
    const char* s = line.constData();
    char* endp = nullptr;

    float vals[8]; int n=0;
    for (; n<8; ++n) {
        while (*s==' '||*s=='\t'||*s=='\r'||*s=='\n') ++s;
        if (*s=='\0') break;
        vals[n] = strtof(s, &endp);
        if (endp==s) break;
        s = endp;
    }
    if (n < 3) return false;

    p.x = vals[0]; p.y = vals[1]; p.z = vals[2];

    // 先默认“无色”，强度=0
    p.hasColor = false; p.r = p.g = p.b = 0.f;
    p.intensity = 0.f;

    if (n >= 6) { // XYZ + RGB
        float R = vals[3], G = vals[4], B = vals[5];
        const bool is255 = (R>1.f || G>1.f || B>1.f);
        const float k = is255 ? (1.f/255.f) : 1.f;
        p.r = std::clamp(R*k, 0.f, 1.f);
        p.g = std::clamp(G*k, 0.f, 1.f);
        p.b = std::clamp(B*k, 0.f, 1.f);
        p.hasColor = true;
        // 如果还有第7列当强度（少见），继续兼容
        if (n >= 7) p.intensity = vals[6];
    }
    else if (n >= 4) { // XYZI
        p.intensity = vals[3];
    }
    return true;
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
    myreconstruction->yuzhi=ui->yuzhi->text().toInt();
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


// 解析一行：xyz 必须；强度、时间戳可选
// 返回 true 表示成功，并填充 out
static inline const char* skipDelims(const char* s) {
    while (*s == ' ' || *s == '\t' || *s == ',' ) ++s;
    return s;
}
static inline bool parsePcdLineFast(const QByteArray& line, PcdPoint& p) {
    const char* s = line.constData();
    // 跳过前导空白/逗号
    s = skipDelims(s);
    if (*s == '\0' || *s == '#') return false;

    char* end = nullptr;
    p.x = std::strtof(s, &end); if (end==s) return false; s = skipDelims(end);
    p.y = std::strtof(s, &end); if (end==s) return false; s = skipDelims(end);
    p.z = std::strtof(s, &end); if (end==s) return false; s = skipDelims(end);

    // 强度可选
    if (*s != '\0' && *s != '\n' && *s != '\r') {
        p.intensity = std::strtof(s, &end);
        if (end==s) p.intensity = 0.0f;
    } else {
        p.intensity = 0.0f;
    }
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(p.intensity);
}


// 读取并着色：强度<3000 → 深蓝梯度归一化；强度≥3000 → 亮绿色；若行是XYZRGB则保留原色
static bool loadTxtAndFeedToViewer(const QString& path,
                                   cloudRender* viewer,
                                   bool /*normalizeIntensity_ignored*/ = true,
                                   int batchSize = 1000000)
{
    if (!viewer) return false;
    viewer->clearCloud();

    QFile f(path);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return false;

    QProgressDialog dlg(QObject::tr("扫描点云(1/2)…"), QObject::tr("取消"), 0, 0, viewer);
    dlg.setWindowTitle(QObject::tr("正在读取"));
    dlg.setFixedSize(520, 110);
    dlg.setWindowModality(Qt::ApplicationModal);
    dlg.setMinimumDuration(0);
    dlg.setAutoClose(false);
    dlg.setAutoReset(false);

    // ---------- 第1遍：统计强度范围 ----------
    bool hasAnyIntensity = false;
    float minI = 0.f, maxI = 0.f;
    qint64 totalLines = 0;

    QElapsedTimer tick; tick.start();
    const int UI_UPDATE_MS = 120;

    while (!f.atEnd()) {
        if (dlg.wasCanceled()) { f.close(); return false; }
        QByteArray line = f.readLine();
        ++totalLines;

        PcdPoint p;
        if (!parseLine_XYZ_I_or_RGB(line, p)) {
            if (tick.elapsed() > UI_UPDATE_MS) { qApp->processEvents(); tick.restart(); }
            continue;
        }
        if (std::isfinite(p.intensity)) {
            if (!hasAnyIntensity) { minI = maxI = p.intensity; hasAnyIntensity = true; }
            else { minI = std::min(minI, p.intensity); maxI = std::max(maxI, p.intensity); }
        }

        if (tick.elapsed() > UI_UPDATE_MS) { qApp->processEvents(); tick.restart(); }
    }

    // 归一化窗口：把 <3000 的强度映射到 [0,1] 用于“深蓝梯度”
    // - 低端：minI
    // - 高端：min(maxI, 3500)
    float lowNormMin = 0.f, lowNormMax = 1.f;
    if (hasAnyIntensity) {
        lowNormMin = minI;
        lowNormMax = std::min(maxI, 7000.f);
        // 防止窗口过窄或无效
        if (!(lowNormMax > lowNormMin + 1e-6f)) {
            // 常见情形：所有 I 都小且相等，比如全是 0 或全是 255
            // 给一个保底窗口
            lowNormMin = 0.f;
            lowNormMax = (maxI > 0.f ? maxI : 3000.f);
        }
    }

    // ---------- 第2遍：读取 + 上色 + 批量喂入 ----------
    f.seek(0);
    dlg.setLabelText(QObject::tr("加载点云(2/2)…"));
    dlg.setRange(0, int(totalLines));
    dlg.setValue(0);

    // 颜色定义
    const QVector3D DEEP_BLUE_MIN(0.0f, 0.04f, 0.20f); // 更暗的深蓝（t=0）
    const QVector3D DEEP_BLUE_MAX(0.0f, 0.12f, 0.65f); // 更亮的深蓝（t=1）
    const QVector3D BRIGHT_GREEN (0.0f, 1.0f, 0.0f);   // 亮绿色
    constexpr float GREEN_THRESHOLD = 3000.0f;

    auto lerp = [](const QVector3D& a, const QVector3D& b, float t)->QVector3D {
        return QVector3D(a.x() + (b.x()-a.x())*t,
                         a.y() + (b.y()-a.y())*t,
                         a.z() + (b.z()-a.z())*t);
    };
    auto norm01 = [&](float I)->float {
        // 把 I 映射到 [0,1]，窗口为 [lowNormMin, lowNormMax]，并裁剪
        float t = 0.f;
        if (std::isfinite(I)) {
            t = (I - lowNormMin) / std::max(1e-6f, (lowNormMax - lowNormMin));
        }
        return std::clamp(t, 0.0f, 1.0f);
    };

    std::vector<PcdPoint> batch;
    batch.reserve(batchSize);

    qint64 fed = 0, lineNo = 0;
    tick.restart();

    // 批量阶段禁重绘
    viewer->beginBulkLoad(size_t(totalLines));

    while (!f.atEnd()) {
        if (dlg.wasCanceled()) { f.close(); return false; }
        QByteArray line = f.readLine(); ++lineNo;

        PcdPoint p;
        if (!parseLine_XYZ_I_or_RGB(line, p)) {
            if (tick.elapsed() > UI_UPDATE_MS) { dlg.setValue(int(lineNo)); qApp->processEvents(); tick.restart(); }
            continue;
        }

        // —— 若该点没有自带 RGB，则按强度阈值着色 —— //
        if (!p.hasColor) {
            if (std::isfinite(p.intensity) && p.intensity >= GREEN_THRESHOLD) {
                p.hasColor = true; p.r = BRIGHT_GREEN.x(); p.g = BRIGHT_GREEN.y(); p.b = BRIGHT_GREEN.z();
            } else {
                float t = hasAnyIntensity ? norm01(p.intensity) : 0.f;     // t∈[0,1]
                QVector3D c = lerp(DEEP_BLUE_MIN, DEEP_BLUE_MAX, t);       // 深蓝梯度
                p.hasColor = true; p.r = c.x(); p.g = c.y(); p.b = c.z();
            }
        }

        batch.emplace_back(p);
        ++fed;

        if ((int)batch.size() >= batchSize) {
            viewer->getCloud2Show(batch);
            batch.clear();
        }

        if (tick.elapsed() > UI_UPDATE_MS) { dlg.setValue(int(lineNo)); qApp->processEvents(); tick.restart(); }
    }
    if (!batch.empty()) viewer->getCloud2Show(batch);

    f.close();
    dlg.setValue(int(totalLines));
    dlg.close();

    viewer->endBulkLoad();  // 一次排序+重绘

    qDebug() << "[loadTxt] fed points =" << fed
             << " file =" << path
             << " Imin=" << minI << " Imax=" << maxI
             << " normWin=[" << lowNormMin << "," << lowNormMax << "]";
    return (fed > 0);
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


void MainWindow::on_radioButton_3_toggled(bool checked)
{
    ui->openGLWidget->pickEnabled_distance=checked;
    if(!checked)
    {
        ui->openGLWidget->clearSelection();
        ui->openGLWidget->distPickCount = 0; ui->openGLWidget->distIdxA = ui->openGLWidget->distIdxB = -1;
    }

}
// 读取 inPath 指定的点云文本（示例行：ts, x, y, z, ... , intensity）
// 对每个“时间戳组”在 Y 上按 0*DY,1*DY,2*DY… 做偏移，并输出到 outPath
// 返回 true 表示成功
static bool offsetYByTimestampGroups_withProgress(QWidget* parent,
                                                  const QString& inPath,
                                                  const QString& outPath,
                                                  double DY,
                                                  QString* errMsg = nullptr)
{
    QFile fin(inPath);
    if (!fin.open(QIODevice::ReadOnly | QIODevice::Text)) {
        if (errMsg) *errMsg = QStringLiteral("无法打开输入文件：%1").arg(inPath);
        return false;
    }
    QFile fout(outPath);
    if (!fout.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        if (errMsg) *errMsg = QStringLiteral("无法创建输出文件：%1").arg(outPath);
        return false;
    }

    QTextStream tin(&fin);
    QTextStream tout(&fout);
    tout.setRealNumberNotation(QTextStream::SmartNotation);
    tout.setRealNumberPrecision(10);

    // ——— 进度条：按字节进度（避免双遍扫描）———
    const qint64 totalBytes = fin.size();
    QProgressDialog dlg(QObject::tr("正在处理点云 Y 偏移…"), QObject::tr("取消"), 0,
                                                                                   totalBytes > 0 ? int(std::min<qint64>(totalBytes, INT_MAX)) : 0, parent);
    dlg.setWindowTitle(QObject::tr("处理中"));
    dlg.setWindowModality(Qt::ApplicationModal);
    dlg.setMinimumDuration(0);   // 立即显示
    dlg.setAutoReset(false);
    dlg.setAutoClose(false);

    // 时间戳 -> 组序号
    QHash<QString, int> ts2group;
    ts2group.reserve(4096);
    int nextGroupIndex = 0;

    qint64 okCount = 0, skipCount = 0;
    qint64 lastProgressBytes = 0;
    const qint64 progressStep = 1 << 16; // 每64KB刷新一次 UI，降低开销

    while (!tin.atEnd()) {
        QString line = tin.readLine();

        // 取消检查（避免每行都 processEvents，按块刷新）
        if (fin.pos() - lastProgressBytes >= progressStep) {
            lastProgressBytes = fin.pos();
            if (totalBytes > 0) dlg.setValue(int(std::min<qint64>(fin.pos(), INT_MAX)));
            QCoreApplication::processEvents();
            if (dlg.wasCanceled()) {
                if (errMsg) *errMsg = QStringLiteral("用户取消，已输出 %1 行").arg(okCount);
                fin.close(); fout.close();
                // 视需求：可删除半成品输出文件
                // fout.remove();
                return false;
            }
        }

        if (line.trimmed().isEmpty()) { ++skipCount; continue; }

        // 逗号/空格混排兼容
        line.replace(',', ' ');
        const QStringList toks = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
        if (toks.size() < 5) { ++skipCount; continue; }

        const QString ts = toks.at(0);
        bool okX=false, okY=false, okZ=false, okI=false;
        double x = toks.at(1).toDouble(&okX);
        double y = toks.at(2).toDouble(&okY);
        double z = toks.at(3).toDouble(&okZ);
        double intensity = toks.last().toDouble(&okI); // 最后一列作为强度

        if (!(okX && okY && okZ && okI)) { ++skipCount; continue; }

        // 分配时间戳组序号（首次出现赋新组）
        int gid;
        auto it = ts2group.constFind(ts);
        if (it == ts2group.constEnd()) {
            gid = nextGroupIndex++;
            ts2group.insert(ts, gid);
        } else {
            gid = it.value();
        }

        const double yOut = y + gid * DY;

        // 输出：X,Y,Z,强度
        tout << x << "," << yOut << "," << z << "," << intensity << "\n";
        ++okCount;
    }

    // 结束进度
    if (totalBytes > 0) dlg.setValue(int(std::min<qint64>(totalBytes, INT_MAX)));

    fin.close();
    fout.close();

    if (errMsg) {
        *errMsg = QStringLiteral("完成：有效 %1 行，跳过 %2 行，时间戳组数 %3")
                      .arg(okCount).arg(skipCount).arg(ts2group.size());
    }
    return true;
}

void MainWindow::on_pushButton_3_clicked()
{
    QString cloudfileName = QFileDialog::getOpenFileName(this,QStringLiteral("选取Voyis点云！"));
QString err;
double DY = 0.02; // 每组 +2cm
bool ok = offsetYByTimestampGroups_withProgress(this, cloudfileName,
    ui->cloudOutPath->text()+"/XYZ_processed.txt",
    ui->speed->text().toDouble(), &err);

if (!ok) {
    QMessageBox::warning(this, "处理失败", err);
} else {
    QMessageBox::information(this, "完成", err);
}
}


void MainWindow::on_radioButton_4_toggled(bool checked)
{
if(checked)
{
    ui->openGLWidget->leftClickSetsFocus_=true;

}
else
{
    ui->openGLWidget->leftClickSetsFocus_=false;
}
}

