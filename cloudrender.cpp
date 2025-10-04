#include "cloudrender.h"
#include <cmath>
#include <limits>
#include <algorithm>

#define M_PI 3.1415926

cloudRender::cloudRender(QWidget *parent)
    : QOpenGLWidget(parent), program(nullptr), vbo(0), vao(0),
    cameraDistance(3000.0f), cameraYaw(0.0f), cameraPitch(0.0f),
    focusPoint(0, 0, 0)
{
    setFocusPolicy(Qt::StrongFocus);
}
void cloudRender::updateProjection()
{
    int w = std::max(1, width());
    int h = std::max(1, height());

    projection.setToIdentity();
    projection.perspective(45.0f, float(w)/float(h), kNearPlane, kFarPlane);
}

void cloudRender::initializeGL()
{
    initializeOpenGLFunctions();

    program = new QOpenGLShaderProgram(this);
    program->addShaderFromSourceCode(QOpenGLShader::Vertex, vertexShaderSource);
    program->addShaderFromSourceCode(QOpenGLShader::Fragment, fragmentShaderSource);
    program->link();

    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);

    glEnable(GL_DEPTH_TEST);
    glPointSize(1.0f);
    // ① 设定“你想要的初始姿态”
    m_rot    = QQuaternion();           // 例如：面向 -Z
    cameraUp = QVector3D(1, 0, 0);      // 你现在的初始 up
    m_worldUp = cameraUp;               // 记住这就是世界上轴基准
    // ② 计算中心&距离（仅负责中心/距离，不改 m_rot/up）
    resetView();                       // ← 确保 resetView 里不再改 m_rot！
    // ③ 用当前 m_rot/m_worldUp 统一刷新相机与 view
    updateProjection();
    updateCamera();                     // 内部用 m_rot + m_worldUp
    intensityMin =  std::numeric_limits<float>::infinity();
    intensityMax = -std::numeric_limits<float>::infinity();
}

void cloudRender::paintGL()
{
    glBindVertexArray(vao);
    glPointSize(1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    program->bind();

    // 组装一个临时的 GPU 缓冲（x,y,z,i,sel）
    const size_t N = pointCloud.size();
    static std::vector<float> gpuBuf;   // 复用避免反复分配
    gpuBuf.resize(N * 5);
    for (size_t i = 0; i < N; ++i) {
        gpuBuf[i*5 + 0] = pointCloud[i][0];
        gpuBuf[i*5 + 1] = pointCloud[i][1];
        gpuBuf[i*5 + 2] = pointCloud[i][2];
        gpuBuf[i*5 + 3] = pointCloud[i][3];                         // intensity
        gpuBuf[i*5 + 4] = (i < selectMask.size() ? selectMask[i] : 0.0f); // sel
    }

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, gpuBuf.size() * sizeof(float), gpuBuf.data(), GL_DYNAMIC_DRAW);

    // attrib 0: 位置 (x,y,z)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5*sizeof(float), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(0);

    // attrib 1: 强度
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, 5*sizeof(float), reinterpret_cast<void*>(3*sizeof(float)));
    glEnableVertexAttribArray(1);

    // attrib 2: 选择标记
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 5*sizeof(float), reinterpret_cast<void*>(4*sizeof(float)));
    glEnableVertexAttribArray(2);

    program->setUniformValue("projection", projection);
    program->setUniformValue("view", view);
    program->setUniformValue("uMinI", intensityMin);
    program->setUniformValue("uMaxI", intensityMax);

    glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(N));

    glDisableVertexAttribArray(2);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(0);
    program->release();
}



void cloudRender::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
    updateProjection();
}

void cloudRender::updateCamera()
{
    clampCameraDistance();

    QVector3D forward = m_rot.rotatedVector(QVector3D(0, 0, 1)).normalized();
    QVector3D upRaw   = m_rot.rotatedVector(QVector3D(1, 0,  0));  // ← 不是 (1,0,0)！

    // 去 roll：投影到与 forward 垂直的平面
    QVector3D upProj = upRaw - QVector3D::dotProduct(upRaw, forward) * forward;
    if (upProj.lengthSquared() < 1e-6f) {
        upProj = m_worldUp - QVector3D::dotProduct(m_worldUp, forward) * forward;  // ← 用初始化的世界上轴
        if (upProj.lengthSquared() < 1e-6f) upProj = QVector3D(1,0,0);
    }
    cameraUp = upProj.normalized();


    // 相机位置：沿 forward 的反方向退 distance
    cameraPosition = focusPoint - forward * cameraDistance;

    view.setToIdentity();
    view.lookAt(cameraPosition, focusPoint, cameraUp);

    update(); // 触发重绘
}



QVector3D cloudRender::sphericalToCartesian(float yaw, float pitch, float distance)
{
    // yaw: 水平角, pitch: 垂直角
    float x = distance * std::cos(pitch) * std::sin(yaw);
    float y = distance * std::sin(pitch);
    float z = distance * std::cos(pitch) * std::cos(yaw);
    return QVector3D(x, y, z);
}

void cloudRender::resetView()
{
    if (pointCloud.empty()) {
        focusPoint = QVector3D(0,0,0);
        cameraDistance = 1000.0f;  // 空场景默认 1m
    } else {
        // 计算质心
        QVector3D center(0,0,0);
        int valid = 0;
        float maxDist = 0.0f;
        for (const auto& p : pointCloud) {
            if (std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2])) {
                QVector3D pt(p[0], p[1], p[2]);
                center += pt;
                ++valid;
            }
        }
        focusPoint = (valid > 0) ? (center / float(valid)) : QVector3D(0,0,0);

        // 粗略估计点云尺寸，用最大半径作为参考
        for (const auto& p : pointCloud) {
            QVector3D pt(p[0], p[1], p[2]);
            float d = (pt - focusPoint).length();
            if (d > maxDist) maxDist = d;
        }

        // 让相机离质心约 2 倍最大半径，保证点云完全落在视野内
        // 如果点云特别小，则给一个最小距离，比如 500mm
        cameraDistance = std::max(500.0f, maxDist * 2.0f);
    }

    cameraYaw = 0.0f;
    cameraPitch = 0.0f;
    // m_rot = QQuaternion::fromAxisAndAngle(0, 1, 0, 180.0f);  // ← 朝向 +Z
    // updateCamera();
}

QVector3D cloudRender::unproject(float x, float y, float depth)
{
    QMatrix4x4 inv = (projection * view).inverted();
    QVector4D screenPos(x, y, depth, 1.0f);
    QVector4D worldPos = inv * screenPos;
    if (std::abs(worldPos.w()) > 1e-6)
        return QVector3D(worldPos.x(), worldPos.y(), worldPos.z()) / worldPos.w();
    return QVector3D();
}

QVector3D cloudRender::findClosestPoint(const QVector3D& rayOrigin, const QVector3D& rayDir)
{
    float minDist = std::numeric_limits<float>::max();
    QVector3D closest;
    const float searchRadius = 10.0f;
    for (const auto& p : pointCloud) {
        QVector3D pt(p[0], p[1], p[2]);
        QVector3D v = pt - rayOrigin;
        float t = QVector3D::dotProduct(v, rayDir);
        if (t < 0) continue;
        QVector3D proj = rayOrigin + rayDir * t;
        float dist = (pt - proj).length();
        if (dist < searchRadius && dist < minDist) {
            minDist = dist;
            closest = pt;
        }
    }
    return (minDist < searchRadius) ? closest : QVector3D();
}

void cloudRender::mousePressEvent(QMouseEvent *event)
{
    lastMousePos = QVector2D(event->pos());
    if (event->button() == Qt::LeftButton) {

        // —— 新增：选点模式 —— //
        if (pickingEnabled) {
            float x = (2.0f * event->x()) / width() - 1.0f;
            float y = 1.0f - (2.0f * event->y()) / height();
            QVector3D rayStart = unproject(x, y, 0.0f);
            QVector3D rayEnd   = unproject(x, y, 1.0f);
            QVector3D rayDir   = (rayEnd - rayStart).normalized();

            int idx = findClosestIndex(rayStart, rayDir);
            if (idx >= 0) {
                // 去重
                if (std::find(pickedIdx.begin(), pickedIdx.end(), idx) == pickedIdx.end()) {
                    pickedIdx.push_back(idx);
                }
                // 收够 3 个：计算并涂色
                if (pickedIdx.size() == 3) {
                    // 排序信息若还没准备好，这里保证一下
                    if (rankOfIndex.size() != pointCloud.size()) {
                        rebuildSortByXAndRanks();
                    }
                    applySelectionFrom3Picked();
                    // 若选完就退出选点模式，可在此自动关闭；不想自动关则注释下一行
                    // pickingEnabled = false;
                    pickedIdx.clear(); // 清空，便于下一轮选择
                    update();
                }
            }
            return; // 已处理左键
        }

        // —— 你已有的“点选聚焦”逻辑 —— //
        if (needNewFocus) {
            float x = (2.0f * event->x()) / width() - 1.0f;
            float y = 1.0f - (2.0f * event->y()) / height();
            QVector3D rayStart = unproject(x, y, 0.0f);
            QVector3D rayEnd = unproject(x, y, 1.0f);
            QVector3D rayDir = (rayEnd - rayStart).normalized();
            m_selectedPoint = findClosestPoint(rayStart, rayDir);
            if (!m_selectedPoint.isNull()) {
                focusPoint = m_selectedPoint;
                updateCamera();
            }
            needNewFocus = false;
            setCursor(Qt::OpenHandCursor);
        } else {
            setCursor(Qt::ClosedHandCursor);
            isRotating = true;
        }
    }
    else if (event->button() == Qt::RightButton) {
        isPanning = true;
        setCursor(Qt::SizeAllCursor);
    }
}


void cloudRender::mouseReleaseEvent(QMouseEvent *)
{
    isRotating = false;
    isPanning = false;
    setCursor(Qt::OpenHandCursor);
}

void cloudRender::mouseMoveEvent(QMouseEvent *event)
{
    QVector2D curPos = QVector2D(event->pos());
    QVector2D delta  = curPos - lastMousePos;

    if (isRotating) {
        const QVector3D worldUp = m_worldUp;
        float yawDeg   = -delta.x() * m_rotSpeed; // 左右
        float pitchDeg = delta.y() * m_rotSpeed; // 上下

        // 先施加世界Y轴的 yaw
        QQuaternion qYaw = QQuaternion::fromAxisAndAngle(worldUp, yawDeg);
        QQuaternion rotY = qYaw * m_rot;

        // 计算 yaw 后的 forward 与右轴（绕右轴做 pitch）
        QVector3D fYaw   = rotY.rotatedVector(QVector3D(0,0,-1)).normalized();
        QVector3D right  = QVector3D::crossProduct(fYaw, worldUp);
        if (right.lengthSquared() < 1e-6f) right = QVector3D(1,0,0); // 防极点退化
        right.normalize();

        // 俯仰角安全夹角（避免直冲 ±90°）
        const float minAngleDeg = 1.0f;     // forward 与 worldUp 的最小夹角
        float minA = minAngleDeg * float(M_PI) / 180.0f;
        float maxA = float(M_PI) - minA;

        // 预估 pitch 后的 forward
        QQuaternion qPitchTent = QQuaternion::fromAxisAndAngle(right, pitchDeg);
        QVector3D fTent = (qPitchTent * rotY).rotatedVector(QVector3D(0,0,-1)).normalized();

        auto clampDot = [](float d){ return std::max(-1.0f, std::min(1.0f, d)); };
        float angTent = std::acos(clampDot(QVector3D::dotProduct(fTent, worldUp)));

        if (angTent < minA || angTent > maxA) {
            float angCur = std::acos(clampDot(QVector3D::dotProduct(fYaw, worldUp)));
            float target = (angTent < minA) ? minA : maxA;
            float deltaRad = target - angCur;
            pitchDeg = deltaRad * 180.0f / float(M_PI); // 只允许到边界
        }

        // 真正施加 pitch 并更新 m_rot
        QQuaternion qPitch = QQuaternion::fromAxisAndAngle(right, pitchDeg);
        m_rot = (qPitch * rotY).normalized();

        updateCamera();
    }
    else if (isPanning) {
        // 与 updateCamera 同一套基向量（去 roll）
       const QVector3D worldUp = m_worldUp;
        QVector3D forward = m_rot.rotatedVector(QVector3D(0,0,-1)).normalized();
        QVector3D upProj  = worldUp - QVector3D::dotProduct(worldUp, forward) * forward;
        QVector3D up      = upProj.normalized();
        QVector3D right   = QVector3D::crossProduct(forward, up).normalized();

        float panSpeed = 0.001f * cameraDistance;
        focusPoint    += (right * delta.x() + up * delta.y()) * panSpeed;

        updateCamera();
    }

    lastMousePos = curPos;
}


void cloudRender::wheelEvent(QWheelEvent *event)
{
    float factor = event->angleDelta().y() > 0 ? 0.9f : 1.1f;
    cameraDistance *= factor;
    clampCameraDistance();
    updateCamera();
}

void cloudRender::leaveEvent(QEvent *)
{
    unsetCursor();
}
void cloudRender::enterEvent(QEvent *)
{
    setCursor(needNewFocus ? Qt::CrossCursor : Qt::OpenHandCursor);
}

void cloudRender::selectFocus()
{
    needNewFocus = true;
    setCursor(Qt::CrossCursor);
}

void cloudRender::getCloud2Show(const std::vector<PcdPoint>& myCloud)
{
    if (need2Clear) {
        pointCloud2Save.clear();
        pointCloud.clear();
        need2Clear = false;

        intensityMin =  std::numeric_limits<float>::infinity();
        intensityMax = -std::numeric_limits<float>::infinity();

        // 清空选择
        clearSelection();
    }

    pointCloud2Save.insert(pointCloud2Save.end(), myCloud.begin(), myCloud.end());

    size_t oldN = pointCloud.size();
    pointCloud.reserve(oldN + myCloud.size());
    for (const auto& p : myCloud) {
        pointCloud.emplace_back(p.x, p.y, p.z, p.intensity);
        if (std::isfinite(p.intensity)) {
            intensityMin = std::min(intensityMin, p.intensity);
            intensityMax = std::max(intensityMax, p.intensity);
        }
    }

    // 确保 selectMask 与 pointCloud 同步
    selectMask.resize(pointCloud.size(), 0.0f);

    // 数据改变后，若后面要算百分位，需要有排序与 rank
    // 为避免频繁重排：可以等“第三个点选完”再 rebuild。
    // 这里简单起见：每次更新都重建（数据量大时可优化）
    rebuildSortByXAndRanks();

    update();

}
void cloudRender::getScanControl_opengl(quint8 sig)
{
    if(sig!=0x12)
    {
         need2Clear=true;
    }

};

void cloudRender::getsaveData_SIG()
{
    emit saveMyCloud_openGL(pointCloud2Save);
};

void cloudRender::b2C_openGL()
{
    // 计算点云质心与最大距离（半径）
    QVector3D centroid(0, 0, 0);
    float maxDist = 0.0f;

    if (pointCloud.empty()) {
        // 空点云时使用默认值
        centroid = QVector3D(0, 0, 0);
        cameraDistance = 3000.0f;
    } else {
        for (const auto &p : pointCloud)
            centroid += QVector3D(p[0], p[1], p[2]);
        centroid /= float(pointCloud.size());

        for (const auto &p : pointCloud) {
            float d = (QVector3D(p[0], p[1], p[2]) - centroid).length();
            if (d > maxDist) maxDist = d;
        }

        // 根据点云尺度设置相机距离（保持与 resetView 的策略一致）
        cameraDistance = std::max(300.0f, maxDist * 2.0f);
    }

    // 设置焦点与旋转中心
    focusPoint = centroid;

    // 将相机放在质心后方（沿 -Z 方向看向质心）
    // 在球面坐标转笛卡尔里 yaw=PI, pitch=0 会产生 (0,0,-distance)
    // cameraYaw = float(M_PI);
    // cameraPitch = 0.0f;
    m_rot = QQuaternion();   // 朝 -Z 看向质心
    // 更新 view 矩阵（cameraPosition = sphericalToCartesian(...) + focusPoint）
    updateCamera();

    // 重新计算 projection（保证 near/far 合理）
    // 调用 resizeGL 用当前 widget 大小更新 projection（resizeGL 已实现 projection 的设置逻辑）
    resizeGL(width(), height());

    // 强制重绘
    update();
};


// 返回“最近点”的索引（基于你已有的光线最近点搜索）
int cloudRender::findClosestIndex(const QVector3D& rayOrigin, const QVector3D& rayDir)
{
    float minDist = std::numeric_limits<float>::max();
    int   bestIdx = -1;
    const float searchRadius = 10.0f;
    for (int i = 0; i < (int)pointCloud.size(); ++i) {
        const auto& p = pointCloud[i];
        QVector3D pt(p[0], p[1], p[2]);
        QVector3D v = pt - rayOrigin;
        float t = QVector3D::dotProduct(v, rayDir);
        if (t < 0) continue;
        QVector3D proj = rayOrigin + rayDir * t;
        float dist = (pt - proj).length();
        if (dist < searchRadius && dist < minDist) {
            minDist = dist;
            bestIdx = i;
        }
    }
    return bestIdx; // -1 表示没找到
}

// 以 X 坐标排序，并建立 rankOfIndex
void cloudRender::rebuildSortByXAndRanks()
{
    const int N = (int)pointCloud.size();
    sortedIdxByX.resize(N);
    std::iota(sortedIdxByX.begin(), sortedIdxByX.end(), 0);
    std::sort(sortedIdxByX.begin(), sortedIdxByX.end(),
              [&](int a, int b){
                  if (pointCloud[a][0] < pointCloud[b][0]) return true;
                  if (pointCloud[a][0] > pointCloud[b][0]) return false;
                  // X 相等时，用 Y/Z 稍作稳定排序（避免 rank 抖动）
                  if (pointCloud[a][1] != pointCloud[b][1]) return pointCloud[a][1] < pointCloud[b][1];
                  return pointCloud[a][2] < pointCloud[b][2];
              });

    rankOfIndex.assign(N, 0);
    for (int r = 0; r < N; ++r) {
        rankOfIndex[ sortedIdxByX[r] ] = r;
    }

    // 保证 selectMask 尺寸同步
    selectMask.resize(N, 0.0f);
}

// 由 pickedIdx（3 个）求它们各自的 rank，换算百分比并选中“百分位最小~最大”之间所有点
void cloudRender::applySelectionFrom3Picked()
{
    const int N = (int)pointCloud.size();
    if (N == 0 || pickedIdx.size() < 3) return;
    if ((int)rankOfIndex.size() != N) rebuildSortByXAndRanks();

    int r0 = rankOfIndex[pickedIdx[0]];
    int r1 = rankOfIndex[pickedIdx[1]];
    int r2 = rankOfIndex[pickedIdx[2]];

    int rmin = std::min(r0, std::min(r1, r2));
    int rmax = std::max(r0, std::max(r1, r2));

    // 百分比仅用于显示/记录（选中集合实际等价于 rank 区间）
    // float pmin = (N>1) ? (float)rmin / (float)(N-1) : 0.0f;
    // float pmax = (N>1) ? (float)rmax / (float)(N-1) : 1.0f;

    // 清旧的选择
    std::fill(selectMask.begin(), selectMask.end(), 0.0f);

    // 把 rank ∈ [rmin, rmax] 的点全部置 1（绿色）
    for (int r = rmin; r <= rmax; ++r) {
        int idx = sortedIdxByX[r];
        selectMask[idx] = 1.0f;
    }
}

// 清除绿色高亮和已选点
void cloudRender::clearSelection()
{
    pickedIdx.clear();
    selectMask.assign(pointCloud.size(), 0.0f);
}

