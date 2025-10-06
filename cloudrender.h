#ifndef CLOUDRENDER_H
#define CLOUDRENDER_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions_4_0_Core>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <opencv2/core.hpp>
#include <QObject>
#include <QVector3D>
#include <QVector2D>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QFile>
#include <QTextStream>
#include <QLocale>
#include <vector>
#include "myStruct.h" // 假定定义了 struct PcdPoint {float x, y, z, intensity; ...};
#include <QSaveFile>
#include <QDir>
#include <QFileInfo>
#include <QMessageBox>
#include <QLocale>

class cloudRender : public QOpenGLWidget, protected QOpenGLFunctions_4_0_Core
{
    Q_OBJECT
public:
    explicit cloudRender(QWidget *parent = nullptr);
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;

    void selectFocus(); // 启用点选聚焦


     bool pickingEnabled = false;
     bool pickingEnabled_swing = false;
     bool pickEnabled_all = false;
     bool pickEnabled_stretchX = false;   // 新增：X拉伸 + 尾部跟随
     float stretchXStep = 0.02f;          // 每次按键的拉伸系数增量（2%）
     bool pickEnabled_distance = false;

     void  clearSelection();                      // 清除绿色高亮与已选点
      void saveAfterprocessTxt(const QString& dirPath);
     int  distPickCount = 0;              // 已选点计数（0/1）
     int  distIdxA = -1, distIdxB = -1;   // 两个被选点的索引
signals:
    void sendUpdateSIG();
    void saveMyCloud_openGL(std::vector<PcdPoint>);
public slots:
    void getCloud2Show(const std::vector<PcdPoint>&);
    void getsaveData_SIG();    // 保存点云
    void b2C_openGL();
    void getScanControl_opengl(quint8);

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void leaveEvent(QEvent *event) override;
    void enterEvent(QEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;

private:
    // private:
    float intensityMin =  std::numeric_limits<float>::infinity();
    float intensityMax = -std::numeric_limits<float>::infinity();

    QVector3D cameraUp = QVector3D(0, 1, 0);   // 可变的上方向（不做横滚）
    QVector3D m_worldUp = QVector3D(0, 1, 0);  // 初始化后会同步成你的初始 up
    QQuaternion m_rot;          // 相机朝向（相机->世界）
    float       m_rotSpeed = 0.2f; // 旋转灵敏度（度/像素）
    // Orbit camera parameters
    bool need2Clear=true;
    void updateProjection();
    float cameraDistance;
    float cameraYaw, cameraPitch;
    QVector3D focusPoint;
    QVector3D cameraPosition;

    QOpenGLShaderProgram *program;
    GLuint vbo, vao;

    QMatrix4x4 view, projection;
    QVector2D lastMousePos;
    bool isRotating = false;
    bool isPanning = false;
    bool needNewFocus = false;

    // 点云数据
    std::vector<cv::Vec4f> pointCloud;
    std::vector<PcdPoint> pointCloud2Save;
    bool newScan = true;

    // 点选相关
    QVector3D m_selectedPoint;
    QVector3D unproject(float x, float y, float depth);
    QVector3D findClosestPoint(const QVector3D& rayOrigin, const QVector3D& rayDirection);

    // 顶点：位置 + 强度 + 选择标记
    static constexpr const char* vertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    layout (location = 1) in float aI;
    layout (location = 2) in float aSel;
    out float vI;
    out float vSel;
    uniform mat4 projection;
    uniform mat4 view;
    void main()
    {
        vI = aI;
        vSel = aSel;
        gl_Position = projection * view * vec4(aPos, 1.0);
    }
)";

    static constexpr const char* fragmentShaderSource = R"(
    #version 330 core
    in float vI;
    in float vSel;
    out vec4 FragColor;
    uniform float uMinI;
    uniform float uMaxI;

    vec3 jet(float t) {
        float r = clamp(1.5 - abs(4.0*(t-0.75)), 0.0, 1.0);
        float g = clamp(1.5 - abs(4.0*(t-0.50)), 0.0, 1.0);
        float b = clamp(1.5 - abs(4.0*(t-0.25)), 0.0, 1.0);
        return vec3(r,g,b);
    }

    void main()
    {

        if (vSel > 0.5) {
            FragColor = vec4(0.0, 1.0, 0.0, 1.0);
            return;
        }

        float t;
        if (uMaxI <= uMinI) t = 0.0;
        else t = clamp((vI - uMinI) / (uMaxI - uMinI), 0.0, 1.0);
        vec3 col = jet(t);
        FragColor = vec4(col, 1.0);
    }
)";


    // 工具
    void updateCamera();
    QVector3D sphericalToCartesian(float yaw, float pitch, float distance);
    void resetView();
    // 固定视锥（单位：与点云一致——你这边是 mm）
    static constexpr float kNearPlane = 50.0f;      // 5cm
    static constexpr float kFarPlane  = 15000.0f;   // 15m

    inline void clampCameraDistance() {
        // 让相机距离保持在 (near, far) 的合理区间内，给一点余量
        cameraDistance = std::clamp(cameraDistance, kNearPlane * 2.0f, kFarPlane * 0.95f);
    }

    // —— 选点 & 排序/遮罩 —— //
                   // 左键是否用于选点
    std::vector<int> pickedIdx;                  // 已选中的三个点的“索引”
    std::vector<int> sortedIdxByX;               // 全局点云按 X 从小到大排序后的“索引数组”
    std::vector<int> rankOfIndex;                // rankOfIndex[i] = 点 i 在按 X 排序后的名次（0~N-1）
    std::vector<float> selectMask;               // 与 pointCloud 等长：0=未选中, 1=绿色高亮

    int   findClosestIndex(const QVector3D& rayOrigin, const QVector3D& rayDir);
    void  rebuildSortByXAndRanks();              // 根据 pointCloud 重建排序与 rank
    void  applySelectionFrom3Picked();           // 由 3 个 pickedIdx 计算区间并更新 selectMask


    // —— 键盘驱动对选中区间做位移 —— //
    bool haveSelection = false;     // 记录当前是否有一个有效选中区间
    int  selRmin = -1, selRmax = -1;// 选中区间在“按X排序后的名次”范围 [rmin,rmax]
    float stepY = 1.0f;             // ← 每次按键对 Y 的基本位移（mm）
    float stepZ = 1.0f;             // ← 每次按键对 Z 的基本位移（mm）
    QOpenGLShaderProgram* overlayProgram = nullptr;
    GLuint overlayVao = 0, overlayVbo = 0;

    inline bool hasIndex(int i) const {
        return i >= 0 && i < (int)pointCloud.size();
    }




};

#endif // CLOUDRENDER_H
