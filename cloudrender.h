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


class cloudRender : public QOpenGLWidget, protected QOpenGLFunctions_4_0_Core
{
    Q_OBJECT
public:
    explicit cloudRender(QWidget *parent = nullptr);
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;

    void selectFocus(); // 启用点选聚焦


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

private:
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

    // Shader源码
    static constexpr const char* vertexShaderSource = R"(
        #version 330 core
        layout (location = 0) in vec3 aPos;
        uniform mat4 projection;
        uniform mat4 view;
        void main()
        {
            gl_Position = projection * view * vec4(aPos, 1.0);
        }
    )";
    static constexpr const char* fragmentShaderSource = R"(
        #version 330 core
        out vec4 FragColor;
        void main()
        {
            FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);
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
};

#endif // CLOUDRENDER_H
