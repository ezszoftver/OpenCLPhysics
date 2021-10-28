#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_MainWindow.h"

#include <Windows.h>

#include <QSplashScreen>
#include <QMainWindow>
#include <QTimer>
#include <QElapsedTimer>
#include <QKeyEvent>
#include <QMouseEvent>

#include "GL/glew.h"
#include "GL/wglew.h"
#include "glm/glm.hpp"
#include "glm/ext.hpp"
#include "AL/alut.h"

#include "Shader.h"
#include "Model.h"
#include "Camera.h"
#include "SkyBox.h"
#include "RenderTarget.h"

#include "OpenCLPhysics.h"
using namespace OpenCLPhysics;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = Q_NULLPTR);

    bool Init();

private slots:
    bool eventFilter(QObject* obj, QEvent* event);
    void keyPressEvent(QKeyEvent* event);
    void keyReleaseEvent(QKeyEvent* event);
    void mousePressEvent(QMouseEvent* event);
    void mouseReleaseEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);

    void TimerTick();
private:
    Ui::MainWindowClass ui;

    // opengl
    bool InitGL(QWidget* pWidget);

    // OpenGL
    HWND hWnd;
    HDC hDC;
    HGLRC hRC;

    // SkyBox
    SkyBox m_SkyBox;

    // Shader
    Shader m_shaderDraw;
    Shader m_shaderShadowMap;

    // RenderTarget
    RenderTarget m_RenderToShadowTexture;
    int nShadowWidth = 2048;

    // SYSTEM
    QTimer m_Timer;
    Camera m_Camera;
    bool   m_bKeys[256];
    //bool m_bMouseButtonDown = false;
    QPoint m_pointElapsedMousePos;

    // FPS
    QElapsedTimer m_elapsedTimer;
    uint64_t m_nElapsedTime;
    uint64_t m_nCurrentTime;
    float dt;
    int nIncFPS = 0;
    int nFPS = 0;
    float fSec = 0;

    // physics
    bool InitPhysics();
    bool ExitPhysics();
    Physics m_physics;
    int static_id;
    std::vector<int> m_listDynamicIds;

    // Static Stage
    Model m_staticmodel;

    // Dynamic Barrel
    Model m_dynamicmodel;
    std::vector<GLuint> m_listRigidBodiesTextureId;
    // Barrel Textures
    int numTextures = 6;
    QString strFilenames[6] = { "diffus_black.tga", "diffus_blue.tga", "diffus_green.tga", "diffus_red.tga", "diffus_rust.tga", "diffus_yellow.tga" };
    GLuint textures[6];
};
