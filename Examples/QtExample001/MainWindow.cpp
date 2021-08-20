#include "MainWindow.h"

#include <QElapsedTimer>
#include <QDebug>
#include <QMessageBox>
#include <QException>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);

    ui.glWidget->installEventFilter(this);
}

bool MainWindow::Init() 
{
    // openal
    alutInit(&__argc, __argv);

    // openal
    ALuint buffer;
    buffer = alutCreateBufferFromFile("Scene/Music.wav");
    ALuint source;
    alGenSources(1, &source);
    alSourcei(source, AL_BUFFER, buffer);
    alSourcei(source, AL_LOOPING, true);
    alSourcePlay(source);

    // opengl
    if (false == InitGL(ui.glWidget))
    {
        QMessageBox msgBox;
        msgBox.setWindowTitle("ERROR!");
        msgBox.setText("OpenGL 3.3 not supported!");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();

        return false;
    }

    // physics
    if (false == InitPhysics())
    {
        QMessageBox msgBox;
        msgBox.setWindowTitle("ERROR!");
        msgBox.setText("OpenCLPhysics: init error!");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();

        return false;
    }

    wglMakeCurrent(hDC, hRC);

    // opengl
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);
    //glEnable(GL_BLEND);
    //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    wglSwapIntervalEXT(0);

    // init
    // static
    m_staticmodel.Load("Scene", "Scene.obj", glm::mat4(1.0f), false);
    static_id = m_physics.GenTriMesh();
    m_physics.SetTriMesh(static_id, m_staticmodel.GetAllVertices());
    m_physics.SetMass(static_id, 0.0f); // static

    // dynamic
    m_dynamicmodel.Load("Scene", "barrel.obj", glm::mat4(1.0f), false);
    for (int x = -5; x < 5; x++) 
    {
        for (int y = 10; y < 20; y++) 
        {
            for (int z = -5; z < 5; z++) 
            {
                dynamic_id = m_physics.GenTriMesh();
                m_physics.SetTriMesh(dynamic_id, m_dynamicmodel.GetAllVertices());

                m_physics.SetPosition(dynamic_id, glm::vec3(x, y, z));
                m_physics.SetMass(dynamic_id, 85.0f); // dynamic
            }
        }
        
    }

    // gravity
    m_physics.SetGravity(glm::vec3(0, -9.81f, 0));

    // Copy RAM to GPU
    m_physics.Commit();

    m_elapsedTimer.start();
    m_nElapsedTime = m_nCurrentTime = m_elapsedTimer.nsecsElapsed();

    connect(&m_Timer, SIGNAL(timeout()), this, SLOT(TimerTick()));
    m_Timer.start(0);

    return true;
}

bool MainWindow::InitPhysics() 
{
    std::vector<std::string> listDevices = m_physics.GetDevices();
    if (listDevices.size() == 0)
    {
        return false;
    }

    if (false == m_physics.CreateDevice(listDevices[0]))
    {
        return false;
    }

    return true;
}

void MainWindow::TimerTick()
{
    // update
    wglMakeCurrent(hDC, hRC);

    // getDT
    m_nElapsedTime = m_nCurrentTime;
    m_nCurrentTime = m_elapsedTimer.nsecsElapsed();
    dt = (float)(m_nCurrentTime - m_nElapsedTime) / 1000000000.0f;

    if (dt <= 0.0f)
    {
        dt = 1.0f / 60.0f;
    }

    // print fps
    nFPS++;
    fSec += dt;
    if (fSec >= 1.0f)
    {
        this->setWindowTitle("FPS: " + QString::number(nFPS));

        nFPS = 0;
        fSec = 0.0f;
    }

    if (dt > 1.0f / 10.0f)
    {
        dt = 1.0f / 10.0f;
    }

    // physics
    m_physics.Update(dt);

    int nWidth = ui.glWidget->width();
    int nHeight = ui.glWidget->height();
    if (nWidth < 1) { nWidth = 1; }
    if (nHeight < 1) { nHeight = 1; }

    // draw
    glClearColor(100.0f / 255.0f, 150 / 255.0f, 240 / 255.0f, 1.0f);
    glViewport(0, 0, nWidth, nHeight);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    ;

    SwapBuffers(hDC);
}

bool MainWindow::ExitPhysics()
{
    return true;
}

bool MainWindow::InitGL(QWidget* pWidget)
{
    hWnd = (HWND)pWidget->winId();
    hDC = GetDC(hWnd);

    PIXELFORMATDESCRIPTOR pfd =
    {
        sizeof(PIXELFORMATDESCRIPTOR),                  // Size Of This Pixel Format Descriptor
        1,                              // Version Number
        PFD_DRAW_TO_WINDOW |                        // Format Must Support Window
        PFD_SUPPORT_OPENGL |                        // Format Must Support OpenGL
        PFD_DOUBLEBUFFER,                       // Must Support Double Buffering
        PFD_TYPE_RGBA,                          // Request An RGBA Format
        32,                               // Select Our Color Depth
        0, 0, 0, 0, 0, 0,                       // Color Bits Ignored
        0,                              // No Alpha Buffer
        0,                              // Shift Bit Ignored
        0,                              // No Accumulation Buffer
        0, 0, 0, 0,                         // Accumulation Bits Ignored
        32,                             // Z-Buffer (Depth Buffer)
        0,                              // Stencil Buffer
        0,                              // No Auxiliary Buffer
        PFD_MAIN_PLANE,                         // Main Drawing Layer
        0,                              // Reserved
        0, 0, 0                             // Layer Masks Ignored
    };

    int pf = ChoosePixelFormat(hDC, &pfd);
    SetPixelFormat(hDC, pf, &pfd);
    hRC = wglCreateContext(hDC);
    wglMakeCurrent(hDC, hRC);

    glewInit();

    if (false == glewIsSupported("GL_VERSION_3_3"))
    {
        return false;
    }

    return true;

}

bool MainWindow::eventFilter(QObject* obj, QEvent* event) 
{
    bool ret = QObject::eventFilter(obj, event);

    ;

    return ret;
}

void MainWindow::keyPressEvent(QKeyEvent* event) 
{
    int id = event->key();
    if (id >= 0 && id <= 255)
    {
        m_bKeys[id] = true;
    }

    if (Qt::Key_Escape == event->key())
    {
        close();
    }
}

void MainWindow::keyReleaseEvent(QKeyEvent* event) 
{
    int id = event->key();
    if (id >= 0 && id <= 255)
    {
        m_bKeys[id] = false;
    }
}

void MainWindow::mousePressEvent(QMouseEvent* event) 
{
}

void MainWindow::mouseReleaseEvent(QMouseEvent* event) 
{
}

void MainWindow::mouseMoveEvent(QMouseEvent* event) 
{
}
