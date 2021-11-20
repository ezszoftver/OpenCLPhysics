#include "MainWindow.h"

#include <QElapsedTimer>
#include <QDebug>
#include <QMessageBox>
#include <QException>

int32_t Rand(int32_t nMin, int32_t nMax) 
{
    if (nMin == nMax || nMax < nMin) 
    {
        return nMin;
    }

    int nRet = (rand() % (nMax - nMin)) + nMin;
    return nRet;
}

float Rand(float fMin, float fMax)
{
    if (fabs(fMin - fMax) < 0.0001f || fMax < fMin)
    {
        return fMin;
    }

    float fRet = (((float)rand() / (float)RAND_MAX) * (fMax - fMin)) + fMin;
    return fRet;
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);

    for (int i = 0; i < 256; i++) 
    {
        m_bKeys[i] = false;
    }

    ui.glWidget->installEventFilter(this);
}

bool MainWindow::Init() 
{
    // randomize
    srand(time(NULL));

    // openal
    alutInit(&__argc, __argv);
    
    // openal
    //ALuint buffer;
    //buffer = alutCreateBufferFromFile("Scene/Music.wav");
    //ALuint source;
    //alGenSources(1, &source);
    //alSourcei(source, AL_BUFFER, buffer);
    //alSourcei(source, AL_LOOPING, true);
    //alSourcePlay(source);

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
    m_shaderDraw.Load("Shaders/Draw.vs.txt", "Shaders/Draw.fs.txt");
    m_shaderShadowMap.Load("Shaders/DrawToDepthTexture.vs.txt", "Shaders/DrawToDepthTexture.fs.txt");
    std::vector<int> listTypes;
    listTypes.push_back(GL_RGBA32F);
    m_RenderToShadowTexture.Load(listTypes, nShadowWidth, nShadowWidth);
    m_SkyBox.Load("Scene/", "barren_bk.jpg", "barren_dn.jpg", "barren_ft.jpg", "barren_lf.jpg", "barren_rt.jpg", "barren_up.jpg");

    // static
    // 1/2 - draw
    m_staticmodel.Load("Scene", "Scene.obj", glm::mat4(1.0f));
    m_staticmodel.CreateOpenGLBuffers();
    // 2/2 - physics
    Model physicsmodel;
    physicsmodel.Load("Scene", "Physics.obj", glm::mat4(1.0f), false);
    static_id = m_physics.CreateTriMesh(physicsmodel.GetAllVertices(), Physics::TriMeshType::Static, false);

    // dynamic
    // 1/2 - draw
    m_dynamicmodel.Load("Scene", "barrel.obj", glm::scale(glm::vec3(0.015f, 0.015f, 0.015f)), false);
    m_dynamicmodel.CreateOpenGLBuffers();
    for (int i = 0; i < numTextures; i++)
    {
        QString strFilename = "Scene/" + strFilenames[i];

        Texture texture;
        texture.Load(strFilename.toStdString());
        textures[i] = texture.ID();
    }

    // 2/2 - physics
    int from_dynamic_id = -1;
    for (int x = -5; x < 5; x++)
    {
        for (int z = -5; z < 5; z++)
        {
            for (int y = 0; y < (1/*100db*/ * 10/*1000db*/); y++)
            //for (int y = 0; y < 20; y++)
            {
                int dynamic_id = -1;
                if (-1 == from_dynamic_id)
                {
                    from_dynamic_id = m_physics.CreateTriMesh(m_dynamicmodel.GetAllVertices(), Physics::TriMeshType::Dynamic, false);
                    dynamic_id = from_dynamic_id;
                }
                else 
                {
                    dynamic_id = m_physics.CreateFromId(from_dynamic_id, false);
                }

                float fScale = 1.0f;
                m_physics.SetPosition(dynamic_id, glm::vec3(x * fScale, 10 + (y * fScale), z * fScale));
                //m_physics.SetPosition(dynamic_id, glm::vec3(x * fScale, -0.6f + (y * fScale), z * fScale));
                m_physics.SetEulerRotate(dynamic_id, glm::vec3(0.0f, 0.0f, 0.0f));
                m_physics.SetMass(dynamic_id, 85.0f);
                m_physics.SetLinearVelocity(dynamic_id, glm::vec3(0.0f, 0.0f, 0.0f));
                m_physics.SetAngularVelocity(dynamic_id, glm::vec3(0.0f, 0.0f, 0.0f));
                m_physics.SetLinearDamping(dynamic_id, 0.5f);
                m_physics.SetAngularDamping(dynamic_id, 0.5f);
                m_physics.SetRestitution(dynamic_id, 0.0f);

                m_listDynamicIds.push_back(dynamic_id);

                int id = rand() % numTextures;
                m_listRigidBodiesTextureId.push_back(textures[id]);
            }
        }
    }

    m_physics.Commit();

    // gravity
    m_physics.SetGravity(glm::vec3(0, -1.0f, 0));

    // Avatar
    m_Camera.Init(glm::vec3(15, 3, 15), glm::vec3(0, 2, 10));

    showMaximized();
    QApplication::setOverrideCursor(Qt::BlankCursor);

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
    nIncFPS++;
    fSec += dt;
    if (fSec >= 1.0f)
    {
        nFPS = nIncFPS;
        this->setWindowTitle("FPS: " + QString::number(nFPS) + "; Num RigidBodies: " + QString::number(m_physics.NumRigidBodies()));

        nIncFPS = 0;
        fSec = 0.0f;
    }

    if (dt > 1.0f / 30.0f)
    {
        dt = 1.0f / 30.0f;
    }

    //if (true == m_bKeys[Qt::Key_L]) 
    //{
    //    int nId = m_listDynamicIds[0];
    //    glm::vec3 v3Pos = m_physics.GetPosition(nId);
    //    v3Pos += glm::vec3(+1, 0, 0) * 2.0f * dt;
    //    m_physics.SetPosition(nId, v3Pos);
    //}
    //if (true == m_bKeys[Qt::Key_J])
    //{
    //    int nId = m_listDynamicIds[0];
    //    glm::vec3 v3Pos = m_physics.GetPosition(nId);
    //    v3Pos += glm::vec3(-1, 0, 0) * 2.0f * dt;
    //    m_physics.SetPosition(nId, v3Pos);
    //}
    //if (true == m_bKeys[Qt::Key_I])
    //{
    //    int nId = m_listDynamicIds[0];
    //    glm::vec3 v3Pos = m_physics.GetPosition(nId);
    //    v3Pos += glm::vec3(0, 0, -1) * 2.0f * dt;
    //    m_physics.SetPosition(nId, v3Pos);
    //}
    //if (true == m_bKeys[Qt::Key_K])
    //{
    //    int nId = m_listDynamicIds[0];
    //    glm::vec3 v3Pos = m_physics.GetPosition(nId);
    //    v3Pos += glm::vec3(0, 0, +1) * 2.0f * dt;
    //    m_physics.SetPosition(nId, v3Pos);
    //}
    //if (true == m_bKeys[Qt::Key_H])
    //{
    //    int nId = m_listDynamicIds[0];
    //    glm::vec3 v3Pos = m_physics.GetPosition(nId);
    //    v3Pos += glm::vec3(0, -1, 0) * 2.0f * dt;
    //    m_physics.SetPosition(nId, v3Pos);
    //}
    //if (true == m_bKeys[Qt::Key_U])
    //{
    //    int nId = m_listDynamicIds[0];
    //    glm::vec3 v3Pos = m_physics.GetPosition(nId);
    //    v3Pos += glm::vec3(0, +1, 0) * 2.0f * dt;
    //    m_physics.SetPosition(nId, v3Pos);
    //}

    int nId = m_listDynamicIds[0];
    glm::vec3 v3Rotate = m_physics.GetEulerRotate(nId);
    if (true == m_bKeys[Qt::Key_1])
    {
        v3Rotate.x -= 0.2f * dt;
    }
    if (true == m_bKeys[Qt::Key_2])
    {
        v3Rotate.x += 0.2f * dt;
    }
    if (true == m_bKeys[Qt::Key_3])
    {
        v3Rotate.y -= 0.2f * dt;
    }
    if (true == m_bKeys[Qt::Key_4])
    {
        v3Rotate.y += 0.2f * dt;
    }
    if (true == m_bKeys[Qt::Key_5])
    {
        v3Rotate.z -= 0.2f * dt;
    }
    if (true == m_bKeys[Qt::Key_6])
    {
        v3Rotate.z += 0.2f * dt;
    }
    m_physics.SetEulerRotate(nId, v3Rotate);

    m_physics.Update(dt, 1);

    int nWidth = ui.glWidget->width();
    int nHeight = ui.glWidget->height();
    if (nWidth < 1) { nWidth = 1; }
    if (nHeight < 1) { nHeight = 1; }
    
    // mouse rotate
    QPoint pointDiff = cursor().pos() - QPoint(width() / 2, height() / 2);
    cursor().setPos(QPoint(width() / 2, height() / 2));
    m_Camera.Rotate(pointDiff.x(), pointDiff.y());
    m_Camera.Update(dt);
    
    glm::vec3 v3CameraPos = m_Camera.GetPos();
    glm::vec3 v3CameraAt = m_Camera.GetAt();
    glm::vec3 v3CameraDir = glm::normalize(v3CameraAt - v3CameraPos);
    
    // move
    float speed = 4.0f;
    glm::vec3 vel(0, 0, 0);
    if (true == m_bKeys[Qt::Key_W])
    {
        vel += v3CameraDir;
    }
    if (true == m_bKeys[Qt::Key_S])
    {
        vel += -v3CameraDir;
    }
    if (true == m_bKeys[Qt::Key_D])
    {
        vel += glm::cross(v3CameraDir, glm::vec3(0, 1, 0));
    }
    if (true == m_bKeys[Qt::Key_A])
    {
        vel += -glm::cross(v3CameraDir, glm::vec3(0, 1, 0));
    }
    
    if (glm::length(vel) > 0.0001f)
    {
        vel = glm::normalize(vel);
        vel *= speed;
    
        glm::vec3 v3CameraPos = m_Camera.GetPos();
        v3CameraPos += vel * dt;
        m_Camera.SetPos(v3CameraPos);
    }
    
    glm::vec3 v3LightPos = glm::vec3(32.6785f, 85.7038f, -39.8369f);
    glm::vec3 v3LightAt = glm::vec3(0, 0, 0);
    glm::vec3 v3LightDir = glm::normalize(v3LightAt - v3LightPos);
    
    //glm::mat4 mLightWorld = glm::mat4(1.0f);
    glm::mat4 mLightView = glm::lookAtRH(v3LightPos, v3LightAt, glm::vec3(0, 1, 0));
    glm::mat4 mLightProj = glm::orthoRH(-50.0f, 50.0f, -50.0f, 50.0f, 1.0f, 200.0f);
    
    // draw
    // 1/2 - draw to shadow texture
    m_RenderToShadowTexture.Bind();
    
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glViewport(0, 0, nShadowWidth, nShadowWidth);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    m_shaderShadowMap.Begin();
    glm::mat4 mWorld = m_physics.GetTransform(static_id);
    m_shaderShadowMap.SetMatrix("matWorld", &mWorld);
    m_shaderShadowMap.SetMatrix("matView", &mLightView);
    m_shaderShadowMap.SetMatrix("matProj", &mLightProj);
    
    m_staticmodel.Begin(&m_shaderShadowMap);
    m_staticmodel.Draw(&m_shaderShadowMap);
    m_staticmodel.End(&m_shaderShadowMap);
    
    m_dynamicmodel.Begin(&m_shaderShadowMap);
    for (int j = 0; j < numTextures; j++)
    {
        m_shaderShadowMap.SetTexture("g_Texture", textures[j], 0);
    
        for (int i = 0; i < m_listDynamicIds.size(); i++)
        {
            if (m_listRigidBodiesTextureId.at(i) != textures[j])
            {
                continue;
            }
    
            int dynamic_id = m_listDynamicIds.at(i);
    
            //if (false == m_physics.IsEnabled(dynamic_id))
            //{
            //    continue;
            //}
    
            glm::mat4 mWorld = m_physics.GetTransform(dynamic_id);
            m_shaderShadowMap.SetMatrix("matWorld", &mWorld);
    
            m_dynamicmodel.Draw(&m_shaderShadowMap);
        }
    }
    m_dynamicmodel.End(&m_shaderShadowMap);
    
    m_shaderShadowMap.End();
    m_RenderToShadowTexture.Unbind();
    
    // 2/2 - draw to screen with shadow
    glm::mat4 mCameraView = glm::lookAtRH(v3CameraPos, v3CameraAt, glm::vec3(0, 1, 0));
    glm::mat4 mCameraProj = glm::perspectiveRH(glm::radians(45.0f), (float)nWidth / (float)nHeight, 0.1f, 10000.0f);
    
    glClearColor(0.5f, 0.5f, 1.0f, 1.0f);
    glViewport(0, 0, nWidth, nHeight);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    m_shaderDraw.Begin();
    mWorld = m_physics.GetTransform(static_id);
    m_shaderDraw.SetMatrix("matWorld", &mWorld);
    m_shaderDraw.SetMatrix("matView", &mCameraView);
    m_shaderDraw.SetMatrix("matProj", &mCameraProj);
    m_shaderDraw.SetMatrix("matLightView", &mLightView);
    m_shaderDraw.SetMatrix("matLightProj", &mLightProj);
    m_shaderDraw.SetVector3("lightDir", &v3LightDir);
    m_shaderDraw.SetTexture("g_DepthTexture", m_RenderToShadowTexture.GetTextureID(0), 1);
    
    m_staticmodel.Begin(&m_shaderDraw);
    m_staticmodel.Draw(&m_shaderDraw);
    m_staticmodel.End(&m_shaderDraw);
    
    m_dynamicmodel.Begin(&m_shaderDraw);
    for (int j = 0; j < numTextures; j++)
    {
        m_shaderDraw.SetTexture("g_Texture", textures[j], 0);
    
        for (int i = 0; i < m_listDynamicIds.size(); i++)
        {
            if (m_listRigidBodiesTextureId.at(i) != textures[j])
            {
                continue;
            }
    
            int dynamic_id = m_listDynamicIds.at(i);
    
            //if (false == m_physics.IsEnabled(dynamic_id))
            //{
            //    continue;
            //}
    
            glm::mat4 mWorld = m_physics.GetTransform(dynamic_id);
            m_shaderDraw.SetMatrix("matWorld", &mWorld);
    
            m_dynamicmodel.Draw(&m_shaderDraw);
        }
    }
    m_dynamicmodel.End(&m_shaderDraw);
    
    m_shaderDraw.DisableTexture(1);
    m_shaderDraw.End();
    
    // sky
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(glm::value_ptr(mCameraProj));
    glMatrixMode(GL_MODELVIEW);
    mWorld = glm::mat4(1.0f);
    glLoadMatrixf(glm::value_ptr(mCameraView * mWorld));
    
    //// Draw BBOXs
    //{
    //    glm::vec3 v3Min = m_physics.GetBBoxMin(static_id);
    //    glm::vec3 v3Max = m_physics.GetBBoxMax(static_id);
    //
    //    glPointSize(20.0f);
    //
    //    glBegin(GL_POINTS);
    //    {
    //        glColor3f(1.0f, 0.0f, 0.0f);
    //        glVertex3f(v3Min.x, v3Min.y, v3Min.z);
    //        glVertex3f(v3Max.x, v3Max.y, v3Max.z);
    //        glColor3f(1.0f, 1.0f, 1.0f);
    //    }
    //    glEnd();
    //}
    //
    //for (int i = 0; i < m_listDynamicIds.size(); i++)
    //{
    //    int dynamic_id = m_listDynamicIds.at(i);
    //
    //    glm::vec3 v3Min = m_physics.GetBBoxMin(dynamic_id);
    //    glm::vec3 v3Max = m_physics.GetBBoxMax(dynamic_id);
    //
    //    glPointSize(20.0f);
    //
    //    glBegin(GL_POINTS);
    //    {
    //        glColor3f(1.0f, 1.0f, 0.0f);
    //        glVertex3f(v3Min.x, v3Min.y, v3Min.z);
    //        glVertex3f(v3Max.x, v3Max.y, v3Max.z);
    //        glColor3f(1.0f, 1.0f, 1.0f);
    //    }
    //    glEnd();
    //}
    //
    //// Draw hits
    //std::vector < structHits > *pListHits = m_physics.GetHits();
    //for (int32_t i = 0; i < pListHits->size(); i++)
    //{
    //    structHits hits = pListHits->at(i);
    //
    //    for (int32_t j = 0; j < hits.m_nNumHits; j++) 
    //    {
    //        structVector3 p1 = hits.m_hits[j].m_v3HitPointInWorldA;
    //        structVector3 p2 = hits.m_hits[j].m_v3HitPointInWorldB;
    //        structVector3 p3 = hits.m_hits[j].m_v3Normal;
    //
    //        glPointSize(20.0f);
    //
    //        glBegin(GL_POINTS);
    //        {
    //            glColor3f(1.0f, 0.0f, 0.0f);
    //            glVertex3f(p1.x, p1.y, p1.z);
    //            glColor3f(1.0f, 1.0f, 1.0f);
    //        }
    //        glEnd();
    //
    //        glBegin(GL_POINTS);
    //        {
    //            glColor3f(1.0f, 0.0f, 0.0f);
    //            glVertex3f(p2.x, p2.y, p2.z);
    //            glColor3f(1.0f, 1.0f, 1.0f);
    //        }
    //        glEnd();
    //
    //        glBegin(GL_LINES);
    //        {
    //            glColor3f(0.0f, 1.0f, 0.0f);
    //
    //            glVertex3f(p1.x, p1.y, p1.z);
    //            glVertex3f(p1.x + p3.x, p1.y + p3.y, p1.z + p3.z);
    //
    //            glVertex3f(p2.x, p2.y, p2.z);
    //            glVertex3f(p2.x + p3.x, p2.y + p3.y, p2.z + p3.z);
    //
    //            glColor3f(1.0f, 1.0f, 1.0f);
    //        }
    //        glEnd();
    //    }
    //}
    
    m_SkyBox.Draw(v3CameraPos, 5000.0f);
    
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
