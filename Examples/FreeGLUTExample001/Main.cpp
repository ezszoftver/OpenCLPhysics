#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include <iostream>
#include <chrono>

#include "GL/glew.h"
#include "GL/wglew.h"
#include "glm/glm.hpp"
#include "glm/ext.hpp"
#include "AL/alut.h"
#include "GL/freeglut.h"

#include "Shader.h"
#include "Model.h"
#include "Camera.h"
#include "SkyBox.h"
#include "RenderTarget.h"

#include "OpenCLPhysics.h"
using namespace OpenCLPhysics;

#define GLUT_KEY_ESC 27
int nMainWindowId;
std::string strTitle = "";

// physics
Physics m_physics;
int static_id;
std::vector<int> m_listDynamicIds;

// glut
int nWidth, nHeight;

// Camera
Camera m_Camera;

// SkyBox
SkyBox m_SkyBox;

// Shader
Shader m_shaderDraw;
Shader m_shaderShadowMap;

// RenderTarget
RenderTarget m_RenderToShadowTexture;
int nShadowWidth = 2048;

// Static Stage
Model m_staticmodel;

// Dynamic Barrel
Model m_dynamicmodel;
std::vector<GLuint> m_listRigidBodiesTextureId;
// Barrel Textures
int numTextures = 6;
std::string strFilenames[6] = { "diffus_black.tga", "diffus_blue.tga", "diffus_green.tga", "diffus_red.tga", "diffus_rust.tga", "diffus_yellow.tga" };
GLuint textures[6];

// FPS
std::chrono::high_resolution_clock::time_point m_nElapsedTime;
std::chrono::high_resolution_clock::time_point m_nCurrentTime;
float dt;
int nIncFPS = 0;
int nFPS = 0;
float fSec = 0;

// Keyboard + Mouse
int nCursorPosX;
int nCursorPosY;
bool m_bKeys[256];

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

bool InitPhysics()
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

bool Init() 
{
	for (int i = 0; i < 256; i++)
	{
		m_bKeys[i] = false;
	}

	// randomize
	srand((unsigned int)time(NULL));

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

	// physics
	if (false == InitPhysics())
	{
		return false;
	}

	// opengl
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);
	//glEnable(GL_BLEND);
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

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
	static_id = m_physics.CreateTriMesh(physicsmodel.GetAllVertices(), TriMeshType::Concave);
	m_physics.SetMass(static_id, 0.0f); // static

	// dynamic
	// 1/2 - draw
	m_dynamicmodel.Load("Scene", "barrel.obj", glm::scale(glm::vec3(0.015f, 0.015f, 0.015f)), false);
	m_dynamicmodel.CreateOpenGLBuffers();
	for (int i = 0; i < numTextures; i++)
	{
		std::string strFilename = "Scene/" + strFilenames[i];

		Texture texture;
		texture.Load(strFilename);
		textures[i] = texture.ID();
	}

	// 2/2 - physics
	int from_dynamic_id = -1;
	for (int x = -5; x < 5; x++)
	{
		//for (int z = -5; z < 5; z++)
		{
			//for (int y = 0; y < (1/*100db*/ * 10/*1000db*/); y++)
			for (int y = 0; y < 1; y++)
			{
				int dynamic_id = -1;
				if (-1 == from_dynamic_id)
				{
					from_dynamic_id = m_physics.CreateTriMesh(m_dynamicmodel.GetAllVertices(), TriMeshType::Concave);
					dynamic_id = from_dynamic_id;
				}
				else
				{
					dynamic_id = m_physics.CreateFromId(from_dynamic_id);
				}

				float fScale = 1.0f;
				//m_physics.SetPosition(dynamic_id, glm::vec3(x * fScale, 10 + (y * fScale), z * fScale));
				m_physics.SetPosition(dynamic_id, glm::vec3(x * fScale, 1.0f + (y * fScale), 12));
				m_physics.SetEulerRotate(dynamic_id, glm::vec3(0.0f, 0.0f, 0.0f));
				m_physics.SetMass(dynamic_id, 85.0f); // dynamic
				m_physics.SetLinearVelocity(dynamic_id, glm::vec3(0.0f, 0.0f, 0.0f));
				m_physics.SetAngularVelocity(dynamic_id, glm::vec3(0.0f, 0.0f, 0.0f));
				m_physics.SetRestitution(dynamic_id, 0.0f);

				m_listDynamicIds.push_back(dynamic_id);

				int id = rand() % numTextures;
				m_listRigidBodiesTextureId.push_back(textures[id]);
			}
		}
	}

	// gravity
	m_physics.SetGravity(glm::vec3(0, -10.0f, 0));

	// Avatar
	m_Camera.Init(glm::vec3(15, 3, 15), glm::vec3(0, 0, 0));

	return true;
}

void ReshapeFunc(int nNewWidth, int nNewHeight) 
{
	nWidth = nNewWidth;
	nHeight = nNewHeight;
}

void KeyboardDownFunc(unsigned char key, int x, int y) 
{
	nCursorPosX = x;
	nCursorPosY = y;

	m_bKeys[key] = true;
}

void KeyboardUpFunc(unsigned char key, int x, int y)
{
	nCursorPosX = x;
	nCursorPosY = y;

	m_bKeys[key] = false;
}

void MouseFunc(int button, int state, int x, int y) 
{
	nCursorPosX = x;
	nCursorPosY = y;
}

void MouseMotionFunc(int x, int y)
{
	nCursorPosX = x;
	nCursorPosY = y;
}

void MousePassiveMotionFunc(int x, int y)
{
	nCursorPosX = x;
	nCursorPosY = y;
}

void glutPrint(int x, int y, char* st)
{
    glPushMatrix();

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0, nWidth, 0.0, nHeight, -1.0, 1.0); // Setup an Ortho view

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    int l, i;
    l = (int)strlen(st);

    glColor3f(1.0f, 0.0f, 0.0f);
    glRasterPos2i(x, nHeight - y);
    for (i = 0; i < l; i++)
    {
        glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, st[i]);
    }
    
    glPopMatrix();
}

void DisplayFunc()
{
    if (true == m_bKeys[GLUT_KEY_ESC]) 
    {
        glutDestroyWindow(nMainWindowId);
        exit(EXIT_SUCCESS);
        return;
    }

	// getDT
	m_nElapsedTime = m_nCurrentTime;
	m_nCurrentTime = std::chrono::high_resolution_clock::now();
	dt = (float)(std::chrono::duration_cast<std::chrono::nanoseconds>(m_nCurrentTime - m_nElapsedTime).count()) / 1000000000.0f;

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
		strTitle = "FPS: " + std::to_string(nFPS) + "; Num RigidBodies: " + std::to_string(m_physics.NumRigidBodies());
		//glutSetWindowTitle(strTitle.c_str());

		nIncFPS = 0;
		fSec = 0.0f;
	}

	if (dt > 1.0f / 10.0f)
	{
		dt = 1.0f / 10.0f;
	}

    if (true == m_bKeys['l'] || true == m_bKeys['L'])
    {
        int nId = m_listDynamicIds[0];
        glm::vec3 v3Pos = m_physics.GetPosition(nId);
        v3Pos += glm::vec3(+1, 0, 0) * 2.0f * dt;
        m_physics.SetPosition(nId, v3Pos);
    }
    if (true == m_bKeys['j'] || true == m_bKeys['J'])
    {
        int nId = m_listDynamicIds[0];
        glm::vec3 v3Pos = m_physics.GetPosition(nId);
        v3Pos += glm::vec3(-1, 0, 0) * 2.0f * dt;
        m_physics.SetPosition(nId, v3Pos);
    }
    if (true == m_bKeys['i'] || true == m_bKeys['I'])
    {
        int nId = m_listDynamicIds[0];
        glm::vec3 v3Pos = m_physics.GetPosition(nId);
        v3Pos += glm::vec3(0, 0, -1) * 2.0f * dt;
        m_physics.SetPosition(nId, v3Pos);
    }
    if (true == m_bKeys['k'] || true == m_bKeys['K'])
    {
        int nId = m_listDynamicIds[0];
        glm::vec3 v3Pos = m_physics.GetPosition(nId);
        v3Pos += glm::vec3(0, 0, +1) * 2.0f * dt;
        m_physics.SetPosition(nId, v3Pos);
    }
    if (true == m_bKeys['h'] || true == m_bKeys['H'])
    {
        int nId = m_listDynamicIds[0];
        glm::vec3 v3Pos = m_physics.GetPosition(nId);
        v3Pos += glm::vec3(0, -1, 0) * 2.0f * dt;
        m_physics.SetPosition(nId, v3Pos);
    }
    if (true == m_bKeys['u'] || true == m_bKeys['U'])
    {
        int nId = m_listDynamicIds[0];
        glm::vec3 v3Pos = m_physics.GetPosition(nId);
        v3Pos += glm::vec3(0, +1, 0) * 2.0f * dt;
        m_physics.SetPosition(nId, v3Pos);
    }

    int nId = m_listDynamicIds[0];
    glm::vec3 v3Rotate = m_physics.GetEulerRotate(nId);
    if (true == m_bKeys['1'])
    {
        v3Rotate.x -= 0.2f * dt;
    }
    if (true == m_bKeys['2'])
    {
        v3Rotate.x += 0.2f * dt;
    }
    if (true == m_bKeys['3'])
    {
        v3Rotate.y -= 0.2f * dt;
    }
    if (true == m_bKeys['4'])
    {
        v3Rotate.y += 0.2f * dt;
    }
    if (true == m_bKeys['5'])
    {
        v3Rotate.z -= 0.2f * dt;
    }
    if (true == m_bKeys['6'])
    {
        v3Rotate.z += 0.2f * dt;
    }
    m_physics.SetEulerRotate(nId, v3Rotate);

	m_physics.Update(dt, 5);

	if (nWidth < 1) { nWidth = 1; }
	if (nHeight < 1) { nHeight = 1; }

	// mouse rotate
	int nPointDiffX = nCursorPosX - (nWidth / 2);
	int nPointDiffY = nCursorPosY - (nHeight / 2);
	glutWarpPointer(nWidth / 2, nHeight / 2);
	m_Camera.Rotate(nPointDiffX, nPointDiffY);
	m_Camera.Update(dt);
    glm::vec3 v3CameraPos = m_Camera.GetPos();
    glm::vec3 v3CameraAt = m_Camera.GetAt();
    glm::vec3 v3CameraDir = glm::normalize(v3CameraAt - v3CameraPos);

    // move
    float speed = 4.0f;
    glm::vec3 vel(0, 0, 0);
    if (true == m_bKeys['w'] || true == m_bKeys['W'])
    {
        vel += v3CameraDir;
    }
    if (true == m_bKeys['s'] || true == m_bKeys['S'])
    {
        vel += -v3CameraDir;
    }
    if (true == m_bKeys['d'] || true == m_bKeys['D'])
    {
        vel += glm::cross(v3CameraDir, glm::vec3(0, 1, 0));
    }
    if (true == m_bKeys['a'] || true == m_bKeys['A'])
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

    // Draw BBOXs
    {
        glm::vec3 v3Min = m_physics.GetBBoxMin(static_id);
        glm::vec3 v3Max = m_physics.GetBBoxMax(static_id);

        glPointSize(20.0f);

        glBegin(GL_POINTS);
        {
            glColor3f(1.0f, 0.0f, 0.0f);
            glVertex3f(v3Min.x, v3Min.y, v3Min.z);
            glVertex3f(v3Max.x, v3Max.y, v3Max.z);
            glColor3f(1.0f, 1.0f, 1.0f);
        }
        glEnd();
    }

    for (int i = 0; i < m_listDynamicIds.size(); i++)
    {
        int dynamic_id = m_listDynamicIds.at(i);

        glm::vec3 v3Min = m_physics.GetBBoxMin(dynamic_id);
        glm::vec3 v3Max = m_physics.GetBBoxMax(dynamic_id);

        glPointSize(20.0f);

        glBegin(GL_POINTS);
        {
            glColor3f(1.0f, 1.0f, 0.0f);
            glVertex3f(v3Min.x, v3Min.y, v3Min.z);
            glVertex3f(v3Max.x, v3Max.y, v3Max.z);
            glColor3f(1.0f, 1.0f, 1.0f);
        }
        glEnd();
    }

    // Draw hits
    std::vector < structHits >* pListHits = m_physics.GetHits();
    for (int32_t i = 0; i < pListHits->size(); i++)
    {
        structHits hits = pListHits->at(i);

        for (int32_t j = 0; j < hits.m_nNumHits; j++)
        {
            structVector3 p1 = hits.m_hits[j].m_v3HitPointInWorld;
            structVector3 p2 = hits.m_hits[j].m_v3Normal;

            glPointSize(20.0f);

            glBegin(GL_POINTS);
            {
                glColor3f(1.0f, 0.0f, 0.0f);
                glVertex3f(p1.x, p1.y, p1.z);
                glColor3f(1.0f, 1.0f, 1.0f);
            }
            glEnd();

            glBegin(GL_LINES);
            {
                glColor3f(0.0f, 1.0f, 0.0f);
                glVertex3f(p1.x, p1.y, p1.z);
                glVertex3f(p1.x + p2.x, p1.y + p2.y, p1.z + p2.z);
                glColor3f(1.0f, 1.0f, 1.0f);
            }
            glEnd();
        }
    }

    m_SkyBox.Draw(v3CameraPos, 5000.0f);

    glutPrint(0, 24, (char*)strTitle.c_str());
    glutPrint(0, 48, (char*)"Exit: [ESC]");

	glutSwapBuffers();
}

void IdleFunc() 
{
	glutPostRedisplay();
}

int main() 
{
	glutInit(&__argc, __argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	/* add command line argument "classic" for a pre-3.x context */
	glutInitContextVersion(2, 0);
	glutInitContextFlags(GLUT_FORWARD_COMPATIBLE);
	glutInitWindowSize(640, 480);
	glutInitWindowPosition(10, 10);
	nMainWindowId = glutCreateWindow("Loading...");

	if (GLEW_OK != glewInit()) 
	{
		return EXIT_FAILURE;
	}

	if (false == glewIsSupported("GL_VERSION_3_3"))
	{
		return EXIT_FAILURE;
	}

    glutFullScreen();

	if (false == Init()) 
	{
		return EXIT_FAILURE;
	}

	glutDisplayFunc(DisplayFunc);
	glutReshapeFunc(ReshapeFunc);
	glutKeyboardFunc(KeyboardDownFunc);
	glutKeyboardUpFunc(KeyboardUpFunc);
	glutMouseFunc(MouseFunc);
	glutMotionFunc(MouseMotionFunc);
	glutPassiveMotionFunc(MousePassiveMotionFunc);
	glutIdleFunc(IdleFunc);

	glutMainLoop();

	return EXIT_SUCCESS;
}