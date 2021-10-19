#ifndef SKYBOX_H
#define SKYBOX_H

#include "GL/glew.h"
#include "GL/wglew.h"
#include "glm/glm.hpp"
#include "glm/ext.hpp"

#include "Texture.h"

#include <string>

class SkyBox
{
public:
    SkyBox();

    void Load(std::string strDir, std::string strBack, std::string strDown, std::string strFront, std::string strLeft, std::string strRight, std::string strUp);
    void Draw(glm::vec3 v3Center, float fZFar = 100.0f);
    void Release();

private:
    Texture texture[6];
    glm::vec3 vertices[36];
    glm::vec2 textures[36];
};

#endif // SKYBOX_H
