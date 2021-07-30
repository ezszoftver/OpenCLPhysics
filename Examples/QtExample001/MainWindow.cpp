#include "MainWindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);

    std::vector<std::string> listDevices = m_physics.GetDevices();
    if (listDevices.size() == 0) 
    {
        QApplication::exit(1);
        return;
    }
    
    if (false == m_physics.CreateDevice(listDevices[0]))
    {
        QApplication::exit(1);
        return;
    }

    // static
    m_staticmodel.Load("Scene", "Scene.obj", glm::mat4(1.0f), false);
    static_id = m_physics.GenTriMesh();
    m_physics.SetTriMesh(static_id, m_staticmodel.GetAllVertices(), m_staticmodel.GetAllNormals());
    m_physics.SetMass(static_id, 0.0f); // static

    // dynamic
    m_dynamicmodel.Load("Scene", "barrel.obj", glm::mat4(1.0f), false);
    dynamic_id = m_physics.GenTriMesh();
    m_physics.SetTriMesh(dynamic_id, m_dynamicmodel.GetAllVertices(), m_dynamicmodel.GetAllNormals());
    m_physics.SetMass(dynamic_id, 85.0f); // dynamic
}
