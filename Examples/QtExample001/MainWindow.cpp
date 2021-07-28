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
    static_id = m_physics.GenStaticMesh();
    m_physics.SetStaticMesh(static_id, m_staticmodel.GetAllVertices(), m_staticmodel.GetAllNormals());

    // dynamic
    m_dynamicmodel.Load("Scene", "barrel.obj", glm::mat4(1.0f), false);
    dynamic_id = m_physics.GenDynamicMesh();
    m_physics.SetDynamicMesh(dynamic_id, m_dynamicmodel.GetAllVertices(), m_dynamicmodel.GetAllNormals(), 0.1f, 0.1f);
}
