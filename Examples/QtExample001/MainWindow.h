#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_MainWindow.h"

#include "Model.h"

#include "OpenCLPhysics.h"
using namespace OpenCLPhysics;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = Q_NULLPTR);

private:
    Ui::MainWindowClass ui;

    Physics m_physics;
    int static_id;
    int dynamic_id;

    Model m_staticmodel;
    Model m_dynamicmodel;
};
