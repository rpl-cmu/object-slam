/******************************************************************************
 * File:             visualizer.cpp
 *
 * Author:           Akash Sharma
 * Created:          05/10/20
 * Description:      Visualizer class
 *****************************************************************************/
#include "visualizer.h"
#include <spdlog/spdlog.h>

namespace oslam {

Visualizer::Visualizer()
  : mp_visualizer(std::make_unique<open3d::visualization::Visualizer>())
{
    bool success = mp_visualizer->CreateVisualizerWindow("Visualizer", 1600, 900);
    if(!success)
        spdlog::error("Unable to create visualizer window");
}

Visualizer::~Visualizer() { mp_visualizer->Close(); mp_visualizer->DestroyVisualizerWindow();}

bool Visualizer::start()
{
    mp_thread = std::make_unique<std::thread>(&Visualizer::run, this);
    if(mp_thread)
        return true;
    return false;
}

void Visualizer::add_geometry(std::shared_ptr<open3d::geometry::Geometry> p_geometry)
{
    mp_visualizer->AddGeometry(p_geometry);
}

void Visualizer::remove_geometry(std::shared_ptr<open3d::geometry::Geometry> p_geometry)
{
    mp_visualizer->RemoveGeometry(p_geometry);
}
void Visualizer::run(void)
{
    while(mp_visualizer->PollEvents())
    {
        mp_visualizer->UpdateGeometry();
        mp_visualizer->UpdateRender();
    }
}


}// namespace oslam
