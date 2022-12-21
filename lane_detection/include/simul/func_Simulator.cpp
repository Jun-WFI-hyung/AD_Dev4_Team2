#include "simul/Simulator.h"

Simulator::Simulator(int width, int height, int fps) 
{
    cam_width = width;
    cam_height = height;
    cam_fps = fps;
}

Simulator::~Simulator() 
{
}