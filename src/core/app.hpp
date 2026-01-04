#pragma once

#include "viz.hpp"
#include "sim.hpp"
#include "vector"
#include <thread>
#include <iostream>
class App
{
public:
    App();
    void update();

    ~App();

private:

    Viz viz;
    bool running = true;
    std::thread simThread;
    Sim sim;
     
};