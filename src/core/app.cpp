#include "app.hpp"

App::App()
    : viz(45, 1080, 1080)
{
    // Disable ESC key as exit key (only X button closes window)
    SetExitKey(0);
    
    // Connect simulation data to visualization
    viz.setSimDataCallback([this]() -> std::shared_ptr<SimSnapshot>
                           { return sim.fetchSimData(); });
    
    // Connect visualization controls to simulation
    viz.setSimControlCallback([this](unsigned int armIdx, unsigned int jointIdx, double angle) {
        auto arm = sim.getArm(armIdx);
        if (arm) {
            arm->setTargetAngle(jointIdx, angle);
        }
    });
    
    // Connect angle getter to simulation
    viz.setGetAngleCallback([this](unsigned int armIdx, unsigned int jointIdx) -> double {
        auto arm = sim.getArm(armIdx);
        if (arm) {
            return arm->getCurrentAngle(jointIdx);
        }
        return 0.0;
    });
    
    // Connect MDH table getter to simulation
    viz.setGetMDHTableCallback([this](unsigned int armIdx) -> std::vector<MDHRow> {
        auto arm = sim.getArm(armIdx);
        if (arm) {
            return arm->getMDHTable();
        }
        return {};
    });
    
    // Connect MDH parameter setter to simulation
    viz.setMDHControlCallback([this](unsigned int armIdx, unsigned int link, 
                                      const std::string& param, double value) -> bool {
        auto arm = sim.getArm(armIdx);
        if (arm) {
            return arm->setMDHParam(link, param, value);
        }
        return false;
    });

    simThread = std::thread([this]()
                            { sim.run(); });
    
    while (!WindowShouldClose())
    {
        viz.update();
    }

    running = false;
    sim.stopSim();
    if (simThread.joinable())
        simThread.join();
}

void App::update()
{
}

App::~App()
{
    running = false;
    sim.stopSim();
    if (simThread.joinable())
        simThread.join();
}