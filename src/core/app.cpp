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
    
    // Connect increment callback (uses Arm::incrementTargetAngle)
    viz.setIncrementJointCallback([this](unsigned int armIdx, unsigned int jointIdx, double delta) {
        auto arm = sim.getArm(armIdx);
        if (arm) {
            arm->incrementTargetAngle(jointIdx, delta);
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
    
    // Connect frame management callbacks
    viz.setCreateFrameCallback([this](const std::string& name) -> unsigned int {
        return sim.createFrame(name);
    });
    
    viz.setDestroyFrameCallback([this](unsigned int frameId) -> bool {
        return sim.destroyFrame(frameId);
    });
    
    viz.setFrameParentCallback([this](unsigned int frameId, int armIdx, int jointIdx) -> bool {
        return sim.setFrameParent(frameId, armIdx, jointIdx);
    });
    
    viz.setFramePositionCallback([this](unsigned int frameId, double x, double y, double z) -> bool {
        return sim.setFrameLocalPosition(frameId, Eigen::Vector3d(x, y, z));
    });
    
    viz.setFrameNameCallback([this](unsigned int frameId, const std::string& name) -> bool {
        return sim.setFrameName(frameId, name);
    });
    
    viz.setGetFrameListCallback([this]() -> std::vector<std::pair<unsigned int, std::string>> {
        return sim.getFrameList();
    });
    
    viz.setArmTargetFrameCallback([this](unsigned int armIdx, int frameId) -> bool {
        return sim.setArmTargetFrame(armIdx, frameId);
    });
    
    viz.setGetArmTargetFrameCallback([this](unsigned int armIdx) -> int {
        return sim.getArmTargetFrame(armIdx);
    });
    
    viz.setFrameRotationCallback([this](unsigned int frameId, double rx, double ry, double rz) -> bool {
        return sim.setFrameLocalRotationEuler(frameId, rx, ry, rz);
    });
    
    viz.setGetArmNumJointsCallback([this](unsigned int armIdx) -> unsigned int {
        auto arm = sim.getArm(armIdx);
        if (arm) {
            return arm->getNumLinks();
        }
        return 0;
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