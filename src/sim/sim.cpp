#include "sim.hpp"
#include <fstream>
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

Sim::Sim() : running(true), testSwingEnabled(true)
{
}

void Sim::run()
{
    createArm("assets/URDF_exerc.xml", Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond::Identity());
    
    auto startTime = std::chrono::steady_clock::now();
    setTestSwingEnabled(false);
    while (running)
    {
        // Calculate elapsed time for animation
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - startTime).count();
        {
            std::lock_guard<std::mutex> lock(armsMutex);
            
            // Test swing using setTargetAngle
            if (testSwingEnabled)
            {
                testSwingArms(elapsed);
            }
            
            for (size_t i = 0; i < arms.size(); i++)
            {
                if (arms[i])
                {
                    arms[i]->updateArm();
                }
            }
        }
        
        // Small sleep to prevent busy loop (roughly 120Hz update)
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
    }
}

void Sim::testSwingArms(double time)
{
    // Alternate between pi/4 and -pi/4 every 2 seconds
    constexpr double interval = 2.0;
    constexpr double angle1 = M_PI / 4.0;
    constexpr double angle2 = -M_PI / 4.0;
    
    // Determine which angle based on time interval
    int cycle = static_cast<int>(time / interval);
    double targetAngle = (cycle % 2 == 0) ? angle1 : angle2;
    
    for (size_t armIdx = 0; armIdx < arms.size(); ++armIdx)
    {
        auto &arm = arms[armIdx];
        if (!arm) continue;
        
        auto transforms = arm->fetchLinkTransformations();
        size_t numJoints = transforms.size();
        
        for (size_t joint = 0; joint < numJoints; ++joint)
        {
            int var = (joint % 2 == 0 ? 1 : -1);
            arm->setTargetAngle(static_cast<unsigned int>(joint), var * targetAngle);
        }
    }
}

void Sim::setTestSwingEnabled(bool enabled)
{
    testSwingEnabled = enabled;
}




void Sim::stopSim()
{
    running = false;
}

std::shared_ptr<SimSnapshot> Sim::fetchSimData()
{
    std::lock_guard<std::mutex> lock(armsMutex);
    auto snap = std::make_shared<SimSnapshot>();
    snap->armTransforms.reserve(arms.size());
    for (const auto &arm : arms)
    {
        if (arm)
        {
            snap->armTransforms.push_back(arm->fetchLinkTransformations());
        }
    }
    return snap;
}

void Sim::formSimData()
{
    std::lock_guard<std::mutex> lock(armsMutex);
    snapshot.armTransforms.clear();
    snapshot.armTransforms.reserve(arms.size());
    for (const auto &arm : arms)
    {
        if (arm)
        {
            snapshot.armTransforms.push_back(arm->fetchLinkTransformations());
        }
    }
}

bool Sim::createArm(const std::string &urdfPath, const Eigen::Vector3d &worldPos, const Eigen::Quaterniond &worldRot, unsigned int armIndex)
{
    std::ifstream ifs(urdfPath);
    if (!ifs.is_open() || ifs.fail())
    {
        std::cerr << "Failed to open URDF file: " << urdfPath << '\n';
        return false;
    }

    std::lock_guard<std::mutex> lock(armsMutex);
    if (armIndex >= arms.size())
    {
        arms.resize(armIndex + 1);
    }
    arms[armIndex] = std::make_shared<Arm>(worldPos, worldRot, urdfPath);
    return true;
}

std::shared_ptr<Arm> Sim::getArm(unsigned int index) const
{
    std::lock_guard<std::mutex> lock(armsMutex);
    if (index < arms.size())
    {
        return arms[index];
    }
    return nullptr;
}
