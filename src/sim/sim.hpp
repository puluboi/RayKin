#pragma once
#include "arm.hpp"
#include <vector>
#include <memory>
#include <atomic>
#include <mutex>
#pragma once
#include "arm.hpp"
#include <vector>
#include <memory>
#include <atomic>
#include <mutex>

struct SimSnapshot
{
    std::vector<std::vector<Eigen::Isometry3d>> armTransforms;
};
class Sim
{
public:
// Constructor
    Sim();
    // Run function, which is called from App.cpp.
    void run();

    // Sets the "running" bool to false, stopping the simulation.
    void stopSim();
    std::shared_ptr<SimSnapshot> fetchSimData();
    void formSimData();
    /**
     * @brief Creates an Arm from a URDF file.
     * @param urdfPath Path to the URDF file.
     * @param worldPos World position of the arm base.
     * @param worldRot World orientation of the arm base.
     * @param armIndex Index in the arms vector (creates/replaces).
     * @return true on success, false on error (logged to stderr).
     *
     * Thread-safe: uses internal mutex for arm storage.
     */
    bool createArm(const std::string &urdfPath,
                   const Eigen::Vector3d &worldPos,
                   const Eigen::Quaterniond &worldRot,
                   unsigned int armIndex = 0);

    /// Get arm pointer (nullptr if not exists). Thread-safe read.
    std::shared_ptr<Arm> getArm(unsigned int index = 0) const;

    /// Enable/disable test swing animation
    void setTestSwingEnabled(bool enabled);

private:
    /// Test function that swings arm joints using setTargetAngle
    void testSwingArms(double time);
    
    // when true: sim running, when false: App stops 
    bool running;
    bool testSwingEnabled;
    std::vector<std::shared_ptr<Arm>> arms;
    mutable std::mutex armsMutex; 
    SimSnapshot snapshot;
    
};
