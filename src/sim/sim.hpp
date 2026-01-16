#pragma once
#include "arm.hpp"
#include "frame.hpp"
#include <vector>
#include <memory>
#include <atomic>
#include <mutex>

// Snapshot of a single frame for visualization (cross-thread)
struct FrameSnapshot
{
    unsigned int id;
    std::string name;
    Eigen::Isometry3d worldTransform;
    int parentArmIndex;   // -1 = world
    int parentJointIndex; // -1 = world/base
};

struct SimSnapshot
{
    std::vector<std::vector<Eigen::Isometry3d>> armTransforms;
    std::vector<FrameSnapshot> frames;
    std::vector<int> armTargetFrameIds;  // -1 if no target, else frame ID for each arm
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
    
    // Frame management (thread-safe via mutex)
    unsigned int createFrame(const std::string& name);
    bool destroyFrame(unsigned int frameId);
    bool setFrameParent(unsigned int frameId, int armIndex, int jointIndex);
    bool setFrameLocalPosition(unsigned int frameId, const Eigen::Vector3d& pos);
    bool setFrameLocalRotation(unsigned int frameId, const Eigen::Quaterniond& rot);
    bool setFrameLocalRotationEuler(unsigned int frameId, double rx, double ry, double rz);  // Euler angles (radians)
    bool setFrameName(unsigned int frameId, const std::string& name);
    std::vector<std::pair<unsigned int, std::string>> getFrameList() const;  // ID, name pairs
    
    // IK target assignment
    bool setArmTargetFrame(unsigned int armIndex, int frameId);  // -1 to clear target
    int getArmTargetFrame(unsigned int armIndex) const;

private:
    /// Test function that swings arm joints using setTargetAngle
    void testSwingArms(double time);
    
    /// Perform IK step for arms with target frames
    void updateIK();
    
    /// Update frame parent pointers after arm changes
    void rebindFrameParents();
    
    // when true: sim running, when false: App stops 
    bool running;
    bool testSwingEnabled;
    std::vector<std::shared_ptr<Arm>> arms;
    std::vector<int> armTargetFrameIds;  // -1 = no target, else frame ID
    mutable std::mutex armsMutex; 
    SimSnapshot snapshot;
    
    // Frame storage
    std::vector<std::unique_ptr<Frame>> frames;
    unsigned int nextFrameId = 1;  // Auto-increment ID
    mutable std::mutex framesMutex;
};
