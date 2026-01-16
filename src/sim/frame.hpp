// Frame: A coordinate frame that can be attached to arm joints or world.
// Used for IK targets, reference points, etc.
#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

class Frame
{
public:
    Frame(const std::string& name, unsigned int id);
    
    // Transform accessors
    Eigen::Isometry3d getWorldTransform() const;
    const Eigen::Isometry3d& getLocalTransform() const { return localTransform; }
    
    // Parent: raw pointer to Arm's tableLinkTransforms entry (same Sim thread, no ownership)
    void setParentTransform(const Eigen::Isometry3d* pTransform);
    void clearParent();  // Attach to world origin
    bool hasParent() const { return parentTransform != nullptr; }
    
    // Local transform modification
    void setLocalTransform(const Eigen::Isometry3d& lTransform);
    void setLocalPosition(const Eigen::Vector3d& pos);
    void setLocalRotation(const Eigen::Quaterniond& rot);
    
    // Identity accessors
    const std::string& getName() const { return name; }
    unsigned int getId() const { return id; }
    void setName(const std::string& n) { name = n; }
    
    // Parent info for UI (which arm/joint this is attached to)
    void setParentInfo(int armIndex, int jointIndex);
    int getParentArmIndex() const { return parentArmIndex; }
    int getParentJointIndex() const { return parentJointIndex; }
    
private:
    std::string name;
    unsigned int id;
    
    const Eigen::Isometry3d* parentTransform = nullptr;  // Points into Arm's live table (same thread)
    int parentArmIndex = -1;   // -1 = world, else arm index
    int parentJointIndex = -1; // -1 = world/base, else joint index
    
    Eigen::Isometry3d localTransform = Eigen::Isometry3d::Identity();
};