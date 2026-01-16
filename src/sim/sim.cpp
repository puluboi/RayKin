#include "sim.hpp"
#include <fstream>
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <algorithm>

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

            // Update all arms
            for (size_t i = 0; i < arms.size(); i++)
            {
                if (arms[i])
                {
                    arms[i]->updateArm();
                }
            }
            
            // Perform IK for arms with target frames
            updateIK();
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
        if (!arm)
            continue;

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
    std::lock_guard<std::mutex> lockFrames(framesMutex);
    
    auto snap = std::make_shared<SimSnapshot>();
    snap->armTransforms.reserve(arms.size());
    for (const auto &arm : arms)
    {
        if (arm)
        {
            snap->armTransforms.push_back(arm->fetchLinkTransformations());
        }
    }
    
    // Copy frame data for visualization
    snap->frames.reserve(frames.size());
    for (const auto& frame : frames)
    {
        if (frame)
        {
            FrameSnapshot fs;
            fs.id = frame->getId();
            fs.name = frame->getName();
            fs.worldTransform = frame->getWorldTransform();
            fs.parentArmIndex = frame->getParentArmIndex();
            fs.parentJointIndex = frame->getParentJointIndex();
            snap->frames.push_back(fs);
        }
    }
    
    // Copy arm target frame IDs
    snap->armTargetFrameIds = armTargetFrameIds;
    
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

// ============================================================================
// Frame Management
// ============================================================================

unsigned int Sim::createFrame(const std::string& name)
{
    std::lock_guard<std::mutex> lock(framesMutex);
    unsigned int id = nextFrameId++;
    frames.push_back(std::make_unique<Frame>(name, id));
    return id;
}

bool Sim::destroyFrame(unsigned int frameId)
{
    std::lock_guard<std::mutex> lock(framesMutex);
    auto it = std::find_if(frames.begin(), frames.end(),
        [frameId](const std::unique_ptr<Frame>& f) { return f && f->getId() == frameId; });
    
    if (it != frames.end())
    {
        // Clear any arm targets pointing to this frame
        std::lock_guard<std::mutex> lockArms(armsMutex);
        for (auto& targetId : armTargetFrameIds)
        {
            if (targetId == static_cast<int>(frameId))
            {
                targetId = -1;
            }
        }
        frames.erase(it);
        return true;
    }
    return false;
}

bool Sim::setFrameParent(unsigned int frameId, int armIndex, int jointIndex)
{
    std::lock_guard<std::mutex> lockFrames(framesMutex);
    std::lock_guard<std::mutex> lockArms(armsMutex);
    
    auto it = std::find_if(frames.begin(), frames.end(),
        [frameId](const std::unique_ptr<Frame>& f) { return f && f->getId() == frameId; });
    
    if (it == frames.end()) return false;
    
    Frame* frame = it->get();
    
    if (armIndex < 0 || jointIndex < 0)
    {
        // Attach to world
        frame->clearParent();
        return true;
    }
    
    if (static_cast<size_t>(armIndex) >= arms.size() || !arms[armIndex])
        return false;
    
    auto transforms = arms[armIndex]->fetchLinkTransformations();
    if (static_cast<size_t>(jointIndex) >= transforms.size())
        return false;
    
    // Get raw pointer to arm's internal transform table
    // This works because Frame is read on Sim thread, same as Arm updates
    frame->setParentInfo(armIndex, jointIndex);
    rebindFrameParents();  // Rebind to actual internal pointers
    
    return true;
}

bool Sim::setFrameLocalPosition(unsigned int frameId, const Eigen::Vector3d& pos)
{
    std::lock_guard<std::mutex> lock(framesMutex);
    auto it = std::find_if(frames.begin(), frames.end(),
        [frameId](const std::unique_ptr<Frame>& f) { return f && f->getId() == frameId; });
    
    if (it != frames.end())
    {
        (*it)->setLocalPosition(pos);
        return true;
    }
    return false;
}

bool Sim::setFrameLocalRotation(unsigned int frameId, const Eigen::Quaterniond& rot)
{
    std::lock_guard<std::mutex> lock(framesMutex);
    auto it = std::find_if(frames.begin(), frames.end(),
        [frameId](const std::unique_ptr<Frame>& f) { return f && f->getId() == frameId; });
    
    if (it != frames.end())
    {
        (*it)->setLocalRotation(rot);
        return true;
    }
    return false;
}

bool Sim::setFrameLocalRotationEuler(unsigned int frameId, double rx, double ry, double rz)
{
    // Convert Euler angles (XYZ order) to quaternion
    Eigen::Quaterniond rot = Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX())
                           * Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY())
                           * Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ());
    return setFrameLocalRotation(frameId, rot);
}

bool Sim::setFrameName(unsigned int frameId, const std::string& name)
{
    std::lock_guard<std::mutex> lock(framesMutex);
    auto it = std::find_if(frames.begin(), frames.end(),
        [frameId](const std::unique_ptr<Frame>& f) { return f && f->getId() == frameId; });
    
    if (it != frames.end())
    {
        (*it)->setName(name);
        return true;
    }
    return false;
}

std::vector<std::pair<unsigned int, std::string>> Sim::getFrameList() const
{
    std::lock_guard<std::mutex> lock(framesMutex);
    std::vector<std::pair<unsigned int, std::string>> list;
    list.reserve(frames.size());
    for (const auto& frame : frames)
    {
        if (frame)
        {
            list.emplace_back(frame->getId(), frame->getName());
        }
    }
    return list;
}

// ============================================================================
// IK Target Management
// ============================================================================

bool Sim::setArmTargetFrame(unsigned int armIndex, int frameId)
{
    std::lock_guard<std::mutex> lock(armsMutex);
    if (armIndex >= arms.size())
        return false;
    
    if (armIndex >= armTargetFrameIds.size())
    {
        armTargetFrameIds.resize(armIndex + 1, -1);
    }
    
    armTargetFrameIds[armIndex] = frameId;
    return true;
}

int Sim::getArmTargetFrame(unsigned int armIndex) const
{
    std::lock_guard<std::mutex> lock(armsMutex);
    if (armIndex < armTargetFrameIds.size())
    {
        return armTargetFrameIds[armIndex];
    }
    return -1;
}

void Sim::updateIK()
{
    // Called within armsMutex lock from run()
    std::lock_guard<std::mutex> lockFrames(framesMutex);
    
    for (size_t i = 0; i < arms.size(); ++i)
    {
        if (!arms[i]) continue;
        if (i >= armTargetFrameIds.size() || armTargetFrameIds[i] < 0) continue;
        
        int targetId = armTargetFrameIds[i];
        
        // Find target frame
        auto it = std::find_if(frames.begin(), frames.end(),
            [targetId](const std::unique_ptr<Frame>& f) { 
                return f && f->getId() == static_cast<unsigned int>(targetId); 
            });
        
        if (it == frames.end()) continue;
        
        Eigen::Isometry3d targetTransform = (*it)->getWorldTransform();
        Eigen::Vector3d target_pos = targetTransform.translation();
        Eigen::Quaterniond target_ori(targetTransform.rotation());
        
        // Get current end-effector position
        auto transforms = arms[i]->fetchLinkTransformations();
        if (transforms.empty()) continue;
        
        Eigen::Vector3d current_pos = transforms.back().translation();
        double pos_error = (target_pos - current_pos).norm();
        
        // Adaptive damping: high far away (stable), low near target (precise)
        double lambda_max = 0.5;
        double lambda_min = 0.01;
        double error_threshold = 0.5;
        
        double lambda = lambda_min + (lambda_max - lambda_min) * 
                        std::min(1.0, pos_error / error_threshold);
        
        // Skip if close enough
        if (pos_error < 0.005) continue;
        
        Eigen::VectorXd dq = arms[i]->computeDLSStep(target_pos, target_ori, lambda);
        
        // Larger step when close (fine-tuning)
        double step_scale = (pos_error < 0.1) ? 0.1 : 0.02;
        
        for (int j = 0; j < dq.size(); ++j)
        {
            arms[i]->incrementTargetAngle(j, dq(j) * step_scale);
        }
    }
}

void Sim::rebindFrameParents()
{
    // Called with both mutexes held
    // Rebinds frame parent pointers to Arm's internal tableLinkTransforms
    // Note: This requires Arm to expose internal transform pointers
    // For now, frames read transforms via getWorldTransform() which uses stored indices
    
    // Future optimization: expose Arm::getTransformPtr(index) for direct pointer access
}