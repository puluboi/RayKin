#include "frame.hpp"

Frame::Frame(const std::string& name, unsigned int id)
    : name(name), id(id)
{
}

Eigen::Isometry3d Frame::getWorldTransform() const
{
    if (parentTransform) {
        return *parentTransform * localTransform;
    }
    return localTransform;  // World frame: local = world
}

void Frame::setParentTransform(const Eigen::Isometry3d* pTransform)
{
    parentTransform = pTransform;
}

void Frame::clearParent()
{
    parentTransform = nullptr;
    parentArmIndex = -1;
    parentJointIndex = -1;
}

void Frame::setLocalTransform(const Eigen::Isometry3d& lTransform)
{
    localTransform = lTransform;
}

void Frame::setLocalPosition(const Eigen::Vector3d& pos)
{
    localTransform.translation() = pos;
}

void Frame::setLocalRotation(const Eigen::Quaterniond& rot)
{
    localTransform.linear() = rot.toRotationMatrix();
}

void Frame::setParentInfo(int armIndex, int jointIndex)
{
    parentArmIndex = armIndex;
    parentJointIndex = jointIndex;
}