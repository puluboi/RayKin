#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <memory>
#include <atomic>
#include <mutex>
#include <string>
#include <algorithm>
struct MDH
{
    double aPrev;     // X-axis translation compared to previous link.
    double alphaPrev; // X-axis rotation compared to previous link.

    double thetaCurr;      // Z-axis rotation compared to previous link. mutable.
    double tCOffset; // offset to current theta

    double dCurr;          // Z-axis translation compared to previous link. mutable.
    double dCOffset; // offset to current d

    bool isRevolute; // False if prismatic

    double upperThetaConstraint;
    double lowerThetaConstraint;

    double upperDConstraint;
    double lowerDConstraint;



    MDH(double aPrev_, double alphaPrev_, double thetaCurr_, double dCurr_, double tCOffset_, double dCOffset_, double lowerThetaConstraint_, double upperThetaConstraint_, bool isRevolute_)
        : aPrev(aPrev_), alphaPrev(alphaPrev_), thetaCurr(thetaCurr_), dCurr(dCurr_), tCOffset(tCOffset_), dCOffset(dCOffset_), lowerThetaConstraint(lowerThetaConstraint_), upperThetaConstraint(upperThetaConstraint_), isRevolute(isRevolute_)
    {

    }
};

/// Snapshot of a single MDH row for UI display
struct MDHRow
{
    double alpha;       // α_{i-1}
    double a;           // a_{i-1}
    double d;           // d_i (current value)
    double theta;       // θ_i (current value)
    double thetaOffset; // θ offset
    double dOffset;     // d offset
    double maxTheta;    // max θ_i
    double minTheta;    // min θ_i
    bool isRevolute;    // R or P
};

class Arm
{
public:
// Inverse Kinematics
Eigen::VectorXd computeDLSStep(const Eigen::Vector3d &targetPos, const Eigen::Quaterniond &targetOri, double lambda) const;

    Arm(Eigen::Vector3d origWorldPos_, Eigen::Quaterniond origWorldRot_, const std::string pathToURDF);
    bool incrementTargetAngle(unsigned int link, double angle);
    bool setTargetAngle(unsigned int link, double rot);

    /// Get current angle of a specific joint in radians
    double getCurrentAngle(unsigned int link) const;
    
    /// Get the MDH table for UI display (thread-safe copy)
    std::vector<MDHRow> getMDHTable() const;
    
    /// Set MDH parameter for a specific link (triggers recalculation)
    bool setMDHParam(unsigned int link, const std::string& param, double value);
    
    /// Get number of links
    unsigned int getNumLinks() const { return noOfLinks; }

    /// @brief updates the links.
    /// @return
    bool updateArm();

    // Fetch link positions
    const std::vector<Eigen::Isometry3d> fetchLinkTransformations() const;

private:
    bool setLinkAngle(unsigned int link, double angle);
    bool addLinkAngle(unsigned int link, double angle);

    Eigen::MatrixXd computeJacobian(unsigned int link) const;

    
    bool setLinkLength(unsigned int link, double length);
    bool addLinkLength(unsigned int link, double length);

    unsigned int noOfLinks;
    unsigned int lowestChangedLink;
    // Link Tables
    std::vector<MDH> tableMDH;
    std::vector<Eigen::Isometry3d> tableLinkTransforms;
    
    std::vector<double> targetAngles;
    std::vector<unsigned int> refreshLinks; // Link's whose target angles differ from their current angles
    mutable std::mutex refreshMutex;
    Eigen::Vector3d origWorldPos;
    Eigen::Quaterniond origWorldRot;

    mutable std::mutex transformsMutex; // protect internal state
    std::shared_ptr<const std::vector<Eigen::Isometry3d>> publishedTransforms;
    mutable std::mutex publishMutex; // protect publishedTransforms
};