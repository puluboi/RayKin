#include "arm.hpp"
#include <tinyxml2.h>
#include <iostream>

using namespace tinyxml2;

Arm::Arm(Eigen::Vector3d origWorldPos_, Eigen::Quaterniond origWorldRot_, std::string pathToURDF)
{
    lowestChangedLink = 0;
    origWorldPos = origWorldPos_;
    origWorldRot = origWorldRot_;

    XMLDocument doc;
    XMLError eResult = doc.LoadFile(pathToURDF.c_str());
    if (eResult != XML_SUCCESS)
    {
        std::cerr << "Error loading URDF file: " << pathToURDF << std::endl;
        return;
    }

    XMLElement *robot = doc.FirstChildElement("robot");
    if (!robot)
        return;

    XMLElement *joint = robot->FirstChildElement("joint");
    while (joint)
    {
        XMLElement *mdh = joint->FirstChildElement("mdh");
        XMLElement *limit = joint->FirstChildElement("limit");

        if (mdh)
        {
            double a = 0, alpha = 0, dOffset = 0, theta_offset = 0, maxTheta = 0, minTheta = -0;
            mdh->QueryDoubleAttribute("a", &a);
            mdh->QueryDoubleAttribute("alpha", &alpha);
            mdh->QueryDoubleAttribute("d", &dOffset);
            if (limit)
            {
                limit->QueryDoubleAttribute("lower", &minTheta);
                limit->QueryDoubleAttribute("upper", &maxTheta);
            }

            bool isRevolute = true;
            const char *type = joint->Attribute("type");
            if (type && std::string(type) == "prismatic")
            {
                isRevolute = false;
            }

            //               MDH(aPrev, alphaPrev, thetaCurr, dCurr, tCOffset, dCOffset,minTheta, maxTheta, isRevolute)
            tableMDH.emplace_back(a, alpha, 0.0, 0.0, theta_offset, dOffset, minTheta, maxTheta, isRevolute);
        }
        joint = joint->NextSiblingElement("joint");
    }

    noOfLinks = tableMDH.size();

    tableLinkTransforms.resize(noOfLinks, Eigen::Isometry3d::Identity());
    targetAngles.resize(noOfLinks, 0.0); // Initialize target angles to 0
}

bool Arm::incrementTargetAngle(unsigned int link, double angle)
{
    if (link >= targetAngles.size())
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(refreshMutex);

    // Compute new target and clamp to constraints
    double newTarget = targetAngles[link] + angle;
    if (newTarget > tableMDH[link].upperThetaConstraint)
    {
        targetAngles[link] = tableMDH[link].upperThetaConstraint;
    }
    else if (newTarget < tableMDH[link].lowerThetaConstraint)
    {
        targetAngles[link] = tableMDH[link].lowerThetaConstraint;
    }
    else
    {
        targetAngles[link] = newTarget;
    }

    // Only add to refreshLinks if not already present (prevent duplicates)
    if (std::find(refreshLinks.begin(), refreshLinks.end(), link) == refreshLinks.end())
    {
        refreshLinks.push_back(link);
    }

    // Mark this link as needing update so updateArm() doesn't skip it
    if (lowestChangedLink > link)
    {
        lowestChangedLink = link;
    }
    return true;
}

bool Arm::setTargetAngle(unsigned int link, double angle)
{

    if (link >= targetAngles.size())
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(refreshMutex);
    if (angle > tableMDH[link].upperThetaConstraint)
    {
        targetAngles[link] = tableMDH[link].upperThetaConstraint;
    }
    else if (angle < tableMDH[link].lowerThetaConstraint)
    {
        targetAngles[link] = tableMDH[link].lowerThetaConstraint;
    }
    else
    {
        targetAngles[link] = angle;
    }

    // Only add to refreshLinks if not already present (prevent duplicates)
    if (std::find(refreshLinks.begin(), refreshLinks.end(), link) == refreshLinks.end())
    {
        refreshLinks.push_back(link);
    }

    // Mark this link as needing update so updateArm() doesn't skip it
    if (lowestChangedLink > link)
    {
        lowestChangedLink = link;
    }
    return true;
}

double Arm::getCurrentAngle(unsigned int link) const
{
    if (link >= tableMDH.size())
    {
        return 0.0;
    }
    return tableMDH[link].thetaCurr + tableMDH[link].tCOffset;
}

std::vector<MDHRow> Arm::getMDHTable() const
{
    std::lock_guard<std::mutex> lock(refreshMutex);
    std::vector<MDHRow> rows;
    rows.reserve(tableMDH.size());

    for (const auto &mdh : tableMDH)
    {
        MDHRow row;
        row.alpha = mdh.alphaPrev;
        row.a = mdh.aPrev;
        row.d = mdh.dCurr;
        row.theta = mdh.thetaCurr;
        row.thetaOffset = mdh.tCOffset;
        row.dOffset = mdh.dCOffset;
        row.isRevolute = mdh.isRevolute;
        row.maxTheta = mdh.upperThetaConstraint;
        row.minTheta = mdh.lowerThetaConstraint;
        rows.push_back(row);
    }
    return rows;
}

bool Arm::setMDHParam(unsigned int link, const std::string &param, double value)
{
    if (link >= tableMDH.size())
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(refreshMutex);

    // All MDH params are now modifiable
    if (param == "alpha")
    {
        tableMDH[link].alphaPrev = value;
    }
    else if (param == "a")
    {
        tableMDH[link].aPrev = value;
    }
    else if (param == "d")
    {
        tableMDH[link].dCurr = value;
    }
    else if (param == "d_offset")
    {
        tableMDH[link].dCOffset = value;
    }
    else if (param == "theta")
    {
        // Set target angle (will be interpolated by P-controller)
        targetAngles[link] = value;
        if (std::find(refreshLinks.begin(), refreshLinks.end(), link) == refreshLinks.end())
        {
            refreshLinks.push_back(link);
        }
    }
    else if (param == "theta_offset")
    {
        tableMDH[link].tCOffset = value;
    }
    else if (param == "minTheta")
    {
        tableMDH[link].lowerThetaConstraint = value;
    }
    else if (param == "maxTheta")
    {
        tableMDH[link].upperThetaConstraint = value;
    }
    else
    {
        std::cerr << "Unknown MDH param: " << param << std::endl;
        return false;
    }

    if (lowestChangedLink > link)
    {
        lowestChangedLink = link;
    }
    return true;
}

bool Arm::updateArm()
{
    // Validate sizes
    if (tableMDH.size() < noOfLinks || tableLinkTransforms.size() < noOfLinks)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(refreshMutex);

    // Process each link that needs updating
    std::vector<unsigned int> stillActive;
    for (size_t i = 0; i < refreshLinks.size(); i++)
    {
        auto link_ = refreshLinks[i];
        if (link_ >= noOfLinks)
            continue;
        if (link_ < lowestChangedLink)
        {
            lowestChangedLink = link_;
        }
        double currAngle = tableMDH[link_].thetaCurr;
        double targetAngle = targetAngles[link_];
        double distance = targetAngle - currAngle;
        double Kp = 0.2;
        double angle = distance * Kp;
        double maxIncrement = M_PI * 0.005;
        double minIncrement = M_PI * 0.0005;

        // Clamp angle increment (handle both positive and negative)
        if (angle > maxIncrement)
        {
            angle = maxIncrement;
        }
        else if (angle < -maxIncrement)
        {
            angle = -maxIncrement;
        }
        else if (std::abs(angle) < minIncrement && std::abs(distance) > minIncrement)
        {
            // Apply minimum increment in correct direction
            angle = (distance > 0) ? minIncrement : -minIncrement;
        }
        else if (std::abs(distance) <= minIncrement)
        {
            // Close enough, snap to target and mark as done
            angle = distance;
        }

        addLinkAngle(link_, angle);

        // Keep in refresh list if not yet at target
        if (std::abs(targetAngle - (currAngle + angle)) > minIncrement)
        {
            stillActive.push_back(link_);
        }
        // std::cout << "Jacobian [" << link_ << "]" << computeJacobian(link_) << std::endl;
    }
    refreshLinks = std::move(stillActive);

    if (lowestChangedLink >= noOfLinks)
    {
        // Nothing to update - this is normal when no links are changing
        return true;
    }
    // Initialize cumulative transform
    // MDH convention: T_i = RotZ(theta) * TransZ(d) * TransX(a) * RotX(alpha)
    Eigen::Isometry3d cumTransform = Eigen::Isometry3d::Identity();
    if (lowestChangedLink > 0)
    {
        cumTransform = tableLinkTransforms[lowestChangedLink - 1];
    }
    else
    {
        Eigen::Isometry3d origTransform = Eigen::Isometry3d::Identity();
        origTransform.translation() = origWorldPos;
        origTransform.linear() = origWorldRot.toRotationMatrix();
        cumTransform = origTransform;
    }
    for (size_t i = lowestChangedLink; i < noOfLinks; ++i)
    {
        const MDH &mdh = tableMDH[i];
        Eigen::Isometry3d linkTransform = Eigen::Isometry3d::Identity();

        double theta = mdh.thetaCurr + mdh.tCOffset;
        double d = mdh.dCurr + mdh.dCOffset;

        // MDH transform: negate theta for right-hand rule compliance
        linkTransform.translate(Eigen::Vector3d(mdh.aPrev, 0.0, 0.0));
        linkTransform.rotate(Eigen::AngleAxisd(mdh.alphaPrev, Eigen::Vector3d::UnitX()));
        linkTransform.rotate(Eigen::AngleAxisd(-theta, Eigen::Vector3d::UnitZ()));
        linkTransform.translate(Eigen::Vector3d(0.0, 0.0, d));

        cumTransform = cumTransform * linkTransform;

        // Store the world position of this link's frame origin
        tableLinkTransforms[i] = cumTransform;
    }

    auto snap = std::make_shared<const std::vector<Eigen::Isometry3d>>(tableLinkTransforms);
    {
        std::lock_guard<std::mutex> lock(publishMutex);
        publishedTransforms = snap;
    }
    lowestChangedLink = noOfLinks;
    return true;
}

const std::vector<Eigen::Isometry3d> Arm::fetchLinkTransformations() const
{
    std::lock_guard<std::mutex> lock(publishMutex);
    if (publishedTransforms)
    {
        return *publishedTransforms;
    }
    return {};
}

bool Arm::setLinkLength(unsigned int link, double length)
{
    if (link >= tableMDH.size())
    {
        return false;
    }
    tableMDH[link].dCurr = length;
    if (lowestChangedLink > link)
    {
        lowestChangedLink = link;
    }
    return true;
}

bool Arm::addLinkLength(unsigned int link, double length)
{
    if (link >= tableMDH.size())
    {
        return false;
    }
    tableMDH[link].dCurr += length;
    if (lowestChangedLink > link)
    {
        lowestChangedLink = link;
    }
    return true;
}

bool Arm::setLinkAngle(unsigned int link, double angle)
{
    // Check the validity of link
    if (tableMDH.size() <= link)
    {
        return false;
    }
    // Set the Z-axis rotation
    tableMDH[link].thetaCurr = angle;

    if (lowestChangedLink > link)
    {
        lowestChangedLink = link;
    }

    return true;
}

bool Arm::addLinkAngle(unsigned int link, double angle)
{
    if (tableMDH.size() < link)
    {
        return false;
    }
    // Add to the Z-axis rotation
    tableMDH[link].thetaCurr += angle;

    if (lowestChangedLink > link)
    {
        lowestChangedLink = link;
    }

    return true;
}
/// ...existing code...
Eigen::MatrixXd Arm::computeJacobian(unsigned int link) const
{
    if (link >= noOfLinks || tableMDH.empty())
    {
        return Eigen::MatrixXd::Zero(6, link + 1);
    }

    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, link + 1);

    // Build transforms matching updateArm() exactly
    std::vector<Eigen::Isometry3d> transforms(link + 1);
    std::vector<Eigen::Isometry3d> jointAxisFrames(link + 1); // Frame where z is rotation axis

    Eigen::Isometry3d cumTransform = Eigen::Isometry3d::Identity();
    cumTransform.translate(origWorldPos);
    cumTransform.rotate(origWorldRot);

    for (size_t i = 0; i <= link; ++i)
    {
        const MDH &mdh = tableMDH[i];

        // Store frame after a and alpha (joint rotation axis is z of this frame)
        Eigen::Isometry3d preTheta = cumTransform;
        preTheta.translate(Eigen::Vector3d(mdh.aPrev, 0.0, 0.0));
        preTheta.rotate(Eigen::AngleAxisd(mdh.alphaPrev, Eigen::Vector3d::UnitX()));
        jointAxisFrames[i] = preTheta;

        double theta = mdh.thetaCurr + mdh.tCOffset;
        double d = mdh.dCurr + mdh.dCOffset;

        // Complete MDH transform
        Eigen::Isometry3d linkTransform = Eigen::Isometry3d::Identity();
        linkTransform.translate(Eigen::Vector3d(mdh.aPrev, 0.0, 0.0));
        linkTransform.rotate(Eigen::AngleAxisd(mdh.alphaPrev, Eigen::Vector3d::UnitX()));
        linkTransform.rotate(Eigen::AngleAxisd(-theta, Eigen::Vector3d::UnitZ()));
        linkTransform.translate(Eigen::Vector3d(0.0, 0.0, d));

        cumTransform = cumTransform * linkTransform;
        transforms[i] = cumTransform;
    }

    Eigen::Vector3d p_e = transforms[link].translation();

    for (size_t i = 0; i <= link; ++i)
    {
        // Z-axis at joint i (negated due to -theta in forward kinematics)
        Eigen::Vector3d z_i = -jointAxisFrames[i].rotation().col(2);
        Eigen::Vector3d p_i = jointAxisFrames[i].translation();

        if (tableMDH[i].isRevolute)
        {
            J.block<3, 1>(0, i) = z_i.cross(p_e - p_i);
            J.block<3, 1>(3, i) = z_i;
        }
        else
        {
            J.block<3, 1>(0, i) = z_i;
            J.block<3, 1>(3, i).setZero();
        }
    }

    return J;
}
Eigen::VectorXd Arm::computeDLSStep(const Eigen::Vector3d &targetPos,
                                    const Eigen::Quaterniond &targetOri,
                                    double lambda) const
{
    if (noOfLinks == 0)
        return Eigen::VectorXd::Zero(1);

    unsigned int eeLink = noOfLinks - 1;
    Eigen::MatrixXd J = computeJacobian(eeLink);

    // Get current end-effector pose
    auto transforms = fetchLinkTransformations();
    if (transforms.empty())
        return Eigen::VectorXd::Zero(noOfLinks);

    Eigen::Isometry3d T_ee = transforms[eeLink];
    Eigen::Vector3d p_current = T_ee.translation();
    Eigen::Quaterniond ori_current(T_ee.rotation());

    // Position error
    Eigen::Vector3d pos_error = targetPos - p_current;
    
    // Orientation error (quaternion difference → axis-angle)
    Eigen::Quaterniond ori_error = targetOri * ori_current.inverse();
    
    if (ori_error.w() < 0)  // Shortest path
        ori_error.coeffs() *= -1; 
    Eigen::AngleAxisd aa(ori_error);
    Eigen::Vector3d rot_error = aa.angle() * aa.axis();

    // Combined error vector
    Eigen::VectorXd dx(6);
    dx << pos_error, rot_error;

    // DLS: Δq = J^T * (J*J^T + λ²I)^(-1) * Δx
    Eigen::MatrixXd JJT = J * J.transpose();
    Eigen::MatrixXd damped = JJT + lambda * lambda * Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXd dq = J.transpose() * damped.ldlt().solve(dx);

    return dq;
}