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
        if (mdh)
        {
            double a = 0, alpha = 0, dOffset = 0, theta_offset = 0;
            mdh->QueryDoubleAttribute("a", &a);
            mdh->QueryDoubleAttribute("alpha", &alpha);
            mdh->QueryDoubleAttribute("d", &dOffset);
            mdh->QueryDoubleAttribute("theta_offset", &theta_offset);

            bool isRevolute = true;
            const char *type = joint->Attribute("type");
            if (type && std::string(type) == "prismatic")
            {
                isRevolute = false;
            }

            //               MDH(aPrev, alphaPrev, thetaCurr, dCurr, tCOffset, dCOffset, isRevolute)
            tableMDH.emplace_back(a, alpha, 0.0, 0.0, theta_offset, dOffset, isRevolute);
        }
        joint = joint->NextSiblingElement("joint");
    }

    noOfLinks = tableMDH.size();

    tableLinkTransforms.resize(noOfLinks, Eigen::Isometry3d::Identity());
    targetAngles.resize(noOfLinks, 0.0); // Initialize target angles to 0
}

bool Arm::setTargetAngle(unsigned int link, double angle)
{
    if (link >= targetAngles.size())
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(refreshMutex);
    targetAngles[link] = angle;

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
    
    for (const auto& mdh : tableMDH)
    {
        MDHRow row;
        row.alpha = mdh.alphaPrev;
        row.a = mdh.aPrev;
        row.d = mdh.dCurr;
        row.theta = mdh.thetaCurr;
        row.thetaOffset = mdh.tCOffset;
        row.dOffset = mdh.dCOffset;
        row.isRevolute = mdh.isRevolute;
        rows.push_back(row);
    }
    return rows;
}

bool Arm::setMDHParam(unsigned int link, const std::string& param, double value)
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
    }
    refreshLinks = std::move(stillActive);

   if (lowestChangedLink >= noOfLinks)
    {
        // Nothing to update
        std::cout << "lowest changed link is more than noOfLinks" << lowestChangedLink << std::endl;
        return true;
    }
    // Initialize cumulative transform
    // MDH convention: T_i = RotZ(theta) * TransZ(d) * TransX(a) * RotX(alpha)
    Eigen::Isometry3d cumTransform = Eigen::Isometry3d::Identity();
    std::cout << "Lowest Changed Link: " << lowestChangedLink << std::endl;
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

        // MDH transform:
        linkTransform.translate(Eigen::Vector3d(mdh.aPrev, 0.0, 0.0));
        linkTransform.rotate(Eigen::AngleAxisd(mdh.alphaPrev, Eigen::Vector3d::UnitX()));
        linkTransform.rotate(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
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
