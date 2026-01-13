#pragma once
#include "raylib.h"
#include "sim.hpp"
#include <memory>
#include <functional>

class App;

class Viz
{
public:
    Viz(const float fov, const int screenW, const int screenH);
    void update();
    void updateCamera();
    void render();
    void rotateCamera();

    // Set callback to fetch simulation data
    void setSimDataCallback(std::function<std::shared_ptr<SimSnapshot>()> callback);

    // Set callback to modify joint angles
    void setSimControlCallback(std::function<void(unsigned int armIdx, unsigned int jointIdx, double angle)> callback);
    void setIncrementJointCallback(std::function<void(unsigned int armIdx, unsigned int jointIdx, double delta)> callback);
    
    // Set callback to get current joint angle
    void setGetAngleCallback(std::function<double(unsigned int armIdx, unsigned int jointIdx)> callback);
    
    // Set callback to get MDH table for an arm
    void setGetMDHTableCallback(std::function<std::vector<MDHRow>(unsigned int armIdx)> callback);
    
    // Set callback to modify MDH parameter
    void setMDHControlCallback(std::function<bool(unsigned int armIdx, unsigned int link, const std::string& param, double value)> callback);

private:
    void renderArms(const SimSnapshot &snapshot);
    void renderArmLink(const Eigen::Isometry3d &transform, const Eigen::Isometry3d &prevTransform, Color color, bool highlighted = false);
    void drawAxis(const Eigen::Isometry3d &transform, float axisLength = 0.3f);

       // Joint selection methods
    void handleJointSelection();
    void updateSelectedJoint();
    void renderSelectionUI();
    
    // Camera Settings
    const int screenWidth;
    const int screenHeight;
    const float fov;
    Camera3D camera = {0};
    Vector2 lookRotation;
    float sensitivity;
    float speed;

    Vector3 cubePosition;
    Vector3 cubeSize;

    Ray ray = {0}; // Picking line ray
    RayCollision collision = {0};

    // Simulation data callback
    std::function<std::shared_ptr<SimSnapshot>()> fetchSimData;
    std::function<void(unsigned int, unsigned int, double)> controlJoint;
    std::function<void(unsigned int, unsigned int, double)> incrementJoint;
    std::function<double(unsigned int, unsigned int)> getCurrentAngle;
    std::shared_ptr<SimSnapshot> cachedSnapshot;

    // Joint selection state
    bool jointSelected;
    unsigned int selectedArmIndex;
    unsigned int selectedJointIndex;
    double currentTargetAngle;
    const float jointSphereRadius = 0.08f;
    const float angleIncrement = 0.05f; // Radians per key press
    
    // Text input mode for angle entry
    bool textInputMode;
    std::string angleInputBuffer;
    const int maxInputLength = 8;
    
    // MDH table view mode
    bool mdhTableMode;
    std::vector<MDHRow> cachedMDHTable;
    int mdhSelectedRow;      // -1 = none, 0+ = link index
    int mdhSelectedCol;      // 0=alpha, 1=a, 2=d_off, 3=theta_off
    std::string mdhInputBuffer;
    Vector3 mdhTablePosition3D;  // 3D position of table in world space
    Vector3 armCenterOfMass;     // Center of mass of selected arm
    
    // MDH callbacks
    std::function<std::vector<MDHRow>(unsigned int)> getMDHTable;
    std::function<bool(unsigned int, unsigned int, const std::string&, double)> setMDHParam;
    
    // MDH table UI methods
    void handleArmSelection();
    void updateMDHTable();
    void renderMDHTableUI();      // Renders 3D elements (line, sphere)
    void renderMDHTable2D();       // Renders 2D table overlay

};