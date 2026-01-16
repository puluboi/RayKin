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
    
    // Frame management callbacks
    void setCreateFrameCallback(std::function<unsigned int(const std::string&)> callback);
    void setDestroyFrameCallback(std::function<bool(unsigned int)> callback);
    void setFrameParentCallback(std::function<bool(unsigned int, int, int)> callback);
    void setFramePositionCallback(std::function<bool(unsigned int, double, double, double)> callback);
    void setFrameRotationCallback(std::function<bool(unsigned int, double, double, double)> callback);
    void setFrameNameCallback(std::function<bool(unsigned int, const std::string&)> callback);
    void setGetFrameListCallback(std::function<std::vector<std::pair<unsigned int, std::string>>()> callback);
    void setArmTargetFrameCallback(std::function<bool(unsigned int, int)> callback);
    void setGetArmTargetFrameCallback(std::function<int(unsigned int)> callback);
    void setGetArmNumJointsCallback(std::function<unsigned int(unsigned int)> callback);

private:
    void renderArms(const SimSnapshot &snapshot);
    void renderArmLink(const Eigen::Isometry3d &transform, const Eigen::Isometry3d &prevTransform, Color color, bool highlighted = false);
    void drawAxis(const Eigen::Isometry3d &transform, float axisLength = 0.3f);
    void renderFrames(const SimSnapshot &snapshot);

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
    
    // IK target dropdown in robot menu
    bool ikTargetDropdownOpen = false;
    int ikDropdownScrollOffset = 0;
    
    // MDH callbacks
    std::function<std::vector<MDHRow>(unsigned int)> getMDHTable;
    std::function<bool(unsigned int, unsigned int, const std::string&, double)> setMDHParam;
    
    // MDH table UI methods
    void handleArmSelection();
    void updateMDHTable();
    void renderMDHTableUI();      // Renders 3D elements (line, sphere)
    void renderMDHTable2D();      // Renders 2D table overlay
    void renderIKTargetDropdown(int x, int y, int width);  // IK target selection dropdown
    
    // Frame management callbacks
    std::function<unsigned int(const std::string&)> createFrame;
    std::function<bool(unsigned int)> destroyFrame;
    std::function<bool(unsigned int, int, int)> setFrameParent;
    std::function<bool(unsigned int, double, double, double)> setFramePosition;
    std::function<bool(unsigned int, double, double, double)> setFrameRotation;
    std::function<bool(unsigned int, const std::string&)> setFrameName;
    std::function<std::vector<std::pair<unsigned int, std::string>>()> getFrameList;
    std::function<bool(unsigned int, int)> setArmTargetFrame;
    std::function<int(unsigned int)> getArmTargetFrame;
    std::function<unsigned int(unsigned int)> getArmNumJoints;  // Get joint count for arm
    
    // Frame UI state
    bool frameMenuMode = false;
    int selectedFrameId = -1;  // -1 = none
    std::string frameNameBuffer;
    float framePositionInput[3] = {0, 0, 0};    // X, Y, Z for local position
    float frameRotationInput[3] = {0, 0, 0};    // Roll, Pitch, Yaw in degrees
    int frameParentArmInput = -1;
    int frameParentJointInput = -1;
    
    // Frame dropdown menu state
    enum class FrameDropdownState { None, SelectArm, SelectJoint };
    FrameDropdownState frameDropdownState = FrameDropdownState::None;
    int dropdownScrollOffset = 0;
    
    // Frame transform text input state
    enum class FrameInputField { None, PosX, PosY, PosZ, RotX, RotY, RotZ };
    FrameInputField frameInputField = FrameInputField::None;
    std::string frameTransformInputBuffer;
    
    // Frame dragging state
    bool isDraggingFrame = false;
    Vector3 dragPlaneNormal;      // Normal of drag plane (camera-facing)
    Vector3 dragOffset;           // Offset from frame center to drag start point
    
    // Frame UI methods
    void handleFrameUI();
    void handleFrameDragging();
    void renderFrameUI();
    void renderFrameDropdown(int x, int y, int width);
    bool renderFrameTransformInput(int x, int& y, const char* label, FrameInputField field, float& value);
};