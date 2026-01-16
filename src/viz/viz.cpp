#include "viz.hpp"
#include "raymath.h"
#include <iostream>
#include <cmath>
#include <sstream>
#include <iomanip>

Viz::Viz(const float fov, const int screenW, const int screenH) : screenWidth(screenW), screenHeight(screenH), fov(fov)
{
    InitWindow(screenWidth, screenHeight, "-- Raykin --");

    // Define the camera to look into our 3d world
    camera.position = (Vector3){10.0f, 10.0f, 10.0f}; // Camera position
    camera.target = (Vector3){0.0f, 0.0f, 0.0f};      // Camera looking at point
    camera.up = (Vector3){0.0f, 1.0f, 0.0f};          // Camera up vector (rotation towards target)
    camera.fovy = fov;                                // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;           // Camera projection type
    lookRotation = Vector2{0.0f, 0.0f};
    cubePosition = {0.0f, 0.0f, 0.0f};
    cubeSize = {0.10f, 0.10f, 0.10f};
    sensitivity = 0.05f;
    speed = 0.1f;

    jointSelected = false;
    textInputMode = false;
    currentTargetAngle = 0.0;
    selectedArmIndex = 0;
    selectedJointIndex = 0;

    // MDH table mode init
    mdhTableMode = false;
    mdhSelectedRow = -1;
    mdhSelectedCol = -1;

    SetTargetFPS(60);
}

void Viz::setSimDataCallback(std::function<std::shared_ptr<SimSnapshot>()> callback)
{
    fetchSimData = callback;
}
void Viz::setSimControlCallback(std::function<void(unsigned int, unsigned int, double)> callback)
{
    controlJoint = callback;
}

void Viz::setIncrementJointCallback(std::function<void(unsigned int, unsigned int, double)> callback)
{
    incrementJoint = callback;
}

void Viz::setGetAngleCallback(std::function<double(unsigned int, unsigned int)> callback)
{
    getCurrentAngle = callback;
}

void Viz::setGetMDHTableCallback(std::function<std::vector<MDHRow>(unsigned int)> callback)
{
    getMDHTable = callback;
}

void Viz::setMDHControlCallback(std::function<bool(unsigned int, unsigned int, const std::string &, double)> callback)
{
    setMDHParam = callback;
}

void Viz::setCreateFrameCallback(std::function<unsigned int(const std::string&)> callback)
{
    createFrame = callback;
}

void Viz::setDestroyFrameCallback(std::function<bool(unsigned int)> callback)
{
    destroyFrame = callback;
}

void Viz::setFrameParentCallback(std::function<bool(unsigned int, int, int)> callback)
{
    setFrameParent = callback;
}

void Viz::setFramePositionCallback(std::function<bool(unsigned int, double, double, double)> callback)
{
    setFramePosition = callback;
}

void Viz::setFrameRotationCallback(std::function<bool(unsigned int, double, double, double)> callback)
{
    setFrameRotation = callback;
}

void Viz::setFrameNameCallback(std::function<bool(unsigned int, const std::string&)> callback)
{
    setFrameName = callback;
}

void Viz::setGetFrameListCallback(std::function<std::vector<std::pair<unsigned int, std::string>>()> callback)
{
    getFrameList = callback;
}

void Viz::setArmTargetFrameCallback(std::function<bool(unsigned int, int)> callback)
{
    setArmTargetFrame = callback;
}

void Viz::setGetArmTargetFrameCallback(std::function<int(unsigned int)> callback)
{
    getArmTargetFrame = callback;
}

void Viz::setGetArmNumJointsCallback(std::function<unsigned int(unsigned int)> callback)
{
    getArmNumJoints = callback;
}

void Viz::update()
{
    // Fetch latest simulation data
    if (fetchSimData)
    {
        cachedSnapshot = fetchSimData();
    }

    // Refresh MDH table if in MDH mode
    if (mdhTableMode && getMDHTable)
    {
        cachedMDHTable = getMDHTable(selectedArmIndex);

        // Update table position relative to arm's current position
        if (cachedSnapshot && selectedArmIndex < cachedSnapshot->armTransforms.size())
        {
            const auto &transforms = cachedSnapshot->armTransforms[selectedArmIndex];
            if (!transforms.empty())
            {
                // Recalculate center of mass
                Eigen::Vector3d com(0, 0, 0);
                for (const auto &t : transforms)
                {
                    com += t.translation();
                }
                com /= transforms.size();

                // Store COM in raylib coordinates
                armCenterOfMass = {(float)com.x(), (float)com.z(), (float)com.y()};

                // Position table offset from COM in world space
                mdhTablePosition3D = {
                    armCenterOfMass.x + 3.f,
                    armCenterOfMass.y + 1.5f,
                    armCenterOfMass.z};
            }
        }
    }

    // Handle joint selection with mouse
    handleJointSelection();

    // Handle arm selection for MDH view (M key toggles)
    handleArmSelection();

    // Update selected joint angle with keyboard
    updateSelectedJoint();

    // Handle MDH table editing
    updateMDHTable();
    
    // Handle frame UI
    handleFrameUI();

    // Update camera based on user input
    updateCamera();

    // Render Frame
    render();
}
void Viz::handleJointSelection()
{
    // Only handle selection when left mouse is clicked and cursor is visible (not in camera control mode)
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !IsCursorHidden() && cachedSnapshot)
    {
        ray = GetMouseRay(GetMousePosition(), camera);

        // Check all joints to find the closest hit
        float closestDistance = 1000000.0f;
        bool foundJoint = false;
        unsigned int hitArmIdx = 0;
        unsigned int hitJointIdx = 0;

        for (size_t armIdx = 0; armIdx < cachedSnapshot->armTransforms.size(); ++armIdx)
        {
            const auto &transforms = cachedSnapshot->armTransforms[armIdx];

            for (size_t jointIdx = 0; jointIdx < transforms.size(); ++jointIdx)
            {
                // Get joint position in world space
                Eigen::Vector3d pos = transforms[jointIdx].translation();
                // Convert to raylib coordinates (swap Y and Z)
                Vector3 sphereCenter = {(float)pos.x(), (float)pos.z(), (float)pos.y()};

                // Check ray-sphere collision
                RayCollision sphereHit = GetRayCollisionSphere(ray, sphereCenter, jointSphereRadius);

                if (sphereHit.hit && sphereHit.distance < closestDistance)
                {
                    closestDistance = sphereHit.distance;
                    foundJoint = true;
                    hitArmIdx = armIdx;
                    hitJointIdx = jointIdx;
                }
            }
        }

        if (foundJoint)
        {
            jointSelected = true;
            selectedArmIndex = hitArmIdx;
            selectedJointIndex = hitJointIdx;

            // Fetch actual current angle from simulation
            if (getCurrentAngle)
            {
                currentTargetAngle = getCurrentAngle(hitArmIdx, hitJointIdx);
            }
            else
            {
                currentTargetAngle = 0.0;
            }

            std::cout << "Selected Joint - Arm: " << selectedArmIndex
                      << ", Joint: " << selectedJointIndex
                      << ", Current angle: " << currentTargetAngle << " rad ("
                      << (currentTargetAngle * 180.0 / M_PI) << " deg)" << std::endl;
        }
        else
        {
            // Deselect if clicking on empty space
            jointSelected = false;
        }
    }
}

void Viz::updateSelectedJoint()
{
    if (!jointSelected || !controlJoint)
        return;

    // Check if user wants to enter text input mode
    if (IsKeyPressed(KEY_T) && !textInputMode)
    {
        textInputMode = true;
        angleInputBuffer.clear();
        std::cout << "Text input mode activated. Type angle in degrees." << std::endl;
        return;
    }

    // Handle text input mode
    if (textInputMode)
    {
        // Get character input
        int key = GetCharPressed();
        while (key > 0)
        {
            // Allow digits, decimal point, and minus sign
            if ((key >= '0' && key <= '9') || key == '.' ||
                (key == '-' && angleInputBuffer.empty()))
            {
                if (angleInputBuffer.length() < maxInputLength)
                {
                    angleInputBuffer += (char)key;
                }
            }
            key = GetCharPressed();
        }

        // Handle backspace
        if (IsKeyPressed(KEY_BACKSPACE) && !angleInputBuffer.empty())
        {
            angleInputBuffer.pop_back();
        }

        // Handle Enter - apply the angle
        if (IsKeyPressed(KEY_ENTER) && !angleInputBuffer.empty())
        {
            try
            {
                double degrees = std::stod(angleInputBuffer);
                currentTargetAngle = degrees * M_PI / 180.0; // Convert to radians
                controlJoint(selectedArmIndex, selectedJointIndex, currentTargetAngle);
                std::cout << "Joint angle set to: " << currentTargetAngle << " rad ("
                          << degrees << " deg)" << std::endl;
                textInputMode = false;
                angleInputBuffer.clear();
            }
            catch (...)
            {
                std::cout << "Invalid angle value" << std::endl;
                angleInputBuffer.clear();
            }
        }

        // Cancel text input
        if (IsKeyPressed(KEY_ESCAPE))
        {
            textInputMode = false;
            angleInputBuffer.clear();
            std::cout << "Text input cancelled" << std::endl;
        }

        return; // Don't process other controls while in text input mode
    }

    // Use arrow keys or +/- to adjust angle when a joint is selected
    bool angleChanged = false;

    if (IsKeyPressed(KEY_KP_ADD) || IsKeyPressed(KEY_EQUAL)) // + key
    {
        currentTargetAngle += angleIncrement;
        angleChanged = true;
    }
    else if (IsKeyPressed(KEY_KP_SUBTRACT) || IsKeyPressed(KEY_MINUS)) // - key
    {
        currentTargetAngle -= angleIncrement;
        angleChanged = true;
    }
    else if (IsKeyDown(KEY_PAGE_UP) && incrementJoint && !IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_RIGHT_CONTROL)) // Use Page Up for angle, but not with Ctrl
    {
        incrementJoint(selectedArmIndex, selectedJointIndex, angleIncrement * 0.5f);
        // Refresh current angle from simulation
        if (getCurrentAngle)
            currentTargetAngle = getCurrentAngle(selectedArmIndex, selectedJointIndex);
    }
    else if (IsKeyDown(KEY_PAGE_DOWN) && incrementJoint && !IsKeyDown(KEY_LEFT_CONTROL) && !IsKeyDown(KEY_RIGHT_CONTROL)) // Use Page Down for angle, but not with Ctrl
    {
        incrementJoint(selectedArmIndex, selectedJointIndex, -angleIncrement * 0.5f);
        // Refresh current angle from simulation
        if (getCurrentAngle)
            currentTargetAngle = getCurrentAngle(selectedArmIndex, selectedJointIndex);
    }

    // Reset angle to zero
    if (IsKeyPressed(KEY_R))
    {
        currentTargetAngle = 0.0;
        angleChanged = true;
    }

    // Navigate between joints with Ctrl+PageUp/PageDown
    if (IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL))
    {
        if (IsKeyPressed(KEY_PAGE_UP) && cachedSnapshot)
        {
            // Move to next joint (wraps within same arm)
            const auto &transforms = cachedSnapshot->armTransforms[selectedArmIndex];
            if (!transforms.empty())
            {
                selectedJointIndex = (selectedJointIndex + 1) % transforms.size();
                if (getCurrentAngle)
                    currentTargetAngle = getCurrentAngle(selectedArmIndex, selectedJointIndex);
                std::cout << "Switched to Joint " << selectedJointIndex << std::endl;
            }
        }
        else if (IsKeyPressed(KEY_PAGE_DOWN) && cachedSnapshot)
        {
            // Move to previous joint (wraps within same arm)
            const auto &transforms = cachedSnapshot->armTransforms[selectedArmIndex];
            if (!transforms.empty())
            {
                selectedJointIndex = (selectedJointIndex == 0) 
                    ? transforms.size() - 1 
                    : selectedJointIndex - 1;
                if (getCurrentAngle)
                    currentTargetAngle = getCurrentAngle(selectedArmIndex, selectedJointIndex);
                std::cout << "Switched to Joint " << selectedJointIndex << std::endl;
            }
        }
    }

    // Deselect joint
    if (IsKeyPressed(KEY_ESCAPE))
    {
        jointSelected = false;
        std::cout << "Joint deselected" << std::endl;
    }

    if (angleChanged)
    {
        // Send the new target angle to the simulation
        controlJoint(selectedArmIndex, selectedJointIndex, currentTargetAngle);
        std::cout << "Joint angle set to: " << currentTargetAngle << " rad ("
                  << (currentTargetAngle * 180.0 / M_PI) << " deg)" << std::endl;
    }
}
void Viz::renderSelectionUI()
{
    if (jointSelected)
    {
        // Draw selection indicator
        int y = 40;
        DrawText("JOINT SELECTED", 10, y, 20, GREEN);
        y += 25;

        std::stringstream ss;
        ss << "Arm: " << selectedArmIndex << " | Joint: " << selectedJointIndex+1;
        DrawText(ss.str().c_str(), 10, y, 16, DARKGREEN);
        y += 20;

        ss.str("");
        ss << "Target Angle: " << std::fixed << std::setprecision(2)
           << currentTargetAngle << " rad (" << (currentTargetAngle * 180.0 / M_PI) << " deg)";
        DrawText(ss.str().c_str(), 10, y, 16, DARKGREEN);
        y += 30;

        // Show text input mode
        if (textInputMode)
        {
            DrawText("TEXT INPUT MODE - Enter angle in degrees:", 10, y, 18, ORANGE);
            y += 25;

            // Draw input buffer with cursor
            std::string displayText = angleInputBuffer + "_";
            DrawText(displayText.c_str(), 10, y, 24, ORANGE);
            y += 30;

            DrawText("ENTER: Apply | BACKSPACE: Delete | ESC: Cancel", 10, y, 12, GRAY);
        }
        else
        {
            // Controls help
            DrawText("Controls:", 10, y, 14, DARKGRAY);
            y += 18;
            DrawText("  T : Type angle in degrees", 10, y, 12, GRAY);
            y += 15;
            DrawText("  +/- : Adjust angle (large step)", 10, y, 12, GRAY);
            y += 15;
            DrawText("  PgUp/PgDn : Adjust angle (small step)", 10, y, 12, GRAY);
            y += 15;
            DrawText("  Ctrl+PgUp/PgDn : Cycle joints", 10, y, 12, GRAY);
            y += 15;
            DrawText("  R : Reset to 0", 10, y, 12, GRAY);
            y += 15;
            DrawText("  ESC : Deselect joint", 10, y, 12, GRAY);
        }
    }
    else
    {
        DrawText("Click on a joint sphere to select it", 10, 40, 16, DARKGRAY);
    }
}
void Viz::updateCamera()
{
    // Skip camera movement if in text input mode
    if (textInputMode)
        return;

    // Update
    //----------------------------------------------------------------------------------
    if (IsCursorHidden())
    {
        rotateCamera();
    }
    else
    {
        lookRotation = Vector2{0.0f, 0.0f};
    }
    UpdateCameraPro(&camera,
                    (Vector3){
                        (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP)) * speed - // Move forward-backward
                            (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN)) * speed,
                        (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) * speed - // Move right-left
                            (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT)) * speed,
                        IsKeyDown(KEY_Q) * speed -    // Move up
                            IsKeyDown(KEY_E) * speed, // Move down
                    },
                    (Vector3){
                        lookRotation.x, // Rotation: yaw
                        lookRotation.y, // Rotation: pitch
                        0.0f            // Rotation: roll
                    },
                    GetMouseWheelMove() * 2.0f); // Move to target (zoom)

    // Toggle camera controls
    if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT))
    {
        if (IsCursorHidden())
            EnableCursor();
        else
            DisableCursor();
    }
}
void Viz::render()
{
    BeginDrawing();
    ClearBackground(RAYWHITE);
    BeginMode3D(camera);

    // Render arms from simulation data
    if (cachedSnapshot)
    {
        renderArms(*cachedSnapshot);
        renderFrames(*cachedSnapshot);
    }

    DrawGrid(10, 1.0f);

    // Render 3D MDH elements before ending 3D mode
    if (mdhTableMode)
    {
        renderMDHTableUI();
    }

    EndMode3D();

    // Render UI overlays
    renderSelectionUI();
    renderMDHTable2D();
    renderFrameUI();

    DrawText("Right click: camera | Click joint: select | M: MDH table | F: Frames", 10, screenHeight - 30, 10, GRAY);
    DrawFPS(10, 10);
    EndDrawing();
}
void Viz::rotateCamera()
{
    Vector2 mouseDelta = GetMouseDelta();
    lookRotation.x = mouseDelta.x * sensitivity;
    lookRotation.y = mouseDelta.y * sensitivity;
}

void Viz::renderArms(const SimSnapshot &snapshot)
{
    // Color palette for different arms
    Color armColors[] = {BLUE, GREEN, ORANGE, PURPLE, PINK, YELLOW};
    int numColors = sizeof(armColors) / sizeof(armColors[0]);

    for (size_t armIdx = 0; armIdx < snapshot.armTransforms.size(); ++armIdx)
    {
        const auto &transforms = snapshot.armTransforms[armIdx];
        Color armColor = armColors[armIdx % numColors];

        // Highlight selected arm when MDH table is shown
        bool isSelectedArm = (mdhTableMode && armIdx == selectedArmIndex);
        if (isSelectedArm)
        {
            // Brighten the color for selected arm
            armColor = ColorBrightness(armColor, 0.3f);
        }

        // Draw base sphere at origin
        if (!transforms.empty())
        {
            // Use identity as "previous" for the first link
            Eigen::Isometry3d baseTransform = Eigen::Isometry3d::Identity();
            renderArmLink(transforms[0], baseTransform, armColor, isSelectedArm);
        }

        // Draw each link
        for (size_t i = 1; i < transforms.size(); ++i)
        {
            renderArmLink(transforms[i], transforms[i - 1], armColor, isSelectedArm);
        }

        // Draw joint spheres at each transform
        for (size_t i = 0; i < transforms.size(); ++i)
        {
            Eigen::Vector3d pos = transforms[i].translation();
            Vector3 spherePos = {(float)pos.x(), (float)pos.z(), (float)pos.y()};

            // Highlight selected joint
            bool isSelectedJoint = (jointSelected && armIdx == selectedArmIndex && i == selectedJointIndex);
            if (isSelectedJoint)
            {

                // Draw bright highlight for selected joint
                DrawSphere(spherePos, 0.08f, armColor);
                DrawSphere(spherePos, 0.10f, Fade(YELLOW, 0.6));
            }
            else
            {
                DrawSphere(spherePos, 0.08f, armColor);
            }

            // Draw coordinate axes at each joint
            drawAxis(transforms[i], 0.4f);
        }

        // Draw end effector marker
        if (!transforms.empty())
        {
            Eigen::Vector3d endPos = transforms.back().translation();
            DrawSphere(
                (Vector3){(float)endPos.x(), (float)endPos.z(), (float)endPos.y()},
                0.12f,
                RED);
        }
    }
}

void Viz::renderArmLink(const Eigen::Isometry3d &transform, const Eigen::Isometry3d &prevTransform, Color color, bool highlighted)
{
    // Get positions (swap Y and Z for raylib coordinate system)
    Eigen::Vector3d startPos = prevTransform.translation();
    Eigen::Vector3d endPos = transform.translation();

    Vector3 start = {(float)startPos.x(), (float)startPos.z(), (float)startPos.y()};
    Vector3 end = {(float)endPos.x(), (float)endPos.z(), (float)endPos.y()};

    // Draw cylinder between joints
    float length = sqrtf(
        (end.x - start.x) * (end.x - start.x) +
        (end.y - start.y) * (end.y - start.y) +
        (end.z - start.z) * (end.z - start.z));

    if (length > 0.001f)
    {
        // Draw a line for the link (simpler visualization)
        DrawLine3D(start, end, color);

        // Draw cylinder with thicker radius and outline if highlighted
        float radius = highlighted ? 0.05f : 0.03f;
        DrawCylinderEx(start, end, radius, radius, 8, color);

        // Add subtle outline for highlighted arm
        if (highlighted)
        {
            DrawCylinderEx(start, end, radius + 0.015f, radius + 0.015f, 8, Fade(WHITE, 0.4f));
        }
    }
}

void Viz::drawAxis(const Eigen::Isometry3d &transform, float axisLength)
{
    // Get origin position (swap Y and Z for raylib coordinate system)
    Eigen::Vector3d origin = transform.translation();
    Vector3 originRaylib = {(float)origin.x(), (float)origin.z(), (float)origin.y()};

    // Get axis directions from the rotation matrix
    Eigen::Matrix3d rotation = transform.rotation();

    // X-axis (Red) - column 0 of rotation matrix
    Eigen::Vector3d xAxis = rotation.col(0) * axisLength;
    Vector3 xEnd = {
        (float)(origin.x() + xAxis.x()),
        (float)(origin.z() + xAxis.z()), // Swap Y/Z
        (float)(origin.y() + xAxis.y())};
    DrawLine3D(originRaylib, xEnd, RED);
    DrawCylinderEx(originRaylib, xEnd, 0.02f, 0.01f, 6, RED);

    // Y-axis (Green) - column 1 of rotation matrix
    Eigen::Vector3d yAxis = rotation.col(1) * axisLength;
    Vector3 yEnd = {
        (float)(origin.x() - yAxis.x()), // Match right hand rule
        (float)(origin.z() - yAxis.z()), // Swap Y/Z
        (float)(origin.y() - yAxis.y())};
    DrawLine3D(originRaylib, yEnd, GREEN);
    DrawCylinderEx(originRaylib, yEnd, 0.02f, 0.01f, 6, GREEN);

    // Z-axis (Blue) - column 2 of rotation matrix
    Eigen::Vector3d zAxis = rotation.col(2) * axisLength;
    Vector3 zEnd = {
        (float)(origin.x() + zAxis.x()),
        (float)(origin.z() + zAxis.z()), // Swap Y/Z
        (float)(origin.y() + zAxis.y())};
    DrawLine3D(originRaylib, zEnd, BLUE);
    DrawCylinderEx(originRaylib, zEnd, 0.02f, 0.01f, 6, BLUE);
}

void Viz::handleArmSelection()
{
    // Check for clicking near an arm (not just on joints)
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !IsCursorHidden() && cachedSnapshot && !textInputMode && !jointSelected)
    {
        ray = GetMouseRay(GetMousePosition(), camera);

        // Check proximity to any arm
        float closestDistance = 1000000.0f;
        bool foundArm = false;
        unsigned int hitArmIdx = 0;

        for (size_t armIdx = 0; armIdx < cachedSnapshot->armTransforms.size(); ++armIdx)
        {
            const auto &transforms = cachedSnapshot->armTransforms[armIdx];

            // Check distance to each link segment and joints
            for (size_t i = 0; i < transforms.size(); ++i)
            {
                Eigen::Vector3d pos = transforms[i].translation();
                Vector3 sphereCenter = {(float)pos.x(), (float)pos.z(), (float)pos.y()};

                // Larger radius for proximity detection (not exact collision)
                RayCollision hit = GetRayCollisionSphere(ray, sphereCenter, 0.3f);

                if (hit.hit && hit.distance < closestDistance)
                {
                    closestDistance = hit.distance;
                    foundArm = true;
                    hitArmIdx = armIdx;
                }
            }
        }

        if (foundArm)
        {
            // Toggle MDH table for this arm
            if (mdhTableMode && selectedArmIndex == hitArmIdx)
            {
                // Clicking same arm closes table
                mdhTableMode = false;
                jointSelected = false;
                std::cout << "MDH Table mode OFF" << std::endl;
            }
            else
            {
                // Open table for this arm
                mdhTableMode = true;
                jointSelected = false;
                selectedArmIndex = hitArmIdx;
                mdhSelectedRow = -1;
                mdhSelectedCol = -1;
                mdhInputBuffer.clear();

                // Calculate center of mass and table position
                const auto &transforms = cachedSnapshot->armTransforms[hitArmIdx];
                Eigen::Vector3d com(0, 0, 0);
                for (const auto &t : transforms)
                {
                    com += t.translation();
                }
                com /= transforms.size();

                // Store COM in raylib coordinates
                armCenterOfMass = {(float)com.x(), (float)com.z(), (float)com.y()};

                // Position table offset from COM
                mdhTablePosition3D = {
                    armCenterOfMass.x + 2.0f,
                    armCenterOfMass.y + 1.0f,
                    armCenterOfMass.z};

                if (getMDHTable)
                {
                    cachedMDHTable = getMDHTable(selectedArmIndex);
                }

                std::cout << "MDH Table mode ON for Arm " << selectedArmIndex << std::endl;
            }
        }
    }

    // Close table with M key or ESC
    if (mdhTableMode && !textInputMode)
    {
        if (IsKeyPressed(KEY_M) || IsKeyPressed(KEY_ESCAPE))
        {
            mdhTableMode = false;
            mdhSelectedRow = -1;
            mdhSelectedCol = -1;
            ikTargetDropdownOpen = false;  // Close IK dropdown too
            std::cout << "MDH Table mode OFF" << std::endl;
        }
    }
}

void Viz::updateMDHTable()
{
    if (!mdhTableMode)
        return;

    // Handle text input mode
    if (textInputMode)
    {
        // Text input mode for editing MDH value
        int key = GetCharPressed();
        while (key > 0)
        {
            if ((key >= '0' && key <= '9') || key == '.' ||
                (key == '-' && mdhInputBuffer.empty()))
            {
                if (mdhInputBuffer.length() < (size_t)maxInputLength)
                {
                    mdhInputBuffer += (char)key;
                }
            }
            key = GetCharPressed();
        }

        if (IsKeyPressed(KEY_BACKSPACE) && !mdhInputBuffer.empty())
        {
            mdhInputBuffer.pop_back();
        }

        // Apply edit
        if (IsKeyPressed(KEY_ENTER) && !mdhInputBuffer.empty())
        {
            try
            {
                double value = std::stod(mdhInputBuffer);
                const char* paramNames[] = {"alpha", "a", "d_offset", "theta_offset", "minTheta", "maxTheta"};
                std::string param = paramNames[mdhSelectedCol];

                if (setMDHParam)
                {
                    bool success = setMDHParam(selectedArmIndex, mdhSelectedRow, param, value);
                    if (success)
                    {
                        std::cout << "Set " << param << "[" << mdhSelectedRow << "] = " << value << std::endl;
                    }
                }
            }
            catch (...)
            {
                std::cout << "Invalid value" << std::endl;
            }
            textInputMode = false;
            mdhInputBuffer.clear();
        }

        // Cancel edit
        if (IsKeyPressed(KEY_ESCAPE))
        {
            textInputMode = false;
            mdhInputBuffer.clear();
            std::cout << "Edit cancelled" << std::endl;
        }
    }
}

void Viz::renderMDHTableUI()
{
    if (!mdhTableMode)
        return;

    // Draw 3D connection line and marker (while still in 3D mode)
    DrawLine3D(mdhTablePosition3D, armCenterOfMass, BLACK);
    DrawSphere(armCenterOfMass, 0.1f, BLACK);
}

void Viz::renderMDHTable2D()
{
    if (!mdhTableMode)
        return;

    // Project 3D table position to 2D screen space
    Vector2 tablePos2D = GetWorldToScreen(mdhTablePosition3D, camera);

    // Table dimensions
    const int rowHeight = 22;
    // Link, alpha, a, d_off, theta_off, lower, upper, Type
    const int colWidths[] = {35, 65, 65, 65, 65, 65, 65, 35}; // Link, α, a, d_off, θ_off, Type
    const int headerHeight = 25;
    int tableWidth = 35 + 65 + 65 + 65 + 65 + 65 + 65 + 35;
    int tableHeight = headerHeight + (cachedMDHTable.size() + 1) * rowHeight + 80;  // Extra space for IK target

    int tableX = (int)tablePos2D.x - tableWidth / 2;
    int tableY = (int)tablePos2D.y;

    // Better validation and positioning
    // Check if position is valid and on-screen
    if (tablePos2D.x < -10000 || tablePos2D.y < -10000 ||
        tablePos2D.x > screenWidth + 10000 || tablePos2D.y > screenHeight + 10000)
    {
        // Position is invalid (likely behind camera), place in corner
        tableX = screenWidth - tableWidth - 20;
        tableY = 50;
    }
    else
    {
        // Clamp to screen bounds
        if (tableX < 10)
            tableX = 10;
        if (tableY < 50)
            tableY = 50;
        if (tableX + tableWidth > screenWidth - 10)
            tableX = screenWidth - tableWidth - 10;
        if (tableY + tableHeight > screenHeight - 50)
            tableY = screenHeight - tableHeight - 10;
    }

    // Background panel with transparency
    DrawRectangle(tableX - 10, tableY - 20, tableWidth + 20, tableHeight + 20, Fade(BLACK, 0.15f));
    DrawRectangleLines(tableX - 10, tableY - 20, tableWidth + 20, tableHeight + 20, BLACK);

    // Title
    std::stringstream title;
    title << "MDH Table - Arm " << selectedArmIndex;
    DrawText(title.str().c_str(), tableX, tableY, 14, BLACK);
    DrawText("(Click cell to edit, ESC/M to close)", tableX, tableY + 15, 10, GRAY);

    int y = tableY + headerHeight + 15;

    // Header row
    int x = tableX;
    DrawText("Lnk", x, y, 11, BLACK); x += colWidths[0];
    DrawText("alpha", x, y, 11, BLACK); x += colWidths[1];
    DrawText("a", x, y, 11, BLACK); x += colWidths[2];
    DrawText("d_off", x, y, 11, BLACK); x += colWidths[3];
    DrawText("th_off", x, y, 11, BLACK); x += colWidths[4];
    DrawText("minTheta", x, y, 11, BLACK); x += colWidths[5];
    DrawText("maxTheta", x, y, 11, BLACK); x += colWidths[6];
    DrawText("Type", x, y, 11, BLACK);
    
    y += rowHeight;
    DrawLine(tableX, y - 2, tableX + tableWidth, y - 2, BLACK);

    // Mouse position for cell detection
    Vector2 mousePos = GetMousePosition();
    bool mouseClicked = IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !textInputMode;

    // Data rows
    for (size_t i = 0; i < cachedMDHTable.size(); ++i)
    {
        const auto &row = cachedMDHTable[i];
        x = tableX;

        bool isSelectedRow = (mdhSelectedRow == (int)i);

        // Check mouse hover over row
        bool mouseOverRow = (mousePos.x >= tableX && mousePos.x <= tableX + tableWidth &&
                             mousePos.y >= y - 2 && mousePos.y <= y + rowHeight);

        if (mouseOverRow && !textInputMode)
        {
            DrawRectangle(tableX - 5, y - 2, tableWidth + 10, rowHeight, Fade(YELLOW, 0.2f));
        }
        else if (isSelectedRow)
        {
            DrawRectangle(tableX - 5, y - 2, tableWidth + 10, rowHeight, Fade(BLUE, 0.3f));
        }

        std::stringstream ss;

        // Link number
        ss << i + 1;
        DrawText(ss.str().c_str(), x + 5, y, 11, Fade(BLACK, 0.8f));
        x += colWidths[0];

        // Helper to draw cells and detect clicks
        auto drawCell = [&](int col, double value)
        {
            int cellStartX = x;
            int cellWidth = colWidths[col + 1];

            // Check if mouse is over this cell
            bool mouseOverCell = (mousePos.x >= cellStartX && mousePos.x < cellStartX + cellWidth &&
                                  mousePos.y >= y - 2 && mousePos.y <= y + rowHeight);

            // Handle click on cell
            if (mouseOverCell && mouseClicked)
            {
                mdhSelectedRow = i;
                mdhSelectedCol = col;
                textInputMode = true;
                mdhInputBuffer.clear();
                const char* colNames[] = {"alpha", "a", "d_offset", "theta_offset", "maxTheta", "minTheta"};
                std::cout << "Editing " << colNames[col] << " at row " << i << std::endl;
            }

            // Draw cell content
            if (textInputMode && isSelectedRow && mdhSelectedCol == col)
            {
                std::string editText = mdhInputBuffer + "_";
                DrawText(editText.c_str(), x, y, 11, YELLOW);
            }
            else
            {
                ss.str("");
                ss << std::fixed << std::setprecision(2) << value;
                Color cellColor = (isSelectedRow && mdhSelectedCol == col) ? GREEN : (mouseOverCell ? YELLOW : BLACK);
                DrawText(ss.str().c_str(), x, y, 11, Fade(cellColor, 0.8));
            }
        };
        // Draw the six editable numeric columns
        drawCell(0, row.alpha);       x += colWidths[1];
        drawCell(1, row.a);           x += colWidths[2];
        drawCell(2, row.dOffset);     x += colWidths[3];
        drawCell(3, row.thetaOffset); x += colWidths[4];
        drawCell(4, row.minTheta);  x += colWidths[5];
        drawCell(5, row.maxTheta);  x += colWidths[6];
        
        // Type (not editable)
        DrawText(row.isRevolute ? "R" : "P", x, y, 11, row.isRevolute ? SKYBLUE : MAGENTA);
        
        y += rowHeight;
    }

    // IK Target section
    y += 15;
    DrawLine(tableX, y - 5, tableX + tableWidth, y - 5, Fade(BLACK, 0.5f));
    
    DrawText("IK Target:", tableX, y, 12, BLACK);
    
    // Get current target
    int currentTargetId = -1;
    std::string targetName = "None";
    if (getArmTargetFrame)
    {
        currentTargetId = getArmTargetFrame(selectedArmIndex);
        if (currentTargetId >= 0 && cachedSnapshot)
        {
            for (const auto& frame : cachedSnapshot->frames)
            {
                if (static_cast<int>(frame.id) == currentTargetId)
                {
                    targetName = frame.name + " (ID:" + std::to_string(frame.id) + ")";
                    break;
                }
            }
        }
    }
    
    // Dropdown button
    Rectangle ikDropdownBtn = {(float)(tableX + 70), (float)(y - 2), 150, 18};
    bool hoverIKDropdown = CheckCollisionPointRec(mousePos, ikDropdownBtn);
    DrawRectangleRec(ikDropdownBtn, hoverIKDropdown ? Fade(SKYBLUE, 0.5f) : Fade(GRAY, 0.3f));
    DrawRectangleLinesEx(ikDropdownBtn, 1, BLACK);
    
    std::string btnText = "[" + targetName + "] v";
    DrawText(btnText.c_str(), tableX + 75, y, 11, currentTargetId >= 0 ? ORANGE : DARKGRAY);
    
    if (hoverIKDropdown && mouseClicked && !textInputMode)
    {
        ikTargetDropdownOpen = !ikTargetDropdownOpen;
        ikDropdownScrollOffset = 0;
    }
    
    y += 22;
    
    // Render dropdown if open
    if (ikTargetDropdownOpen)
    {
        renderIKTargetDropdown(tableX + 70, y, 180);
    }

    // Instructions
    y += 10;
    DrawText("Click cell to edit | ENTER: Apply | ESC: Cancel", tableX, y, 10, GRAY);
}

void Viz::renderIKTargetDropdown(int x, int y, int width)
{
    if (!cachedSnapshot) return;
    
    Vector2 mousePos = GetMousePosition();
    bool mouseClicked = IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
    
    const int itemHeight = 18;
    const int maxVisibleItems = 6;
    
    // Count items: "None" + all frames
    int numFrames = cachedSnapshot->frames.size();
    int numItems = numFrames + 1;  // +1 for "None" option
    int visibleItems = std::min(numItems, maxVisibleItems);
    int dropdownHeight = visibleItems * itemHeight + 4;
    
    // Background
    DrawRectangle(x, y, width, dropdownHeight, Fade(DARKBLUE, 0.95f));
    DrawRectangleLinesEx({(float)x, (float)y, (float)width, (float)dropdownHeight}, 1, WHITE);
    
    int itemY = y + 2;
    
    // "None" option to clear target
    Rectangle noneRect = {(float)x, (float)itemY, (float)width, (float)itemHeight};
    bool hoverNone = CheckCollisionPointRec(mousePos, noneRect);
    if (hoverNone) DrawRectangleRec(noneRect, Fade(ORANGE, 0.3f));
    DrawText("None (disable IK)", x + 5, itemY + 2, 11, hoverNone ? WHITE : LIGHTGRAY);
    
    if (hoverNone && mouseClicked)
    {
        if (setArmTargetFrame)
        {
            setArmTargetFrame(selectedArmIndex, -1);
        }
        ikTargetDropdownOpen = false;
    }
    itemY += itemHeight;
    
    // Frame options (with scroll)
    int startFrame = ikDropdownScrollOffset;
    int endFrame = std::min(numFrames, startFrame + maxVisibleItems - 1);
    
    for (int i = startFrame; i < endFrame; ++i)
    {
        const auto& frame = cachedSnapshot->frames[i];
        
        Rectangle frameRect = {(float)x, (float)itemY, (float)width, (float)itemHeight};
        bool hoverFrame = CheckCollisionPointRec(mousePos, frameRect);
        
        // Highlight current target
        int currentTarget = getArmTargetFrame ? getArmTargetFrame(selectedArmIndex) : -1;
        bool isCurrentTarget = (static_cast<int>(frame.id) == currentTarget);
        
        if (isCurrentTarget)
        {
            DrawRectangleRec(frameRect, Fade(GREEN, 0.3f));
        }
        else if (hoverFrame)
        {
            DrawRectangleRec(frameRect, Fade(SKYBLUE, 0.3f));
        }
        
        std::stringstream ss;
        ss << frame.name << " (ID:" << frame.id << ")";
        Color textColor = isCurrentTarget ? GREEN : (hoverFrame ? WHITE : LIGHTGRAY);
        DrawText(ss.str().c_str(), x + 5, itemY + 2, 11, textColor);
        
        if (hoverFrame && mouseClicked)
        {
            if (setArmTargetFrame)
            {
                setArmTargetFrame(selectedArmIndex, frame.id);
            }
            ikTargetDropdownOpen = false;
        }
        itemY += itemHeight;
    }
    
    // Scroll indicator if needed
    if (numFrames > maxVisibleItems - 1)
    {
        if (ikDropdownScrollOffset > 0)
        {
            DrawText("^ scroll up", x + 5, y - 12, 9, GRAY);
        }
        if (endFrame < numFrames)
        {
            DrawText("v scroll down", x + 5, y + dropdownHeight + 2, 9, GRAY);
        }
    }
    
    // Scroll with mouse wheel when hovering dropdown
    Rectangle dropdownArea = {(float)x, (float)y, (float)width, (float)dropdownHeight};
    if (CheckCollisionPointRec(mousePos, dropdownArea))
    {
        float wheel = GetMouseWheelMove();
        if (wheel != 0)
        {
            ikDropdownScrollOffset -= (int)wheel;
            ikDropdownScrollOffset = std::max(0, std::min(ikDropdownScrollOffset, numFrames - maxVisibleItems + 1));
        }
    }
    
    // Close dropdown if clicking outside (but not on the button that opened it)
    // The button is at y - 22 approximately, so expand the check area upward
    Rectangle expandedArea = {(float)x - 70, (float)(y - 25), (float)(width + 70), (float)(dropdownHeight + 25)};
    if (mouseClicked && !CheckCollisionPointRec(mousePos, expandedArea))
    {
        ikTargetDropdownOpen = false;
    }
}

// ============================================================================
// Frame Rendering & UI
// ============================================================================

void Viz::renderFrames(const SimSnapshot &snapshot)
{
    for (const auto& frame : snapshot.frames)
    {
        Eigen::Vector3d pos = frame.worldTransform.translation();
        // Swap Y/Z for raylib coordinate system
        Vector3 framePos = {(float)pos.x(), (float)pos.z(), (float)pos.y()};
        
        // Draw frame as a small cube + axes
        bool isSelected = (selectedFrameId == static_cast<int>(frame.id));
        bool isIKTarget = false;
        
        // Check if this frame is an IK target for any arm
        for (size_t i = 0; i < snapshot.armTargetFrameIds.size(); ++i)
        {
            if (snapshot.armTargetFrameIds[i] == static_cast<int>(frame.id))
            {
                isIKTarget = true;
                break;
            }
        }
        
        // Draw frame marker
        Color frameColor = isIKTarget ? ORANGE : (isSelected ? YELLOW : MAGENTA);
        DrawCube(framePos, 0.08f, 0.08f, 0.08f, frameColor);
        DrawCubeWires(framePos, 0.10f, 0.10f, 0.10f, BLACK);
        
        // Draw coordinate axes at frame
        drawAxis(frame.worldTransform, 0.25f);
        
        // Draw label above frame
        Vector2 screenPos = GetWorldToScreen(framePos, camera);
        if (screenPos.x > 0 && screenPos.x < screenWidth && 
            screenPos.y > 0 && screenPos.y < screenHeight)
        {
            DrawText(frame.name.c_str(), (int)screenPos.x - 20, (int)screenPos.y - 20, 12, frameColor);
        }
    }
}

void Viz::handleFrameUI()
{
    // F key toggles frame menu
    if (IsKeyPressed(KEY_F) && !textInputMode && frameInputField == FrameInputField::None)
    {
        frameMenuMode = !frameMenuMode;
        frameDropdownState = FrameDropdownState::None;
        if (frameMenuMode)
        {
            std::cout << "Frame menu opened" << std::endl;
        }
        else
        {
            std::cout << "Frame menu closed" << std::endl;
        }
    }
    
    if (!frameMenuMode) return;
    
    // Handle frame dragging
    handleFrameDragging();
    
    // Handle transform text input mode
    if (frameInputField != FrameInputField::None)
    {
        int key = GetCharPressed();
        while (key > 0)
        {
            if ((key >= '0' && key <= '9') || key == '.' || key == '-')
            {
                if (frameTransformInputBuffer.length() < 12)
                {
                    frameTransformInputBuffer += (char)key;
                }
            }
            key = GetCharPressed();
        }
        
        if (IsKeyPressed(KEY_BACKSPACE) && !frameTransformInputBuffer.empty())
        {
            frameTransformInputBuffer.pop_back();
        }
        
        if (IsKeyPressed(KEY_ENTER) && !frameTransformInputBuffer.empty())
        {
            try
            {
                double value = std::stod(frameTransformInputBuffer);
                switch (frameInputField)
                {
                    case FrameInputField::PosX: framePositionInput[0] = value; break;
                    case FrameInputField::PosY: framePositionInput[1] = value; break;
                    case FrameInputField::PosZ: framePositionInput[2] = value; break;
                    case FrameInputField::RotX: frameRotationInput[0] = value; break;
                    case FrameInputField::RotY: frameRotationInput[1] = value; break;
                    case FrameInputField::RotZ: frameRotationInput[2] = value; break;
                    default: break;
                }
                
                // Apply position change
                if (frameInputField >= FrameInputField::PosX && frameInputField <= FrameInputField::PosZ)
                {
                    if (setFramePosition && selectedFrameId >= 0)
                    {
                        setFramePosition(selectedFrameId, 
                            framePositionInput[0], framePositionInput[1], framePositionInput[2]);
                    }
                }
                // Apply rotation change
                else if (frameInputField >= FrameInputField::RotX && frameInputField <= FrameInputField::RotZ)
                {
                    if (setFrameRotation && selectedFrameId >= 0)
                    {
                        // Convert degrees to radians
                        double rx = frameRotationInput[0] * M_PI / 180.0;
                        double ry = frameRotationInput[1] * M_PI / 180.0;
                        double rz = frameRotationInput[2] * M_PI / 180.0;
                        setFrameRotation(selectedFrameId, rx, ry, rz);
                    }
                }
            }
            catch (...) { }
            
            frameInputField = FrameInputField::None;
            frameTransformInputBuffer.clear();
        }
        
        if (IsKeyPressed(KEY_ESCAPE))
        {
            frameInputField = FrameInputField::None;
            frameTransformInputBuffer.clear();
        }
        
        return;  // Don't process other inputs while typing
    }
    
    // Handle frame selection by clicking (when not in dropdown or dragging)
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !IsCursorHidden() && cachedSnapshot 
        && frameDropdownState == FrameDropdownState::None && !isDraggingFrame)
    {
        ray = GetMouseRay(GetMousePosition(), camera);
        float closestDistance = 1000000.0f;
        int hitFrameId = -1;
        
        for (const auto& frame : cachedSnapshot->frames)
        {
            Eigen::Vector3d pos = frame.worldTransform.translation();
            Vector3 framePos = {(float)pos.x(), (float)pos.z(), (float)pos.y()};
            
            RayCollision hit = GetRayCollisionSphere(ray, framePos, 0.15f);
            if (hit.hit && hit.distance < closestDistance)
            {
                closestDistance = hit.distance;
                hitFrameId = frame.id;
            }
        }
        
        if (hitFrameId >= 0)
        {
            selectedFrameId = hitFrameId;
            
            for (const auto& frame : cachedSnapshot->frames)
            {
                if (static_cast<int>(frame.id) == selectedFrameId)
                {
                    Eigen::Vector3d pos = frame.worldTransform.translation();
                    framePositionInput[0] = pos.x();
                    framePositionInput[1] = pos.y();
                    framePositionInput[2] = pos.z();
                    
                    // Extract rotation as Euler angles (approximate)
                    Eigen::Matrix3d rot = frame.worldTransform.rotation();
                    Eigen::Vector3d euler = rot.eulerAngles(0, 1, 2);  // XYZ order
                    frameRotationInput[0] = euler.x() * 180.0 / M_PI;
                    frameRotationInput[1] = euler.y() * 180.0 / M_PI;
                    frameRotationInput[2] = euler.z() * 180.0 / M_PI;
                    
                    frameParentArmInput = frame.parentArmIndex;
                    frameParentJointInput = frame.parentJointIndex;
                    frameNameBuffer = frame.name;
                    break;
                }
            }
        }
    }
    
    // N key: create new frame
    if (IsKeyPressed(KEY_N) && createFrame)
    {
        unsigned int newId = createFrame("Frame_" + std::to_string(rand() % 1000));
        selectedFrameId = newId;
        std::cout << "Created new frame with ID: " << newId << std::endl;
    }
    
    // DELETE key: destroy selected frame
    if (IsKeyPressed(KEY_DELETE) && selectedFrameId >= 0 && destroyFrame)
    {
        if (destroyFrame(selectedFrameId))
        {
            std::cout << "Destroyed frame ID: " << selectedFrameId << std::endl;
            selectedFrameId = -1;
        }
    }
    
    // I key: set selected frame as IK target for selected arm
    if (IsKeyPressed(KEY_I) && selectedFrameId >= 0 && setArmTargetFrame)
    {
        setArmTargetFrame(selectedArmIndex, selectedFrameId);
        std::cout << "Set Arm " << selectedArmIndex << " IK target to Frame " << selectedFrameId << std::endl;
    }
    
    // U key: clear IK target for selected arm
    if (IsKeyPressed(KEY_U) && setArmTargetFrame)
    {
        setArmTargetFrame(selectedArmIndex, -1);
        std::cout << "Cleared IK target for Arm " << selectedArmIndex << std::endl;
    }
    
    // ESC closes dropdown
    if (IsKeyPressed(KEY_ESCAPE) && frameDropdownState != FrameDropdownState::None)
    {
        frameDropdownState = FrameDropdownState::None;
    }
}

void Viz::handleFrameDragging()
{
    if (selectedFrameId < 0 || !cachedSnapshot) return;
    
    // Find selected frame's current position
    Vector3 frameWorldPos = {0, 0, 0};
    for (const auto& frame : cachedSnapshot->frames)
    {
        if (static_cast<int>(frame.id) == selectedFrameId)
        {
            Eigen::Vector3d pos = frame.worldTransform.translation();
            frameWorldPos = {(float)pos.x(), (float)pos.z(), (float)pos.y()};
            break;
        }
    }
    
    // Start dragging on middle mouse button
    if (IsMouseButtonPressed(MOUSE_BUTTON_MIDDLE) && !IsCursorHidden())
    {
        ray = GetMouseRay(GetMousePosition(), camera);
        RayCollision hit = GetRayCollisionSphere(ray, frameWorldPos, 0.2f);
        
        if (hit.hit)
        {
            isDraggingFrame = true;
            // Create drag plane facing camera
            dragPlaneNormal = Vector3Normalize(Vector3Subtract(camera.position, frameWorldPos));
            dragOffset = Vector3Subtract(frameWorldPos, hit.point);
        }
    }
    
    // Continue dragging
    if (isDraggingFrame && IsMouseButtonDown(MOUSE_BUTTON_MIDDLE))
    {
        ray = GetMouseRay(GetMousePosition(), camera);
        
        // Intersect ray with drag plane
        float denom = Vector3DotProduct(dragPlaneNormal, ray.direction);
        if (fabs(denom) > 0.0001f)
        {
            Vector3 planePoint = frameWorldPos;
            float t = Vector3DotProduct(Vector3Subtract(planePoint, ray.position), dragPlaneNormal) / denom;
            
            if (t >= 0)
            {
                Vector3 newPos = Vector3Add(
                    Vector3Add(ray.position, Vector3Scale(ray.direction, t)),
                    dragOffset
                );
                
                // Convert back to Eigen/Sim coordinates (swap Y/Z back)
                framePositionInput[0] = newPos.x;
                framePositionInput[1] = newPos.z;  // raylib Y -> Eigen Z
                framePositionInput[2] = newPos.y;  // raylib Z -> Eigen Y
                
                if (setFramePosition)
                {
                    setFramePosition(selectedFrameId, 
                        framePositionInput[0], framePositionInput[1], framePositionInput[2]);
                }
            }
        }
    }
    
    // Stop dragging
    if (IsMouseButtonReleased(MOUSE_BUTTON_MIDDLE))
    {
        isDraggingFrame = false;
    }
}

void Viz::renderFrameUI()
{
    if (!frameMenuMode) return;
    
    const int panelX = 10;
    const int panelWidth = 320;
    int panelY = screenHeight - 280;
    int y = panelY;
    int x = panelX;
    
    DrawRectangle(x - 5, y - 5, panelWidth, 275, Fade(DARKGRAY, 0.9f));
    DrawRectangleLines(x - 5, y - 5, panelWidth, 275, WHITE);
    
    DrawText("FRAME MENU (F to toggle)", x, y, 16, YELLOW);
    y += 22;
    
    // Show frame count
    int frameCount = cachedSnapshot ? cachedSnapshot->frames.size() : 0;
    std::stringstream ss;
    ss << "Frames: " << frameCount;
    DrawText(ss.str().c_str(), x, y, 12, WHITE);
    y += 18;
    
    // Selected frame info
    if (selectedFrameId >= 0)
    {
        ss.str("");
        ss << "Selected: ID " << selectedFrameId << " (" << frameNameBuffer << ")";
        DrawText(ss.str().c_str(), x, y, 12, YELLOW);
        y += 18;
        
        // Position inputs (clickable)
        DrawText("Position:", x, y, 11, WHITE);
        y += 14;
        
        int inputX = x + 10;
        renderFrameTransformInput(inputX, y, "X:", FrameInputField::PosX, framePositionInput[0]);
        renderFrameTransformInput(inputX + 90, y, "Y:", FrameInputField::PosY, framePositionInput[1]);
        renderFrameTransformInput(inputX + 180, y, "Z:", FrameInputField::PosZ, framePositionInput[2]);
        y += 18;
        
        // Rotation inputs (clickable)
        DrawText("Rotation (deg):", x, y, 11, WHITE);
        y += 14;
        
        renderFrameTransformInput(inputX, y, "X:", FrameInputField::RotX, frameRotationInput[0]);
        renderFrameTransformInput(inputX + 90, y, "Y:", FrameInputField::RotY, frameRotationInput[1]);
        renderFrameTransformInput(inputX + 180, y, "Z:", FrameInputField::RotZ, frameRotationInput[2]);
        y += 20;
        
        // Parent dropdown button
        DrawText("Parent:", x, y, 11, WHITE);
        
        ss.str("");
        if (frameParentArmInput < 0)
        {
            ss << "[World]";
        }
        else
        {
            ss << "[Arm " << frameParentArmInput << " J" << frameParentJointInput << "]";
        }
        
        Rectangle dropdownBtn = {(float)(x + 50), (float)(y - 2), 120, 16};
        bool hoverDropdown = CheckCollisionPointRec(GetMousePosition(), dropdownBtn);
        DrawRectangleRec(dropdownBtn, hoverDropdown ? Fade(SKYBLUE, 0.5f) : Fade(GRAY, 0.3f));
        DrawRectangleLinesEx(dropdownBtn, 1, WHITE);
        DrawText(ss.str().c_str(), x + 55, y, 11, LIGHTGRAY);
        
        if (hoverDropdown && IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
        {
            frameDropdownState = FrameDropdownState::SelectArm;
            dropdownScrollOffset = 0;
        }
        y += 22;
        
        // Render dropdown if open
        if (frameDropdownState != FrameDropdownState::None)
        {
            renderFrameDropdown(x + 50, y, 150);
        }
    }
    else
    {
        DrawText("No frame selected", x, y, 12, GRAY);
        DrawText("(Click frame in 3D view to select)", x, y + 14, 10, DARKGRAY);
        y += 30;
    }
    
    y += 5;
    
    // Show current arm's IK target
    if (getArmTargetFrame)
    {
        int targetId = getArmTargetFrame(selectedArmIndex);
        ss.str("");
        if (targetId >= 0)
        {
            ss << "Arm " << selectedArmIndex << " IK target: Frame " << targetId;
            DrawText(ss.str().c_str(), x, y, 12, ORANGE);
        }
        else
        {
            ss << "Arm " << selectedArmIndex << " IK target: None";
            DrawText(ss.str().c_str(), x, y, 12, GRAY);
        }
    }
    y += 22;
    
    // Controls help
    DrawText("Controls:", x, y, 11, WHITE);
    y += 14;
    DrawText("  N: New frame | DEL: Delete frame", x, y, 10, GRAY);
    y += 12;
    DrawText("  I: Set as IK target | U: Clear IK target", x, y, 10, GRAY);
    y += 12;
    DrawText("  Middle-click + drag: Move frame in 3D", x, y, 10, GRAY);
    y += 12;
    DrawText("  Click values to type new position/rotation", x, y, 10, GRAY);
}

void Viz::renderFrameDropdown(int x, int y, int width)
{
    Vector2 mousePos = GetMousePosition();
    const int itemHeight = 18;
    const int maxVisibleItems = 8;
    
    if (frameDropdownState == FrameDropdownState::SelectArm)
    {
        // Get number of arms
        int numArms = cachedSnapshot ? cachedSnapshot->armTransforms.size() : 0;
        int numItems = numArms + 1;  // +1 for "World" option
        
        int visibleItems = std::min(numItems, maxVisibleItems);
        int dropdownHeight = visibleItems * itemHeight + 4;
        
        DrawRectangle(x, y, width, dropdownHeight, Fade(DARKBLUE, 0.95f));
        DrawRectangleLinesEx({(float)x, (float)y, (float)width, (float)dropdownHeight}, 1, WHITE);
        
        int itemY = y + 2;
        
        // World option
        Rectangle worldRect = {(float)x, (float)itemY, (float)width, (float)itemHeight};
        bool hoverWorld = CheckCollisionPointRec(mousePos, worldRect);
        if (hoverWorld) DrawRectangleRec(worldRect, Fade(SKYBLUE, 0.3f));
        DrawText("World", x + 5, itemY + 2, 11, hoverWorld ? WHITE : LIGHTGRAY);
        
        if (hoverWorld && IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
        {
            if (setFrameParent && selectedFrameId >= 0)
            {
                setFrameParent(selectedFrameId, -1, -1);
                frameParentArmInput = -1;
                frameParentJointInput = -1;
            }
            frameDropdownState = FrameDropdownState::None;
        }
        itemY += itemHeight;
        
        // Arm options
        for (int i = 0; i < numArms && (i + 1) < visibleItems; ++i)
        {
            Rectangle armRect = {(float)x, (float)itemY, (float)width, (float)itemHeight};
            bool hoverArm = CheckCollisionPointRec(mousePos, armRect);
            if (hoverArm) DrawRectangleRec(armRect, Fade(SKYBLUE, 0.3f));
            
            std::stringstream ss;
            ss << "Arm " << i << " >";
            DrawText(ss.str().c_str(), x + 5, itemY + 2, 11, hoverArm ? WHITE : LIGHTGRAY);
            
            if (hoverArm && IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
            {
                frameParentArmInput = i;
                frameDropdownState = FrameDropdownState::SelectJoint;
                dropdownScrollOffset = 0;
            }
            itemY += itemHeight;
        }
    }
    else if (frameDropdownState == FrameDropdownState::SelectJoint)
    {
        // Get number of joints for selected arm
        unsigned int numJoints = 0;
        if (getArmNumJoints && frameParentArmInput >= 0)
        {
            numJoints = getArmNumJoints(frameParentArmInput);
        }
        else if (cachedSnapshot && frameParentArmInput >= 0 
                 && (size_t)frameParentArmInput < cachedSnapshot->armTransforms.size())
        {
            numJoints = cachedSnapshot->armTransforms[frameParentArmInput].size();
        }
        
        int numItems = numJoints + 1;  // +1 for "Back" option
        int visibleItems = std::min((int)numItems, maxVisibleItems);
        int dropdownHeight = visibleItems * itemHeight + 4;
        
        DrawRectangle(x, y, width, dropdownHeight, Fade(DARKBLUE, 0.95f));
        DrawRectangleLinesEx({(float)x, (float)y, (float)width, (float)dropdownHeight}, 1, WHITE);
        
        int itemY = y + 2;
        
        // Back option
        Rectangle backRect = {(float)x, (float)itemY, (float)width, (float)itemHeight};
        bool hoverBack = CheckCollisionPointRec(mousePos, backRect);
        if (hoverBack) DrawRectangleRec(backRect, Fade(ORANGE, 0.3f));
        DrawText("< Back", x + 5, itemY + 2, 11, hoverBack ? WHITE : ORANGE);
        
        if (hoverBack && IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
        {
            frameDropdownState = FrameDropdownState::SelectArm;
        }
        itemY += itemHeight;
        
        // Joint options (with scroll)
        int startJoint = dropdownScrollOffset;
        int endJoint = std::min((int)numJoints, startJoint + maxVisibleItems - 1);
        
        for (int j = startJoint; j < endJoint; ++j)
        {
            Rectangle jointRect = {(float)x, (float)itemY, (float)width, (float)itemHeight};
            bool hoverJoint = CheckCollisionPointRec(mousePos, jointRect);
            if (hoverJoint) DrawRectangleRec(jointRect, Fade(SKYBLUE, 0.3f));
            
            std::stringstream ss;
            ss << "Joint " << j;
            DrawText(ss.str().c_str(), x + 5, itemY + 2, 11, hoverJoint ? WHITE : LIGHTGRAY);
            
            if (hoverJoint && IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
            {
                if (setFrameParent && selectedFrameId >= 0)
                {
                    setFrameParent(selectedFrameId, frameParentArmInput, j);
                    frameParentJointInput = j;
                }
                frameDropdownState = FrameDropdownState::None;
            }
            itemY += itemHeight;
        }
        
        // Scroll with mouse wheel
        float wheel = GetMouseWheelMove();
        if (wheel != 0)
        {
            dropdownScrollOffset -= (int)wheel;
            dropdownScrollOffset = std::max(0, std::min(dropdownScrollOffset, (int)numJoints - maxVisibleItems + 1));
        }
    }
}

bool Viz::renderFrameTransformInput(int x, int& y, const char* label, FrameInputField field, float& value)
{
    Vector2 mousePos = GetMousePosition();
    
    DrawText(label, x, y, 10, LIGHTGRAY);
    
    Rectangle valueRect = {(float)(x + 18), (float)(y - 2), 65, 14};
    bool hover = CheckCollisionPointRec(mousePos, valueRect);
    bool isEditing = (frameInputField == field);
    
    if (isEditing)
    {
        DrawRectangleRec(valueRect, Fade(YELLOW, 0.3f));
        DrawRectangleLinesEx(valueRect, 1, YELLOW);
        std::string displayText = frameTransformInputBuffer + "_";
        DrawText(displayText.c_str(), x + 20, y, 10, YELLOW);
    }
    else
    {
        DrawRectangleRec(valueRect, hover ? Fade(SKYBLUE, 0.2f) : Fade(BLACK, 0.2f));
        DrawRectangleLinesEx(valueRect, 1, hover ? SKYBLUE : GRAY);
        
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << value;
        DrawText(ss.str().c_str(), x + 20, y, 10, hover ? WHITE : LIGHTGRAY);
        
        if (hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
        {
            frameInputField = field;
            frameTransformInputBuffer.clear();
            return true;
        }
    }
    
    return false;
}