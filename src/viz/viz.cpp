#include "viz.hpp"
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

void Viz::setGetAngleCallback(std::function<double(unsigned int, unsigned int)> callback)
{
    getCurrentAngle = callback;
}

void Viz::setGetMDHTableCallback(std::function<std::vector<MDHRow>(unsigned int)> callback)
{
    getMDHTable = callback;
}

void Viz::setMDHControlCallback(std::function<bool(unsigned int, unsigned int, const std::string&, double)> callback)
{
    setMDHParam = callback;
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
                    armCenterOfMass.z
                };
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
    else if (IsKeyDown(KEY_PAGE_UP)) // Use Page Up instead of UP arrow (conflicts with camera)
    {
        currentTargetAngle += angleIncrement * 0.5f;
        angleChanged = true;
    }
    else if (IsKeyDown(KEY_PAGE_DOWN)) // Use Page Down instead of DOWN arrow (conflicts with camera)
    {
        currentTargetAngle -= angleIncrement * 0.5f;
        angleChanged = true;
    }

    // Reset angle to zero
    if (IsKeyPressed(KEY_R))
    {
        currentTargetAngle = 0.0;
        angleChanged = true;
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
        ss << "Arm: " << selectedArmIndex << " | Joint: " << selectedJointIndex;
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
    
    DrawText("Right click: camera | Click joint: select | Click near arm: MDH table", 10, screenHeight - 30, 10, GRAY);
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
        (float)(origin.x() + yAxis.x()),
        (float)(origin.z() + yAxis.z()), // Swap Y/Z
        (float)(origin.y() + yAxis.y())};
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
                    armCenterOfMass.z
                };
                
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
                const char* paramNames[] = {"alpha", "a", "d_offset", "theta_offset"};
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
    const int colWidths[] = {35, 65, 65, 65, 65, 35}; // Link, α, a, d_off, θ_off, Type
    const int headerHeight = 25;
    int tableWidth = 35 + 65 + 65 + 65 + 65 + 35;
    int tableHeight = headerHeight + (cachedMDHTable.size() + 1) * rowHeight + 40;
    
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
        if (tableX < 10) tableX = 10;
        if (tableY < 50) tableY = 50;
        if (tableX + tableWidth > screenWidth - 10) tableX = screenWidth - tableWidth - 10;
        if (tableY + tableHeight > screenHeight - 50) tableY = screenHeight - tableHeight - 10;
    }
    
    // Background panel with transparency
    DrawRectangle(tableX - 10, tableY - 20, tableWidth + 20, tableHeight+20, Fade(BLACK, 0.15f));
    DrawRectangleLines(tableX - 10, tableY - 20, tableWidth + 20, tableHeight+20, BLACK);
    
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
    DrawText("Type", x, y, 11, BLACK);
    
    y += rowHeight;
    DrawLine(tableX, y - 2, tableX + tableWidth, y - 2, BLACK);
    
    // Mouse position for cell detection
    Vector2 mousePos = GetMousePosition();
    bool mouseClicked = IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !textInputMode;
    
    // Data rows
    for (size_t i = 0; i < cachedMDHTable.size(); ++i)
    {
        const auto& row = cachedMDHTable[i];
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
        DrawText(ss.str().c_str(), x+5, y, 11, Fade(BLACK,0.8f));
        x += colWidths[0];
        
        // Helper to draw cells and detect clicks
        auto drawCell = [&](int col, double value) {
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
                const char* colNames[] = {"alpha", "a", "d_offset", "theta_offset"};
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
                ss.str(""); ss << std::fixed << std::setprecision(2) << value;
                Color cellColor = (isSelectedRow && mdhSelectedCol == col) ? GREEN : 
                                 (mouseOverCell ? YELLOW : BLACK);
                DrawText(ss.str().c_str(), x, y, 11, Fade(cellColor,0.8));
            }
        };
        
        drawCell(0, row.alpha);
        x += colWidths[1];
        
        drawCell(1, row.a);
        x += colWidths[2];
        
        drawCell(2, row.dOffset);
        x += colWidths[3];
        
        drawCell(3, row.thetaOffset);
        x += colWidths[4];
        
        // Type (not editable)
        DrawText(row.isRevolute ? "R" : "P", x, y, 11, row.isRevolute ? SKYBLUE : MAGENTA);
        
        y += rowHeight;
    }
    
    // Instructions
    y += 10;
    DrawText("Click cell to edit | ENTER: Apply | ESC: Cancel", tableX, y, 10, GRAY);
}
