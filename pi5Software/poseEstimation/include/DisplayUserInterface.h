#ifndef LINE_QUALITY_USERINTERFACE_H
#define LINE_QUALITY_USERINTERFACE_H

#include <stdio.h>
#include <iostream>
#include "include/glad/glad.h"
#include <glfw3.h>
#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include <string_view>
#include <vector>

#include "DisplayData.h"

using namespace std;

class DisplayUserInterface {
    public:
    GLFWwindow* window = nullptr;
    ImGuiContext* imguiContext = nullptr;
    ImGuiIO* io = nullptr;

    // Our state
    bool show_demo_window = false;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
    bool items[20] = {};
    Visibility& visibility;

    // Interface state
    bool exit = false;
    bool run = false;
    int simulationSteps = 0;
    int stepTime = 100;

    explicit DisplayUserInterface(Visibility& pVisibility) : visibility(pVisibility){
        // Create window with graphics context
        float main_scale = ImGui_ImplGlfw_GetContentScaleForMonitor(glfwGetPrimaryMonitor()); // Valid on GLFW 3.3+ only
        window = glfwCreateWindow((int)(1280 * main_scale), (int)(800 * main_scale), "User interface", nullptr, nullptr);
        if (window == nullptr) {
            std::cerr << "User Interface - Failed to create GLFW window\n";
        }

        // Make context current
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);

        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
            std::cerr << "Failed to initialize GLAD\n";
            return;
        }

        const char* glsl_version = "#version 330";

        // Setup Dear ImGui context
        IMGUI_CHECKVERSION();
        imguiContext = ImGui::CreateContext();
        ImGui::SetCurrentContext(imguiContext);
        io = &ImGui::GetIO();
        (void)io;
        io->ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
        io->ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

        // Setup Dear ImGui style
        ImGui::StyleColorsDark();
        //ImGui::StyleColorsLight();

        // Setup scaling
        ImGuiStyle& style = ImGui::GetStyle();
        style.ScaleAllSizes(main_scale);        // Bake a fixed style scale. (until we have a solution for dynamic style scaling, changing this requires resetting Style + calling this again)
        style.FontScaleDpi = main_scale;        // Set initial font scale. (using io.ConfigDpiScaleFonts=true makes this unnecessary. We leave both here for documentation purpose)

        // Setup Platform/Renderer backends
        ImGui_ImplGlfw_InitForOpenGL(window, true);
        ImGui_ImplOpenGL3_Init(glsl_version);

        for (auto& b : items) {b=true;}
    }
    
    void update() {
        if(!window) {cout<<"Warning no GLFW window!"<<std::endl; return;}
        if(!imguiContext) {cout<<"Warning No ImGui context!"<<std::endl; return;}
        
        ImGui::SetCurrentContext(imguiContext); 
        glfwMakeContextCurrent(window);
        glfwPollEvents();
        if (glfwGetWindowAttrib(window, GLFW_ICONIFIED) != 0)
        {
            ImGui_ImplGlfw_Sleep(10);
            return;
        }

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
        if (show_demo_window)
            ImGui::ShowDemoWindow(&show_demo_window);

        // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
        {
            static float f = 0.0f;

            ImGui::Begin("Settings");
            ImGui::SeparatorText("General");
            if (ImGui::Button("Exit")) exit = true;
            ImGui::SeparatorText("Lidar");
            if (ImGui::TreeNode("Point visiblity"))
            {
                // If you have an array of checkboxes, you may want to use NoAutoSelect + NoAutoClear and the ImGuiSelectionExternalStorage helper.
                static ImGuiMultiSelectFlags flags = ImGuiMultiSelectFlags_NoAutoSelect | ImGuiMultiSelectFlags_NoAutoClear | ImGuiMultiSelectFlags_ClearOnEscape;
                //ImGui::CheckboxFlags("ImGuiMultiSelectFlags_NoAutoSelect", &flags, ImGuiMultiSelectFlags_NoAutoSelect);
                //ImGui::CheckboxFlags("ImGuiMultiSelectFlags_NoAutoClear", &flags, ImGuiMultiSelectFlags_NoAutoClear);
                //ImGui::CheckboxFlags("ImGuiMultiSelectFlags_BoxSelect2d", &flags, ImGuiMultiSelectFlags_BoxSelect2d); // Cannot use ImGuiMultiSelectFlags_BoxSelect1d as checkboxes are varying width.

                if (ImGui::BeginChild("##Basket", ImVec2(-FLT_MIN, ImGui::GetFontSize() * 20), ImGuiChildFlags_Borders | ImGuiChildFlags_ResizeY))
                {
                    ImGuiMultiSelectIO* ms_io = ImGui::BeginMultiSelect(flags, -1, IM_ARRAYSIZE(items));
                    ImGuiSelectionExternalStorage storage_wrapper;
                    storage_wrapper.UserData = (void*)visibility.showPointType;
                    storage_wrapper.AdapterSetItemSelected = [](ImGuiSelectionExternalStorage* self, int n, bool selected) { bool* array = (bool*)self->UserData; array[n] = selected; };
                    storage_wrapper.ApplyRequests(ms_io);
                    for (int n = 0; n < DisplayPointType::DISPLAY_POINT_TYPE_COUNT; n++)
                    {
                        char label[32];
                        //sprintf(label, DisplayPointTypeNames[n]);
                        ImGui::SetNextItemSelectionUserData(n);
                        ImGui::Checkbox(label, &items[n]);
                    }
                    ms_io = ImGui::EndMultiSelect();
                    storage_wrapper.ApplyRequests(ms_io);
                }
                ImGui::EndChild();

                ImGui::TreePop();
            }

            ImGui::SeparatorText("Simulation");
            if (!run) {if (ImGui::Button("Start")){run = true;}}
            else if (ImGui::Button("Stop")){run = false;}
            ImGui::SameLine();
            if (ImGui::Button("Step")) simulationSteps++;
            ImGui::InputInt("Time per simulation step", &stepTime, 10, 100);
            stepTime = std::clamp(stepTime, 0, 1000);

            ImGui::SeparatorText("ImGui");
            ImGui::Checkbox("Show ImGUI Demo Window", &show_demo_window);      // Edit bools storing our window open/close state
            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io->Framerate, io->Framerate);
            ImGui::End();
        }

        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }
};

#endif //LINE_QUALITY_USERINTERFACE_H
