/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_PBD_VIEWER_H
#define POSITIONBASEDDYNAMICS_PBD_VIEWER_H

#define IMGUI_DEFINE_MATH_OPERATORS

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>

namespace HINAVIEWER
{
    class PBDViewer
    {
    public:
        explicit PBDViewer(int width = 0, int height = 0, Eigen::Vector4f color = Eigen::Vector4f::Ones());

        void launch_rendering(const std::string &window_name = "Hello HinaPE");

        igl::opengl::glfw::Viewer &viewer();

    public:
        PBDViewer &set_background_color(const Eigen::Vector4f &color);

        PBDViewer &set_max_fps(double fps);

        PBDViewer &set_full_screen();

    protected:
        void setup_menu();

        void setup_callback();

    protected:
        igl::opengl::glfw::Viewer viewer_;
        igl::opengl::glfw::imgui::ImGuiMenu menu_;

    protected:
        int width_, height_;
        Eigen::Vector4f color_;
        bool is_full_screen = false;
    };
}

#endif //POSITIONBASEDDYNAMICS_PBD_VIEWER_H