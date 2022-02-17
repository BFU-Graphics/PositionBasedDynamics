/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_PBD_VIEWER_H
#define POSITIONBASEDDYNAMICS_PBD_VIEWER_H

#define IMGUI_DEFINE_MATH_OPERATORS

#include "src/inspector.h"
#include "src/renderable.h"

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

        void update_vertex_positions(unsigned int id, Eigen::Ref<const Eigen::MatrixXd> V);

        igl::opengl::glfw::Viewer &viewer();

    public:
        int record(HINAVIEWER::RENDERABLE::Renderable *renderable);

        void track(HINAVIEWER::INSPECTOR::Trackable *trackable, int index = 1);

    public:
        PBDViewer &set_background_color(const Eigen::Vector4f &color);

        PBDViewer &set_max_fps(double fps);

        PBDViewer &set_full_screen();

        PBDViewer &focus_object(int ID);

        PBDViewer &show_inspector();

    protected:
        void setup_menu();

        void setup_callback();

        void setup_inspector();

    protected:
        igl::opengl::glfw::Viewer viewer_;
        igl::opengl::glfw::imgui::ImGuiMenu menu_;
        std::vector<HINAVIEWER::INSPECTOR::Inspector *> inspector_list_;
        std::vector<int> object_list_;

    protected:
        int width_, height_;
        Eigen::Vector4f color_;
        bool is_full_screen = false;
        bool enable_menu = true;
        bool enable_inspector = false;
        bool enable_custom_mouse_callback = false;
    };
}

#endif //POSITIONBASEDDYNAMICS_PBD_VIEWER_H