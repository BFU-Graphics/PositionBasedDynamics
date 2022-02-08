/**
 * @author YC XIANG.
 * @date 01/29, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_VIEWER_H
#define POSITIONBASEDDYNAMICS_VIEWER_H

#include <igl/opengl/glfw/Viewer.h>

#define IMGUI_DEFINE_MATH_OPERATORS

#include <igl/unproject.h>
#include "pick_nearest_vertices.h"

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>

namespace pbd_viewer
{
    igl::opengl::glfw::Viewer &viewer();

    /// set up igl viewer system
    void setup(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot, bool ps_plot = false);

    /// int object
    void add_object_to_scene(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, Eigen::RowVector3d color);

    /// mouse callback
    bool mouse_down(igl::opengl::glfw::Viewer &viewer, int x, int y);

    bool mouse_up(igl::opengl::glfw::Viewer &viewer, int x, int y);

    bool mouse_move(igl::opengl::glfw::Viewer &viewer, int x, int y);

    const Eigen::Vector3d & mouse_world();

    const Eigen::Vector3d & mouse_drag_world();

    bool is_mouse_dragging();

    const std::vector<unsigned int> & picked_vertices();
}


#endif //POSITIONBASEDDYNAMICS_VIEWER_H
