/**
 * @author YC XIANG.
 * @date 01/29, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_VIEWER_H
#define POSITIONBASEDDYNAMICS_VIEWER_H

#include "pick_nearest_vertices.h"
#include "inspector.h"

#include <igl/opengl/glfw/Viewer.h>

#define IMGUI_DEFINE_MATH_OPERATORS

#include <igl/unproject.h>

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>

#include <vector>
#include <array>
#include <deque>

namespace pbd_viewer
{
    /// set up igl viewer system
    void setup(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot, bool ps_plot = false);

    /// int object
    void add_object_to_scene(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F, Eigen::RowVector3d color);

    /// update rendering vertices
    void update_vertex_positions(unsigned int id, Eigen::Ref<const Eigen::VectorXd> pos);

    /// get igl viewer
    igl::opengl::glfw::Viewer &viewer();

    /// mouse callback
    bool mouse_down(igl::opengl::glfw::Viewer &viewer, int x, int y);

    bool mouse_up(igl::opengl::glfw::Viewer &viewer, int x, int y);

    bool mouse_move(igl::opengl::glfw::Viewer &viewer, int x, int y);

    const Eigen::Vector3d &mouse_world();

    const Eigen::Vector3d &mouse_drag_world();

    const std::vector<unsigned int> &picked_vertices();

    bool is_mouse_dragging();

    // Custom Plot Canvas
    void track(pbd_inspector::Trackable *trackable, int index = 1);
}

#endif //POSITIONBASEDDYNAMICS_VIEWER_H
