#include "mouse_callback.h"
#include "utils.h"

namespace HINAVIEWER::MOUSE_CALLBACK
{
    //picking variables
    std::vector<unsigned int> g_picked_vertices;
    unsigned int g_selected_obj;
    //mouse UI state variables
    bool g_mouse_dragging = false;
    Eigen::Vector3d g_mouse_win; //mouse window coordinates
    Eigen::Vector3d g_mouse_drag; //last mouse drag vector
    Eigen::Vector3d g_mouse_world;
    Eigen::Vector3d g_mouse_drag_world; //mouse drag delta in the world space
}


bool HINAVIEWER::MOUSE_CALLBACK::mouse_down(igl::opengl::glfw::Viewer &viewer, int x, int y)
{
    g_mouse_win = Eigen::Vector3d(viewer.current_mouse_x, viewer.core().viewport(3) - viewer.current_mouse_y, 0.);
    igl::unproject(
            g_mouse_win,
            viewer.core().view,
            viewer.core().proj,
            viewer.core().viewport,
            g_mouse_world);

    if (HINAVIEWER::UTILS::pick_nearest_vertices(g_picked_vertices, g_mouse_win,
                                                 viewer.core().view, viewer.core().proj, viewer.core().viewport,
                                                 viewer.data(0).V, viewer.data(0).F, 0.1))
    {

        g_selected_obj = 0;
        g_mouse_dragging = true;
    }
    return false;
}

bool HINAVIEWER::MOUSE_CALLBACK::mouse_up(igl::opengl::glfw::Viewer &viewer, int x, int y)
{
    g_mouse_dragging = false;
    g_picked_vertices.clear();
    g_mouse_drag_world.setZero();
    return false;
}

bool HINAVIEWER::MOUSE_CALLBACK::mouse_move(igl::opengl::glfw::Viewer &viewer, int x, int y)
{
    g_mouse_drag = Eigen::Vector3d(viewer.current_mouse_x, viewer.core().viewport(3) - viewer.current_mouse_y, 0.) - g_mouse_win;
    g_mouse_win = Eigen::Vector3d(viewer.current_mouse_x, viewer.core().viewport(3) - viewer.current_mouse_y, 0.);

    igl::unproject(
            g_mouse_win,
            viewer.core().view,
            viewer.core().proj,
            viewer.core().viewport,
            g_mouse_drag_world);

    g_mouse_drag_world -= g_mouse_world;

    igl::unproject(
            g_mouse_win,
            viewer.core().view,
            viewer.core().proj,
            viewer.core().viewport,
            g_mouse_world);

    if (g_mouse_dragging && !g_picked_vertices.empty())
    {
        return true;
    }

    return false;
}

const Eigen::Vector3d &HINAVIEWER::MOUSE_CALLBACK::mouse_world()
{
    return g_mouse_world;
}

const Eigen::Vector3d &HINAVIEWER::MOUSE_CALLBACK::mouse_drag_world()
{
    return g_mouse_drag_world;
}

const std::vector<unsigned int> &HINAVIEWER::MOUSE_CALLBACK::picked_vertices()
{
    return g_picked_vertices;
}

bool HINAVIEWER::MOUSE_CALLBACK::is_mouse_dragging()
{
    return g_mouse_dragging;
}