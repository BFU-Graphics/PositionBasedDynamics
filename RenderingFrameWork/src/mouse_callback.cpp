/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#include "mouse_callback.h"
#include "utils.h"

namespace HINAVIEWER
{
    extern std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXi>> g_geometry;
}

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
                                                 viewer.data().V, HINAVIEWER::g_geometry[0].second, 0.1))
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