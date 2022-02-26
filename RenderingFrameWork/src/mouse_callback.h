#ifndef POSITIONBASEDDYNAMICS_MOUSE_CALLBACK_H
#define POSITIONBASEDDYNAMICS_MOUSE_CALLBACK_H

#include <igl/opengl/glfw/Viewer.h>

namespace HINAVIEWER::MOUSE_CALLBACK
{
    bool mouse_down(igl::opengl::glfw::Viewer &viewer, int x, int y);

    bool mouse_up(igl::opengl::glfw::Viewer &viewer, int x, int y);

    bool mouse_move(igl::opengl::glfw::Viewer &viewer, int x, int y);

    const Eigen::Vector3d & mouse_world();

    const Eigen::Vector3d & mouse_drag_world();

    const std::vector<unsigned int> & picked_vertices();

    bool is_mouse_dragging();
}


#endif //POSITIONBASEDDYNAMICS_MOUSE_CALLBACK_H
