/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_MOUSE_CALLBACK_H
#define POSITIONBASEDDYNAMICS_MOUSE_CALLBACK_H

#include <igl/opengl/glfw/Viewer.h>

namespace HINAVIEWER::MOUSE_CALLBACK
{
    bool mouse_down(igl::opengl::glfw::Viewer &viewer, int x, int y);

    bool mouse_up(igl::opengl::glfw::Viewer &viewer, int x, int y);

    bool mouse_move(igl::opengl::glfw::Viewer &viewer, int x, int y);
}


#endif //POSITIONBASEDDYNAMICS_MOUSE_CALLBACK_H
