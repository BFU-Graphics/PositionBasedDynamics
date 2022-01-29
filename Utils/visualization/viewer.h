/**
 * @author YC XIANG.
 * @date 01/29, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_VIEWER_H
#define POSITIONBASEDDYNAMICS_VIEWER_H

#include <igl/opengl/glfw/Viewer.h>

namespace PBD
{
    class Viewer
    {
    public:
        void launch();

    public:
        void set_texture(int data_id, const std::string &path);

    public:
        igl::opengl::glfw::Viewer viewer;
    };
}


#endif //POSITIONBASEDDYNAMICS_VIEWER_H
