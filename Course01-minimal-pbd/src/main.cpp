#include <igl/opengl/glfw/Viewer.h>

#include "Eigen/Eigen"

#include "Discregrid/All"

#include <iostream>
#include <string>

#include "Utils/visualization/viewer.h"

int main(int argc, char *argv[])
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    std::string bunny_path   = std::string(PBD_MODEL_DIR) + "bun_zipper.ply";
    std::string mitsuba_path = std::string(PBD_TEXTURE_DIR) + "mitsuba.png";

    Discregrid::TriangleMesh mesh(bunny_path);
    
    igl::opengl::glfw::Viewer viewer;
    viewer.load_mesh_from_file(bunny_path);
    viewer.launch();
}
