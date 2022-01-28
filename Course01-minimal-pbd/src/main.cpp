#include <igl/opengl/glfw/Viewer.h>
#include <igl/readOBJ.h>
#include <string>

#include "Discregrid/All"

int main(int argc, char *argv[])
{
    Discregrid::TriangleMesh mesh();


    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    std::string path = std::string(PBD_MODEL_DIR) + "starry_fish.obj";

    Discregrid::TriangleMesh my_mesh(path);

    igl::readOBJ(path, V, F);
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    viewer.launch();
}
