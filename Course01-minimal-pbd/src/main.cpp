#include <igl/opengl/glfw/Viewer.h>
#include <string>

#include "Discregrid/All"

#include "opencv2/opencv.hpp"

#define STB_IMAGE_IMPLEMENTATION

#include "stb/stb_image.h"

int main(int argc, char *argv[])
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    std::string bunny_path   = std::string(PBD_MODEL_DIR) + "bun_zipper.ply";
    std::string mitsuba_path = std::string(PBD_TEXTURE_DIR) + "mitsuba.png";

    Discregrid::TriangleMesh mesh(bunny_path);

    using namespace cv;
    Mat img = imread(mitsuba_path, IMREAD_COLOR);
    imshow("Mitsuba", img);
    int k = waitKey(0);

    int           width, height, nrChannels;
    unsigned char *data      = stbi_load(mitsuba_path.c_str(), &width, &height, &nrChannels, 0);


    igl::opengl::glfw::Viewer viewer;
    viewer.load_mesh_from_file(bunny_path);
    viewer.launch();
}
