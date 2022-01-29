/**
 * @author YC XIANG.
 * @date 01/29, 2022
 */

#include "viewer.h"

#define STB_IMAGE_IMPLEMENTATION

#include "stb/stb_image.h"

using namespace PBD;

void Viewer::launch()
{
    viewer.launch();
}

void Viewer::set_texture(int data_id, const std::string &path)
{
    auto &data = viewer.data(data_id);

    int width, height, nrChannels;
    unsigned char *image = stbi_load(path.c_str(), &width, &height, &nrChannels, 0);


    if (nrChannels == 4)
    {
        Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R, G, B, A;
        R.resize(width, height);
        G.resize(width, height);
        B.resize(width, height);
        A.resize(width, height);

        for (int i = 0; i < height; ++i)
        {
            for (int j = 0; j < width; ++j)
            {
//                std::cout << "[ ";
//                for (int k = 0; k < nrChannels; ++k)
//                {
//                    std::cout << (int) (image[i * width + j + k]) << " ";
//                }
//                std::cout << "]" << std::endl;

                R(i, j) = image[i * width + j + 0];
                G(i, j) = image[i * width + j + 1];
                B(i, j) = image[i * width + j + 2];
                A(i, j) = image[i * width + j + 3];
            }
        }
        data.set_texture(R, G, B, A);
    }
}
