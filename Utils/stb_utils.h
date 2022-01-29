/**
 * @author YC XIANG.
 * @date 01/29, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_STB_UTILS_H
#define POSITIONBASEDDYNAMICS_STB_UTILS_H

#include "Eigen/Eigen"

#include <tuple>

typedef Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> MY_TEXTURE_TYPE;

inline static std::tuple<MY_TEXTURE_TYPE, MY_TEXTURE_TYPE, MY_TEXTURE_TYPE, MY_TEXTURE_TYPE>
stb_data_to_rgba()
{
}

#endif //POSITIONBASEDDYNAMICS_STB_UTILS_H
