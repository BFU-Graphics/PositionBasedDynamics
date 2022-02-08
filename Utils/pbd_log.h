/**
 * @author YC XIANG.
 * @date 02/08, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_PBD_LOG_H
#define POSITIONBASEDDYNAMICS_PBD_LOG_H

#include "Eigen/Eigen"

#include <string>
#include <fstream>
#include <iostream>

namespace pbd_util
{
    template<typename Derived>
    void log(const Eigen::PlainObjectBase<Derived> &mat, const std::string &name)
    {
        std::cout << std::string(PBD_LOG_DIR) + name + ".txt" << std::endl;
        std::ofstream o(std::string(PBD_LOG_DIR) + name + ".txt");

        if (o.fail())
        {
            std::cout << "FAIL BIT" << std::endl;
        }

        o << "size: " << mat.rows() << " x " << mat.cols() << std::endl;

        o << mat;
        o.flush();

        o.close();
    }
}

#endif //POSITIONBASEDDYNAMICS_PBD_LOG_H
