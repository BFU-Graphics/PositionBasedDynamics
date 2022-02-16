/**
 * @author YC XIANG.
 * @date 02/16, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_RENDERABLE_H
#define POSITIONBASEDDYNAMICS_RENDERABLE_H

#include "../eigen_types.h"

#include <string>

namespace HINAVIEWER::RENDERABLE
{
    class Renderable
    {
    public:
        virtual void init_geometry(const std::string &path) = 0;

    public:
        bool is_recorded_ = false;
        int ID_ = -1; // invalid unless recorded

    public:
        Eigen::MatrixXd V_;
        Eigen::MatrixXi F_;
        Eigen::MatrixXi E_;
    };
}


#endif //POSITIONBASEDDYNAMICS_RENDERABLE_H
