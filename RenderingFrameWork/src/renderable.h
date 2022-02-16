/**
 * @author YC XIANG.
 * @date 02/16, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_RENDERABLE_H
#define POSITIONBASEDDYNAMICS_RENDERABLE_H

#include "../eigen_types.h"

namespace HINAVIEWER::RENDERABLE
{
    class Renderable
    {
    public:
        virtual void init_geometry() = 0;

    public:
        bool is_recorded_ = false;
        int ID_ = -1; // invalid unless recorded

    public:
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
    };
}


#endif //POSITIONBASEDDYNAMICS_RENDERABLE_H
