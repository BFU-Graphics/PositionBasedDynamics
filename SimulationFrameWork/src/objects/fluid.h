#ifndef POSITIONBASEDDYNAMICS_FLUID_H
#define POSITIONBASEDDYNAMICS_FLUID_H

#include "objects.h"

namespace HINASIM
{
    class Fluid : public SimObject
    {
    public: // constructors

    public: // chained useful methods

    public: // simulation state

    protected: // disabled constructors
        Eigen::MatrixXd x_; // n x 3
        Eigen::MatrixXd v_; // n x 3
        Eigen::MatrixXd a_; // n x 3

    protected: // overrides

        void init_physics_states() override;

        void update_rendering_info() override;

        void update_physics_info() override;
    };
}

#endif //POSITIONBASEDDYNAMICS_FLUID_H
