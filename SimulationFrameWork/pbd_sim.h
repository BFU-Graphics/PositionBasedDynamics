/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_PBD_SIM_H
#define POSITIONBASEDDYNAMICS_PBD_SIM_H

#include "src/objects.h"
#include "src/constraints.h"

#include <vector>

namespace HINASIM
{
    class PBDSim
    {
    public:
        void add_object(SimObject *object);

    protected:
        typedef Constraint Joint;
        std::vector<SimObject *> objects_;
        std::vector<Joint *> joints_;

    protected: // Env

    };
}

#endif //POSITIONBASEDDYNAMICS_PBD_SIM_H
