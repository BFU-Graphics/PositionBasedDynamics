/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_PBD_SIM_H
#define POSITIONBASEDDYNAMICS_PBD_SIM_H

#include "src/constraints.h"


#include <vector>

namespace HINASIM
{
    class PBDSim
    {
    public:
        PBDSim();

    public:
        void pbd_init();

        void pbd_loop();

        void pbd_end();

    protected:
        virtual void project_positions();

    protected:
        std::vector<Constraint> constraints;

    protected: // Env

    };
}

#endif //POSITIONBASEDDYNAMICS_PBD_SIM_H
