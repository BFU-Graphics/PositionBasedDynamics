/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#include "pbd_sim.h"

void HINASIM::PBDSim::add_object(HINASIM::SimObject *object)
{
    objects_.emplace_back(object);
}
