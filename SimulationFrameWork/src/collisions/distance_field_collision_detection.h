#ifndef POSITIONBASEDDYNAMICS_DISTANCE_FIELD_COLLISION_DETECTION_H
#define POSITIONBASEDDYNAMICS_DISTANCE_FIELD_COLLISION_DETECTION_H

#include "collision_detection.h"

namespace HINASIM
{
    class DistanceFieldCollisionDetection : public CollisionDetection
    {
    public:
        DistanceFieldCollisionDetection() = default;
        ~DistanceFieldCollisionDetection() override = default;

    public:
        void collision_detection() override;

    protected:

    };
}
#endif //POSITIONBASEDDYNAMICS_DISTANCE_FIELD_COLLISION_DETECTION_H
