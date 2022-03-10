#ifndef POSITIONBASEDDYNAMICS_DISTANCE_FIELD_COLLISION_DETECTION_H
#define POSITIONBASEDDYNAMICS_DISTANCE_FIELD_COLLISION_DETECTION_H

#include "collision_detection.h"
#include "bounding_sphere_hierarchy.h"

namespace HINASIM
{
    class DistanceFieldCollisionObject : public CollisionObject
    {
    public:
        explicit DistanceFieldCollisionObject(class SimObject *object);

        ~DistanceFieldCollisionObject() override = default;

    public:

        PointCloudBSH bvh_;

        bool invertSDF_ = false;

    public:

        virtual double distance(const Eigen::Vector3d &x, double tolerance) = 0;
    };

    class DistanceFieldCollisionSphere : public DistanceFieldCollisionObject
    {
    public:
        explicit DistanceFieldCollisionSphere(class SimObject *o);

        ~DistanceFieldCollisionSphere() override = default;

    public:
        double radius_;

    public:
        double distance(const Eigen::Vector3d &x, double tolerance) override;
    };

    class DistanceFieldCollisionDetection : public CollisionDetection
    {
    public:
        DistanceFieldCollisionDetection();

        ~DistanceFieldCollisionDetection() override = default;

    public:
        struct ContactData
        {

        };

        void collision_detection() override;

        void collision_detection_rigid_rigid(class RigidBody *rb1, DistanceFieldCollisionObject *co1, class RigidBody *rb2, DistanceFieldCollisionObject *co2, std::vector<std::vector<ContactData> > &contacts_mt, double restitution,
                                             double friction);

        void add_collider_sphere(class RigidBody *sphere, bool invert_sdf = false);
    };
}
#endif //POSITIONBASEDDYNAMICS_DISTANCE_FIELD_COLLISION_DETECTION_H
