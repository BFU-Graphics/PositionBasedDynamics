#ifndef POSITIONBASEDDYNAMICS_COLLIDERS_H
#define POSITIONBASEDDYNAMICS_COLLIDERS_H

#include "aabb.h"
#include "bounding_sphere_hierarchy.h"

namespace HINASIM
{
    class Collider
    {
    public:
        explicit Collider(class SimObject *o);
        virtual ~Collider();

    public:
        virtual Collider &update_aabb();

    public:
        AABB *aabb_;

    public:
        class SimObject *object_; // the object Collider was attached to;
    };


    class DistanceFieldCollider : public Collider
    {
    public:
        explicit DistanceFieldCollider(class SimObject *o, bool inside_collision);
        ~DistanceFieldCollider() override;

    public:
        virtual DistanceFieldCollider &update_bvh();

    public:
//        virtual bool collision_test(const Eigen::Vector3d &x, double tolerance, Eigen::Vector3d &contact_point, Eigen::Vector3d &normal, double distance, double max_distance);
        virtual double distance(const Eigen::Vector3d &x, double tolerance) = 0;

    protected:
        PointCloudBSH *bvh_;
        bool inside_collision_;
    };


    class SphereColliderDF final : public DistanceFieldCollider
    {
    public:
        explicit SphereColliderDF(class SimObject *o, bool inside_collision = false);

    public:
        double distance(const Eigen::Vector3d &x, double tolerance) final;

    private:
        double radius_;
    };
}

#endif //POSITIONBASEDDYNAMICS_COLLIDERS_H
