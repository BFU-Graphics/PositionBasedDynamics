#ifndef POSITIONBASEDDYNAMICS_COLLISION_OBJECTS_H
#define POSITIONBASEDDYNAMICS_COLLISION_OBJECTS_H

#include "RenderingFrameWork/api/renderable.h"
#include <Eigen/Dense>

namespace HINASIM
{
    enum class CollisionObjectType
    {
        NoType,
        Sphere,
        Box,
    };

    class CollisionObject : public HINAVIEWER::RENDERABLE::Renderable
    {
    protected:
        explicit CollisionObject(const std::string &path, Eigen::Vector3d position) : position_(std::move(position))
        {
            TYPE_ = CollisionObjectType::NoType;
            load_model_from_path(path);
            V_.rowwise() += position_.transpose();
            color_ = {134.0 / 255.9, 206.0 / 255.9, 203.0 / 255.9}; // miku color ~
        }

        virtual ~CollisionObject() = default; // to support dynamics_cast<>()

    public:
        CollisionObjectType TYPE_;
        Eigen::Vector3d position_;

    public:
    };

    class SphereCollider : public CollisionObject
    {
    public:
        explicit SphereCollider(Eigen::Vector3d position, double radius) : CollisionObject(PBD_MODEL_DIR + std::string("sphere.obj"), std::move(position)), radius_(radius)
        {
            TYPE_ = CollisionObjectType::Sphere;
            for (int i = 0; i < V_.rows(); ++i)
            {
                V_.row(i) = (radius * V_.row(i)).eval();
            }
        }

    public:
        double radius_;
    };

    class BoxCollider : public CollisionObject
    {
    public:
        explicit BoxCollider(Eigen::Vector3d position, Eigen::Vector3d half_extent) : CollisionObject(PBD_MODEL_DIR + std::string("cube.obj"), std::move(position)), half_extent_(std::move(half_extent))
        {
            TYPE_ = CollisionObjectType::Box;
            // TODO:
        }

    public:
        Eigen::Vector3d half_extent_; // [half_length, half_width, half_height]
    };
}

#endif //POSITIONBASEDDYNAMICS_COLLISION_OBJECTS_H
