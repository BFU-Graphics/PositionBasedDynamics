#ifndef POSITIONBASEDDYNAMICS_OBJECTS_H
#define POSITIONBASEDDYNAMICS_OBJECTS_H

#include "RenderingFrameWork/api/draggable.h"
#include "RenderingFrameWork/api/renderable.h"

namespace HINASIM
{
    enum class SimObjectType
    {
        NoType,
        Deformable,
        RigidBody,
        Fluid,
    };


    class SimObject : public HINAVIEWER::RENDERABLE::Renderable, public HINAVIEWER::MOUSE_DRAGGABLE::Draggable
    {
    public:
        explicit SimObject(Eigen::Vector3d position = {0, 0, 0}, Eigen::Vector3d rotation = {0, 0, 0}, Eigen::Vector3d scale = {1, 1, 1});
        explicit SimObject(const std::string &path, Eigen::Vector3d position = {0, 0, 0}, Eigen::Vector3d rotation = {0, 0, 0}, Eigen::Vector3d scale = {1, 1, 1});
        explicit SimObject(const std::function<void(Eigen::MatrixXd &V, Eigen::MatrixXi &F)> &custom_init_geometry, Eigen::Vector3d position = {0, 0, 0}, Eigen::Vector3d rotation = {0, 0, 0}, Eigen::Vector3d scale = {1, 1, 1});
        virtual ~SimObject() = default;

    public:
        Eigen::Vector3d position_;
        Eigen::Vector3d rotation_;
        Eigen::Vector3d scale_;
        SimObjectType TYPE_;

    public:
        virtual void init_rendering_info() final;
        virtual void init_physics_states() = 0;
        virtual void update_rendering_info() = 0;
        virtual void update_physics_info() = 0;
    };
}

#endif //POSITIONBASEDDYNAMICS_OBJECTS_H
