/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_OBJECTS_H
#define POSITIONBASEDDYNAMICS_OBJECTS_H

#include "RenderingFrameWork/api/draggable.h"
#include "RenderingFrameWork/api/renderable.h"

#include "src/constraints/constraints.h"

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
    protected:
        explicit SimObject(Eigen::Vector3d position = {0, 0, 0}, const Eigen::Quaterniond &rotation = {0, 0, 0, 1});

        explicit SimObject(const std::string &path, Eigen::Vector3d position = {0, 0, 0}, const Eigen::Quaterniond &rotation = {0, 0, 0, 1});

        explicit SimObject(const std::function<void(Eigen::MatrixXd &V, Eigen::MatrixXi &F)> &custom_init_geometry, Eigen::Vector3d position = {0, 0, 0}, const Eigen::Quaterniond &rotation = {0, 0, 0, 1});

    public:
        Eigen::Vector3d position_;
        Eigen::Quaterniond rotation_;
        SimObjectType TYPE_;

    public:
        void init_rendering_info();

        virtual void update_rendering_info() = 0;

        virtual void update_physics_info() = 0;

    public:
        virtual void init_physics_states() = 0;
    };
}

#endif //POSITIONBASEDDYNAMICS_OBJECTS_H
