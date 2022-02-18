/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_OBJECTS_H
#define POSITIONBASEDDYNAMICS_OBJECTS_H

#include "RenderingFrameWork/src/renderable.h"

#include "SimulationFrameWork/src/constraints.h"

namespace HINASIM
{
    class SimObject : public HINAVIEWER::RENDERABLE::Renderable
    {
    public:
        void init_geometry(const std::string &path) override;

        virtual void init_physical_state() = 0;

        virtual void update_geometry_info() = 0;

    public:
        SimObject &add_constraint(Constraint *constraint);

        SimObject &set_inv_mass(double unified_inv_mass);

        SimObject &set_inv_mass(int index, double inv_mass);

    public:
        /// once [custom_init_geometry] is not nullptr, this would replace default [init_geometry]
        std::function<void(Eigen::MatrixXd &V_, Eigen::MatrixXi &F_)> custom_init_geometry;

    public:
        Eigen::MatrixXd q_; // n x 3
        Eigen::MatrixXd qdot_; // n x 3
        Eigen::MatrixXd p_; // n x 3
        Eigen::VectorXd inv_mass_; // n x 1

        std::vector<Constraint *> inner_constraints_;
    };

    class DeformableObject : public SimObject
    {
    public:
        void init_physical_state() override;

        void update_geometry_info() override;
    };

    class Cloth : public DeformableObject
    {
    public:
        Cloth(int rows, int cols, int width, int height);

        void init_geometry();

    public:
        int rows_, cols_;
        int width_, height_;
    };
}

#endif //POSITIONBASEDDYNAMICS_OBJECTS_H
