#ifndef POSITIONBASEDDYNAMICS_DEFORMABLE_H
#define POSITIONBASEDDYNAMICS_DEFORMABLE_H

#include "objects.h"

namespace HINASIM
{
    class DeformableObject : public SimObject
    {
    public:
        explicit DeformableObject(const std::string &path, Eigen::Vector3d position = {0, 0, 0}, const Eigen::Quaterniond &rotation = {0, 0, 0, 1});

        explicit DeformableObject(const std::function<void(Eigen::MatrixXd &V, Eigen::MatrixXi &F)> &custom_init_geometry, Eigen::Vector3d position = {0, 0, 0}, const Eigen::Quaterniond &rotation = {0, 0, 0, 1});

    public:
        DeformableObject &add_constraint(Constraint *constraint);

        DeformableObject &set_inv_mass(double unified_inv_mass);

        DeformableObject &set_inv_mass(int index, double inv_mass);

    public: // simulation state
        Eigen::MatrixXd x_; // n x 3
        Eigen::MatrixXd v_; // n x 3
        Eigen::MatrixXd a_; // n x 3
        Eigen::MatrixXd p_; // n x 3
        Eigen::VectorXd inv_mass_; // n x 1

        std::vector<Constraint *> inner_constraints_;

    protected:
        explicit DeformableObject(Eigen::Vector3d position = {0, 0, 0}, const Eigen::Quaterniond &rotation = {0, 0, 0, 1});

    private:
        void init_physics_states() override;

        void update_rendering_info() override;

        void update_mouse_drag_force() override;
    };

    class Cloth : public DeformableObject
    {
    public:
        Cloth(int rows, int cols, int width, int height, Eigen::Vector3d position = {0, 0, 0}, const Eigen::Quaterniond &rotation = {0, 0, 0, 1});

    public:
        int rows_, cols_;
        int width_, height_;
    };
}

#endif //POSITIONBASEDDYNAMICS_DEFORMABLE_H