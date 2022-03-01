#ifndef POSITIONBASEDDYNAMICS_FIELDS3_H
#define POSITIONBASEDDYNAMICS_FIELDS3_H

#include <Eigen/Dense>

namespace HINASIM::UTILS
{
    class Field3
    {
    public:
        Field3() = default;

        ~Field3() = default;
    };

    class ScalarField3 : public Field3
    {
    public:
        ScalarField3() = default;

        virtual ~ScalarField3() = default;

        [[nodiscard]] virtual double sample(const Eigen::Vector3d &x) const = 0;

        [[nodiscard]] virtual Eigen::Vector3d gradient(const Eigen::Vector3d &x) const;

        [[nodiscard]] virtual double laplacian(const Eigen::Vector3d &x) const;

        [[nodiscard]] virtual std::function<double(const Eigen::Vector3d &)> sampler() const;
    };

    class VectorField3 : public Field3 {
    public:
        VectorField3() = default;

        virtual ~VectorField3() = default;

        [[nodiscard]] virtual Eigen::Vector3d sample(const Eigen::Vector3d& x) const = 0;

        [[nodiscard]] virtual double divergence(const Eigen::Vector3d& x) const;

        [[nodiscard]] virtual Eigen::Vector3d curl(const Eigen::Vector3d& x) const;

        [[nodiscard]] virtual std::function<Eigen::Vector3d(const Eigen::Vector3d&)> sampler() const;
    };
}

#endif //POSITIONBASEDDYNAMICS_FIELDS3_H
