#ifndef POSITIONBASEDDYNAMICS_BOUNDING_SPHERE_H
#define POSITIONBASEDDYNAMICS_BOUNDING_SPHERE_H

#include <Eigen/Dense>
#include <vector>
#include <utility>

namespace HINASIM
{
    class BoundingSphere
    {
        /**
         * \brief Computes smallest enclosing spheres of point sets using Welzl's algorithm
         * \Author: Tassilo Kugelstadt
         */
    public:
        /**
         * \brief default constructor sets the center and radius to zero.
         */
        BoundingSphere() : x_(Eigen::Vector3d::Zero()), r_(0.0)
        {}

        /**
         * \brief constructor which sets the center and radius
         *
         * \param x	3d coordiantes of the center point
         * \param r radius of the sphere
         */
        BoundingSphere(Eigen::Vector3d x, const double r) : x_(std::move(x)), r_(r)
        {}

        /**
         * \brief	constructs a sphere for one point (with radius 0)
         *
         * \param a	3d coordinates of point a
         */
        explicit BoundingSphere(const Eigen::Vector3d &a)
        {
            x_ = a;
            r_ = 0.0;
        }

        /**
         * \brief	constructs the smallest enclosing sphere for two points
         *
         * \param a 3d coordinates of point a
         * \param b 3d coordinates of point b
         */
        BoundingSphere(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
        {
            const Eigen::Vector3d ba = b - a;

            x_ = (a + b) * static_cast<double >(0.5);
            r_ = static_cast<double >(0.5) * ba.norm();
        }

        /**
         * \brief	constructs the smallest enclosing sphere for three points
         *
         * \param a 3d coordinates of point a
         * \param b 3d coordinates of point b
         * \param c 3d coordinates of point c
         */
        BoundingSphere(const Eigen::Vector3d &a, const Eigen::Vector3d &b, const Eigen::Vector3d &c)
        {
            const Eigen::Vector3d ba = b - a;
            const Eigen::Vector3d ca = c - a;
            const Eigen::Vector3d baxca = ba.cross(ca);
            Eigen::Vector3d r;
            Eigen::Matrix3d T;
            T << ba[0], ba[1], ba[2],
                    ca[0], ca[1], ca[2],
                    baxca[0], baxca[1], baxca[2];

            r[0] = static_cast<double>(0.5) * ba.squaredNorm();
            r[1] = static_cast<double>(0.5) * ca.squaredNorm();
            r[2] = static_cast<double>(0.0);

            x_ = T.inverse() * r;
            r_ = x_.norm();
            x_ += a;
        }

        BoundingSphere(const Eigen::Vector3d &a, const Eigen::Vector3d &b, const Eigen::Vector3d &c, const Eigen::Vector3d &d)
        {
            const Eigen::Vector3d ba = b - a;
            const Eigen::Vector3d ca = c - a;
            const Eigen::Vector3d da = d - a;
            Eigen::Vector3d r;
            Eigen::Matrix3d T;
            T << ba[0], ba[1], ba[2],
                    ca[0], ca[1], ca[2],
                    da[0], da[1], da[2];

            r[0] = static_cast<double>(0.5) * ba.squaredNorm();
            r[1] = static_cast<double>(0.5) * ca.squaredNorm();
            r[2] = static_cast<double>(0.5) * da.squaredNorm();
            x_ = T.inverse() * r;
            r_ = x_.norm();
            x_ += a;
        }

        explicit BoundingSphere(const Eigen::MatrixXd &vertices_set)
        {
            r_ = 0;
            x_.setZero();
            setPoints(vertices_set);
        }

        void setPoints(const Eigen::MatrixXd &vertices_set)
        {
            std::vector<Eigen::Vector3d> v;
            v.resize(vertices_set.rows());

            // TODO: to avoid copy
            for (int i = 0; i < vertices_set.rows(); ++i)
                v[i] = vertices_set.row(i).transpose();

            std::sort(v.begin(), v.end(), [](const Eigen::Vector3d &a, const Eigen::Vector3d &b)
            {
                if (a[0] < b[0]) return true;
                if (a[0] > b[0]) return false;
                if (a[1] < b[1]) return true;
                if (a[1] > b[1]) return false;
                return (a[2] < b[2]);
            });
            v.erase(std::unique(v.begin(), v.end(), [](Eigen::Vector3d &a, Eigen::Vector3d &b)
            { return a.isApprox(b); }), v.end());

            Eigen::Vector3d d;
            const int n = int(v.size());

            //generate random permutation of the points and permute the points by epsilon to avoid corner cases
            const double epsilon = 1.0e-6;
            for (int i = n - 1; i > 0; i--)
            {
                const Eigen::Vector3d epsilon_vec = epsilon * Eigen::Vector3d::Random();
                const int j = static_cast<int>(floor(i * double(rand()) / RAND_MAX));
                d = v[i] + epsilon_vec;
                v[i] = v[j] - epsilon_vec;
                v[j] = d;
            }

            BoundingSphere S = BoundingSphere(v[0], v[1]);

            for (int i = 2; i < n; i++)
            {
                //SES0
                d = v[i] - S.x_;
                if (d.squaredNorm() > S.r_ * S.r_)
                    S = ses1(i, v, v[i]);
            }

            x_ = S.x_;
            r_ = S.r_ + static_cast<double >(epsilon);
        }

        [[nodiscard]] bool overlaps(BoundingSphere const &other) const
        {
            double rr = r_ + other.r_;
            return (x_ - other.x_).squaredNorm() < rr * rr;
        }

        [[nodiscard]] bool contains(BoundingSphere const &other) const
        {
            double rr = r_ - other.r_;
            return (x_ - other.x_).squaredNorm() < rr * rr;
        }

        [[nodiscard]] bool contains(Eigen::Vector3d const &point) const
        {
            return (x_ - point).squaredNorm() < r_ * r_;
        }

    private:
        /**
		 * \brief		constructs the smallest enclosing sphere for n points with the points q1, q2, and q3 on the surface of the sphere
		 *
		 * \param n		number of points
		 * \param p		vertices of the points
		 * \param q1	3d coordinates of a point on the surface
		 * \param q2	3d coordinates of a second point on the surface
		 * \param q3	3d coordinates of a third point on the surface
		 * \return		smallest enclosing sphere
		 */
        BoundingSphere ses3(int n, std::vector<Eigen::Vector3d> &p, Eigen::Vector3d &q1, Eigen::Vector3d &q2, Eigen::Vector3d &q3)
        {
            BoundingSphere S(q1, q2, q3);

            for (int i = 0; i < n; i++)
            {
                Eigen::Vector3d d = p[i] - S.x_;
                if (d.squaredNorm() > S.r_ * S.r_)
                    S = BoundingSphere(q1, q2, q3, p[i]);
            }
            return S;
        }

        /**
         * \brief		constructs the smallest enclosing sphere for n points with the points q1 and q2 on the surface of the sphere
         *
         * \param n		number of points
         * \param p		vertices of the points
         * \param q1	3d coordinates of a point on the surface
         * \param q2	3d coordinates of a second point on the surface
         * \return		smallest enclosing sphere
         */
        BoundingSphere ses2(int n, std::vector<Eigen::Vector3d> &p, Eigen::Vector3d &q1, Eigen::Vector3d &q2)
        {
            BoundingSphere S(q1, q2);

            for (int i = 0; i < n; i++)
            {
                Eigen::Vector3d d = p[i] - S.x_;
                if (d.squaredNorm() > S.r_ * S.r_)
                    S = ses3(i, p, q1, q2, p[i]);
            }
            return S;
        }

        /**
         * \brief		constructs the smallest enclosing sphere for n points with the point q1 on the surface of the sphere
         *
         * \param n		number of points
         * \param p		vertices of the points
         * \param q1	3d coordinates of a point on the surface
         * \return		smallest enclosing sphere
         */
        BoundingSphere ses1(int n, std::vector<Eigen::Vector3d> &p, Eigen::Vector3d &q1)
        {
            BoundingSphere S(p[0], q1);

            for (int i = 1; i < n; i++)
            {
                Eigen::Vector3d d = p[i] - S.x_;
                if (d.squaredNorm() > S.r_ * S.r_)
                    S = ses2(i, p, q1, p[i]);
            }
            return S;
        }

    public:
        Eigen::Vector3d x_;
        double r_;
    };
}

#endif //POSITIONBASEDDYNAMICS_BOUNDING_SPHERE_H
