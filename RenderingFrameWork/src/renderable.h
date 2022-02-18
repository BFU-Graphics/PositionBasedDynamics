/**
 * @author YC XIANG.
 * @date 02/16, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_RENDERABLE_H
#define POSITIONBASEDDYNAMICS_RENDERABLE_H

#include "../eigen_types.h"

#include <vector>
#include <string>

namespace HINAVIEWER::RENDERABLE
{
    class Renderable
    {
    public:
        virtual void init_geometry(const std::string &path) = 0;

        void build_neighbors();

    public:
        bool is_recorded_ = false;
        int ID_ = -1; // invalid unless recorded

    public:
        Eigen::MatrixXd V_; // Vertices - size = [#vertices x 3], row = [x, y, z]
        Eigen::MatrixXi F_; // Faces - size = [#faces x 3], row = [v1_index, v2_index, v3_index]
        Eigen::MatrixXi E_; // Edges - size = [#edges x 2], row = [v1_index, v2_index]

        std::vector<std::vector<unsigned int>> VE_; // Vertex-edges - size = [#vertices x ?], row = [edge1_index, edge2_index, ... ...]
        std::vector<std::vector<unsigned int>> VF_; // Vertex-faces - size = [#faces x ?], row = [face1_index, face2_index, ... ...]
        std::vector<std::vector<unsigned int>> FE_; // Face-edges - size = [#faces x 3], row = [edge1_index, edge2_index, edge3_index]
        std::vector<std::vector<unsigned int>> EVF_; // Edge-vertices-faces - size = [#edges x 4], row = [v1_index, v2_index, face1_index, face2_index]
        bool is_closed_;
    };
}


#endif //POSITIONBASEDDYNAMICS_RENDERABLE_H
