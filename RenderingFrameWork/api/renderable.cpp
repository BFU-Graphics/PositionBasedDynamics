#include "renderable.h"

#include <igl/readOBJ.h>
#include <igl/readPLY.h>

void HINAVIEWER::RENDERABLE::Renderable::load_model_from_path(const std::string &path)
{
    std::string model_name = path.substr(path.find_last_of('/') + 1);
    std::string model_type = path.substr(path.find_last_of('.') + 1);
    if (model_type.empty())
        return;
    model_type = model_type.substr(0, 3);
    if (model_type == "obj")
        igl::readOBJ(path, V_, F_);
    else if (model_type == "ply")
        igl::readPLY(path, V_, F_);
    else
        return;

    build_neighbors(); // Edges info would be auto build ourselves

    std::clog << "Model Name: " << model_name << "     Vertex Sum: " << V_.rows() << "     Face Sum: " << F_.rows() << std::endl;
}

void HINAVIEWER::RENDERABLE::Renderable::build_neighbors()
{
    if (V_.rows() == 0)
        return;

    int vertices_per_face = 3; // triangle mesh only

    VE_.clear();
    VF_.clear();
    EVF_.clear();
    FE_.clear();

    VE_.resize(V_.rows());
    VF_.resize(V_.rows());
    FE_.resize(F_.rows());
    EVF_.reserve(E_.rows());

    for (int i = 0; i < F_.rows(); ++i)
    {
        FE_[i].resize(vertices_per_face);
        for (int j = 0; j < vertices_per_face; ++j)
        {
            const unsigned int v_index = F_.row(i)(j);

            bool found = false;
            for (auto &face_index: VF_[v_index])
            {
                if (face_index == i)
                {
                    found = true;
                    break;
                }
            }

            if (!found)
                VF_[v_index].emplace_back(i);

            const unsigned int ev1 = F_.row(i)(j);
            const unsigned int ev2 = F_.row(i)((j + 1) % 3);
            unsigned int edge = 0xffffffff;
            for (auto &edge_index: VE_[ev1])
                if ((EVF_[edge_index][0] == ev1 || EVF_[edge_index][0] == ev2) && (EVF_[edge_index][1] == ev1 || EVF_[edge_index][1] == ev2))
                    edge = edge_index;
            if (edge == 0xffffffff)
            {
                EVF_.emplace_back(std::vector<unsigned int>({ev1, ev2, static_cast<unsigned int>(i), 0xffffffff}));
                edge = EVF_.size() - 1;
                VE_[ev1].emplace_back(edge);
                VE_[ev2].emplace_back(edge);
            } else
                EVF_[edge][3] = i;

            FE_[i][j] = edge;
        }
    }

    E_.resize(EVF_.size(), 2);
    for (int i = 0; i < EVF_.size(); ++i)
        E_.row(i) = Eigen::RowVector2i(EVF_[i][0], EVF_[i][1]);

    for (auto &evf: EVF_)
    {
        if (evf[3] == 0xffffffff)
        {
            is_closed_ = false;
            return;
        }
    }
    is_closed_ = true;
}