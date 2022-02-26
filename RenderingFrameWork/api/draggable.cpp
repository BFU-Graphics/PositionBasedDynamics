#include "draggable.h"

void HINAVIEWER::MOUSE_DRAGGABLE::Draggable::update_mouse_drag_force()
{
    for (auto &v_index: HINAVIEWER::MOUSE_CALLBACK::picked_vertices())
    {
        double stiffness = HINAVIEWER::MOUSE_CALLBACK::is_mouse_dragging() ? 1e4 : 0;
        Eigen::Vector3d mouse_drag_delta = HINAVIEWER::MOUSE_CALLBACK::mouse_drag_world() + Eigen::Vector3d::Constant(1e-6);
        Eigen::Vector3d mouse_drag_force = stiffness * mouse_drag_delta;

        mouse_drag_force_.row(v_index) += mouse_drag_force.transpose();
    }

    if (HINAVIEWER::MOUSE_CALLBACK::picked_vertices().empty())
        mouse_drag_force_.setZero();
}
