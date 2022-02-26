#ifndef POSITIONBASEDDYNAMICS_DRAGGABLE_H
#define POSITIONBASEDDYNAMICS_DRAGGABLE_H

#include "RenderingFrameWork/src/mouse_callback.h"

namespace HINAVIEWER::MOUSE_DRAGGABLE
{
    class Draggable
    {
    public:
        virtual void update_mouse_drag_force() final; // mark as final now

    public:
        Eigen::MatrixXd mouse_drag_force_;
    };
}


#endif //POSITIONBASEDDYNAMICS_DRAGGABLE_H
