/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_OBJECT_H
#define POSITIONBASEDDYNAMICS_OBJECT_H

#include "RenderingFrameWork/src/renderable.h"

namespace HINASIM
{
    class Object : public HINAVIEWER::RENDERABLE::Renderable
    {
    public:
        void init_geometry(const std::string &path) override;

    public:
        void (*custom_init_geometry)() = nullptr;
    };
}

#endif //POSITIONBASEDDYNAMICS_OBJECT_H
