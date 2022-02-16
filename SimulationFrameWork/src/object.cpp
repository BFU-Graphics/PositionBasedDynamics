/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#include "object.h"

#include <igl/readOBJ.h>
#include <igl/readPLY.h>

void HINASIM::Object::init_geometry(const std::string &path)
{
    if (custom_init_geometry != nullptr)
    {
        custom_init_geometry();
        return;
    }

    igl::readOBJ(path, V_, F_);
}