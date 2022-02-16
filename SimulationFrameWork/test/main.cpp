/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#include "RenderingFrameWork/pbd_viewer.h"

#include "../src/object.h"
#include "../src/constraints.h"

int main()
{
    HINAVIEWER::PBDViewer viewer;

    HINASIM::Object obj;
    obj.init_geometry(PBD_MODEL_DIR + std::string("cube.obj"));
    viewer.record(&obj);

    viewer.launch_rendering();
    return 0;
}
