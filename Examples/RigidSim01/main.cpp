#include "RenderingFrameWork/pbd_viewer.h"
#include "SimulationFrameWork/pbd_sim.h"
#include "SimulationFrameWork/src/collisions/distance_field_collision_detection.h"

int main()
{
    HINAVIEWER::PBDViewer pbd_viewer;
    HINASIM::PBDSim pbd_sim;
    HINASIM::DistanceFieldCollisionDetection cd;

    HINASIM::RigidBody sphere1(PBD_MODEL_DIR + std::string("sphere.obj"), {0, 0, 0});
    HINASIM::RigidBody sphere2(PBD_MODEL_DIR + std::string("sphere.obj"), {0.9, 0.9, 0.9});

    pbd_sim
            .set_collision_engine(&cd)
            .add_object(&sphere1)
            .add_object(&sphere2);

    cd.add_collider_sphere(&sphere1);
    cd.add_collider_sphere(&sphere2);
    cd.collision_detection();

    return 0;
}
