/**
 * @author YC XIANG.
 * @date 02/15, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_INSPECTOR_H
#define POSITIONBASEDDYNAMICS_INSPECTOR_H

#include <vector>
#include <deque>
#include <string>

namespace HINAVIEWER::INSPECTOR
{
    class Timeable
    {
    public:
        static double simulation_time_;
        static double physics_rate;
        static long long physics_runtime;
    };

    class Trackable : public Timeable
    {
    public:
        std::deque<std::vector<double> > tracked_state_; // [index, {time, ...}]
        std::vector<std::string> tracked_names_;
        std::vector<double> tracked_max;
        std::vector<double> tracked_min;

    public:
        void record(const std::vector<double> &states, const std::vector<std::string> &names);

        size_t max_cache_ = 10000;
    };

    class Inspector
    {
    public:
        Inspector *track(Trackable *trackable, int index);

        virtual void plot(float start_pos_x, float start_pos_y, float width, float height);

        Trackable *trackable_ = nullptr;
        int index_ = -1;
    };

    class ScalarTimeValueInspector : public Inspector
    {
    public:
        void plot(float start_pos_x, float start_pos_y, float width, float height) override;

    public:

    };
}

#endif //POSITIONBASEDDYNAMICS_INSPECTOR_H
