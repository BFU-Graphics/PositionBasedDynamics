/**
 * @author YC XIANG.
 * @date 02/10, 2022
 */

#ifndef POSITIONBASEDDYNAMICS_INSPECTOR_H
#define POSITIONBASEDDYNAMICS_INSPECTOR_H

#include <vector>
#include <deque>

namespace pbd_inspector
{
    class Timeable
    {
    public:
        static double simulation_time_;
    };

    class Trackable : public Timeable
    {
    public:
        std::deque<std::vector<double> > tracked_state_; // [index, {time, ...}]

    public:
        void record(double state);
        double max = std::numeric_limits<double>::min();
        double min = std::numeric_limits<double>::max();
        size_t max_cache_ = 10000;
    };

    class Inspector
    {
    public:
        Inspector *track(Trackable *trackable, int index);

        virtual void plot(const char *label);

        Trackable *trackable_;
        int index_;
    };

    class ScalarTimeValueInspector : public Inspector
    {
    public:
        void plot(const char *label) override;

    public:

    };
}


#endif //POSITIONBASEDDYNAMICS_INSPECTOR_H
