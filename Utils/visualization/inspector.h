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
    class Trackable
    {
    public:
        virtual std::pair<int, std::deque<std::vector<double> > > &track(int index) = 0;
    };

    class Inspector
    {
    public:
        void track(Trackable &trackable, int index);
        virtual void plot(const char *label) = 0;
    };

    class ScalarTimeValueInspector : public Inspector
    {
    public:
        void plot(const char *label) override;

    public:
        
    };
}


#endif //POSITIONBASEDDYNAMICS_INSPECTOR_H
