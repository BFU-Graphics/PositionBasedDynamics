/**
 * @author YC XIANG.
 * @date 02/10, 2022
 */

#include "inspector.h"

void pbd_inspector::ScalarTimeValueInspector::plot(const char *label)
{

}

void pbd_inspector::Inspector::track(pbd_inspector::Trackable &trackable, int index)
{
    trackable.track(index);
}
