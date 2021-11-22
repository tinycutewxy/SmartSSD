#ifndef __Smart_SSD_HH__
#define __Smart_SSD_HH__

#include "params/SmartSSD.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"
#include "SmartSSD/if_ctrl.hh"


namespace gem5
{

/**
 * A very simple memory object. Current implementation doesn't even cache
 * anything it just forwards requests and responses.
 * This memobj is fully blocking (not non-blocking). Only a single request can
 * be outstanding at a time.
 */
class SmartSSD : public SimObject
{
    InterfaceController* if_ctrl;

  public:

    SmartSSD(const SmartSSDParams &params) : 
            SimObject(params),
            if_ctrl(params.if_ctrl)
    {
        if_ctrl->init(params.system);
    }

};

} // namespace gem5

#endif // __LEARNING_GEM5_PART2_SIMPLE_MEMOBJ_HH__
