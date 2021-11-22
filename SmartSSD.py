#This is a cache control unit for prefetching n-iters look-ahead cache
#Base on the simple_memobj from the tutorial

from m5.params import *
from m5.SimObject import SimObject

from m5.proxy import *

class SmartSSD(SimObject):
    type = 'SmartSSD'
    cxx_header = "SmartSSD/smartssd.hh"
    cxx_class = 'gem5::SmartSSD'

    if_ctrl = Param.InterfaceController("Interface Controller")
    system = Param.System(Parent.any, "System this device belongs to")