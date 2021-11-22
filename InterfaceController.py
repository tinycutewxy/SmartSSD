from m5.params import *
from m5.SimObject import SimObject
from m5.objects import DRAMCacheControl
from m5.objects.AbstractMemory import *
from m5.proxy import *

#An interface controller to handle the interface control of the smartssd
# Convert the incoming request to the emb_table indices and check whether it has finished
class InterfaceController(AbstractMemory):
    type = 'InterfaceController'
    cxx_header = "SmartSSD/if_ctrl.hh"
    cxx_class = 'gem5::InterfaceController'

    bus_port = ResponsePort("Port for incoming requests from the system bus")
    dram_port = RequestPort("Port for connecting the DRAM")

    batch_size = Param.UInt32("Batch Size")

    table_ids = VectorParam.UInt32("Indices for the assigned tables. It could be ignored in the kernel since too small")
    dcache_ctrl = Param.DRAMCacheControl("DRAMCacheController this kernel is attached to.")

