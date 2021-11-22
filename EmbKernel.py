#This is a cache control unit for prefetching n-iters look-ahead cache
#Base on the simple_memobj from the tutorial

from m5.params import *
from m5.SimObject import SimObject
from m5.objects import DRAMCacheControl
from m5.objects.ClockedObject import ClockedObject

from m5.proxy import *

class EmbKernel(ClockedObject):
    type = 'EmbKernel'
    cxx_header = "SmartSSD/emb_kernel.hh"
    cxx_class = 'gem5::EmbKernel'

    entry_rd_port = RequestPort("Entry Reader Port, sends requests")
    grad_rd_port = RequestPort("Gradient Reader Port, sends requests")
    entry_wr_port = RequestPort("Entry Writer Port, sends requests")
    result_wr_port = RequestPort("Result Writer Port, sends requests")

    trace_file = Param.String("trace file of the DLRM input")

    num_buf = Param.Int(1, "Number of buffer slices")
    buf_size = Param.MemorySize("Size of the buffer")

    table_id = Param.Int('The index of the embedding table')
    

    batch_size = Param.Int("Batch size of the input to determine the reserved region")