#This is a cache control unit for prefetching n-iters look-ahead cache
#Base on the simple_memobj from the tutorial

from m5.params import *
from m5.SimObject import SimObject

from m5.proxy import *

class DRAMCacheControl(SimObject):
    type = 'DRAMCacheControl'
    cxx_header = "SmartSSD/dramcache_control.hh"
    cxx_class = 'gem5::DRAMCacheControl'

    dram_port = RequestPort("Mem side port, sends requests")
    ssd_side = RequestPort("SSD side port, sends requests")

    trace_file = Param.String("trace file of the DLRM input")

    vlen = Param.Int("Vector Int of the embedding table")
    n_tables = Param.Int("Number of embedding tables")
    cache_ratio = VectorParam.Float("Percentage assignment of the DRAM Cache for each table")
    table_size = VectorParam.Int("Table size")
    table_ids = VectorParam.Int("Table IDs")
    emb_kernels = VectorParam.EmbKernel("Embedding Kernels")
    process_lat = Param.Tick(1, "Delay between sending memory requests")

    batch_size = Param.Int("Batch size of the input to determine the reserved region")