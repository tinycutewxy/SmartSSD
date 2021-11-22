from m5.SimObject import SimObject


Import("*")

print("SSD Mem compiled")
SimObject('SSDMem.py')
SimObject("DRAMCacheControl.py")
SimObject("EmbKernel.py")
SimObject("InterfaceController.py")
SimObject('SmartSSD.py')

Source('ssd_mem.cc')
Source('dramcache_control.cc')
Source('emb_table.cc')
Source('emb_kernel.cc')
Source('if_ctrl.cc')

DebugFlag('SimpleSSD')
DebugFlag('DRAMCacheControl')
DebugFlag('EmbKernel')
DebugFlag('InterfaceController')
