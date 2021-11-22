/*
 * Copyright (c) 2017 Jason Lowe-Power
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __IF_CTRL_HH__
#define __IF_CTRL_HH__

#include "mem/port.hh"
#include "mem/packet_queue.hh"
#include "params/InterfaceController.hh"
#include "sim/sim_object.hh"
#include "sim/clocked_object.hh"
#include "SmartSSD/emb_table.hh"
#include "base/types.hh"
#include "mem/abstract_mem.hh"
#include "sim/system.hh"
#include "SmartSSD/dramcache_control.hh"

#include <fstream>
#include <queue>
#include <map>

namespace gem5
{

/**
 * A very simple memory object. Current implementation doesn't even cache
 * anything it just forwards requests and responses.
 * This memobj is fully blocking (not non-blocking). Only a single request can
 * be outstanding at a time.
 */
class InterfaceController : public memory::AbstractMemory
{
  private:
    typedef struct GPURequest
    {
        PacketPtr pkt;
        Addr offset;
        uint64_t table_id;
        uint64_t sample_id;
    } GPURequest;
    
    
    class MemSidePort : public RequestPort
    {
      private:
        /// The object that owns this object (InterfaceController)
        InterfaceController *owner;
        ReqPacketQueue &queue;

      public:
        /**
         * Constructor. Just calls the superclass constructor.
         */
        MemSidePort(const std::string& name, InterfaceController *owner, ReqPacketQueue &queue) :
            RequestPort(name, owner), owner(owner),  queue(queue)
        { }

      protected:
        /**
         * Receive a timing response from the response port.
         */
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override;
    };

    class BusSidePort : public ResponsePort
    {
      private:
        InterfaceController *owner;
        RespPacketQueue &queue;

      public:
        BusSidePort(const std::string& name, InterfaceController *owner, RespPacketQueue &queue) :
            ResponsePort(name, owner), owner(owner), queue(queue)
        { }

        AddrRangeList getAddrRanges() const override;


      protected:

        Tick recvAtomic(PacketPtr pkt) override
        { panic("recvAtomic unimpl."); }

        void recvFunctional(PacketPtr pkt) override;
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
    };

    bool handleRequest(PacketPtr pkt);
    bool handleResponse(PacketPtr pkt);
    void handleFunctional(PacketPtr pkt);
    AddrRangeList getAddrRanges() const;
    void sendRangeChange();

    EventFunctionWrapper serve_event;
    
    /// Instantiation of the memory-side port
    MemSidePort dram_port;
    ReqPacketQueue dramQueue;

    BusSidePort bus_port;
    RespPacketQueue busQueue;

    uint batch_size;

    DRAMCacheControl *cache_ctrl;   //Retrive Info about the embedding table
    std::vector<uint> table_ids;

    std::queue<GPURequest> incomingQueue, holdingQueue;
    std::map<Addr, GPURequest> pendingReqs;
    
    RequestorID requestID;

    uint _req_per_entry = 0;
    std::map<uint, uint> _grad_done;

    Addr dram_offset;


  public:

    InterfaceController(const InterfaceControllerParams &params);
    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;
    void init(System *sys)
    {
      requestID = sys->getRequestorId(this);
      cache_ctrl->init(sys);
    }
    void tryServe();        //Also should be called by kernel when the calculate is done
};

} // namespace gem5

#endif // __LEARNING_GEM5_PART2_SIMPLE_MEMOBJ_HH__
