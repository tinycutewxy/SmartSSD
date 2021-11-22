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

#ifndef __EMB_KERNEL_HH__
#define __EMB_KERNEL_HH__

#include "mem/port.hh"
#include "mem/packet_queue.hh"
#include "params/EmbKernel.hh"
#include "sim/sim_object.hh"
#include "sim/clocked_object.hh"
#include "SmartSSD/emb_table.hh"
#include "base/types.hh"
#include "sim/system.hh"

#include <fstream>
#include <deque>

namespace gem5
{

/**
 * A very simple memory object. Current implementation doesn't even cache
 * anything it just forwards requests and responses.
 * This memobj is fully blocking (not non-blocking). Only a single request can
 * be outstanding at a time.
 */
class EmbKernel : public ClockedObject
{
  private:
    class MemSidePort : public RequestPort
    {
      private:
        /// The object that owns this object (EmbKernel)
        EmbKernel *owner;
        ReqPacketQueue &queue;

      public:
        /**
         * Constructor. Just calls the superclass constructor.
         */
        MemSidePort(const std::string& name, EmbKernel *owner, ReqPacketQueue &queue) :
            RequestPort(name, owner), owner(owner),  queue(queue)
        { }

        //void sendPacket(PacketPtr pkt);

      protected:
        /**
         * Receive a timing response from the response port.
         */
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override;
    };

    //bool handleRequest(PacketPtr pkt);
    bool handleResponse(PacketPtr pkt, RequestPort *port);
    void handleFunctional(PacketPtr pkt);
    AddrRangeList getAddrRanges() const;
    void sendRangeChange();


    /// Instantiation of the memory-side port
    MemSidePort entry_rd_port, entry_wr_port, grad_rd_port, result_wr_port;
    ReqPacketQueue entryRdQueue, entryWrQueue, gradRdQueue, resultWrQueue;
    
    RequestorID requestID;

    emb_table* table_item;  //Reference to the table management entry

    uint _req_per_entry = 0;

    std::ifstream emb_trace;

    Addr dram_offset;

    std::deque<uint64_t> idx_order;         //Use a queue to keep the order of the on-chip entries
    

    uint _iter_id = 0;

    uint batch_size;
    uint64_t buffer_size;
    uint64_t n_buf_slices;

    uint fin_counter = 0;
    uint forward_packets = 0;
    uint backward_packets = 0;
    uint result_packets = 0;

    bool fin_forward = false, start_backward = false;

  public:

    EmbKernel(const EmbKernelParams &params);
    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;

    void init(System * sys)
    {
      requestID = sys->getRequestorId(this);
    }
    uint table_id;      //The id of embedding table this kernel corresponds to 
    
    EventFunctionWrapper start_for_event, start_back_event, fin_for_event, fin_back_event;
    Tick fin_backward = 0;

    //Call by DRAMCacheControl to setup
    void init(emb_table *table, uint nrequests)
    {
      table_item = table;
      _req_per_entry = nrequests;
    }

    bool checkDone(uint sample_id)
    {
      return sample_id < fin_counter;
    }
    void startup();
    void startForward();
    void startBackward();
    void finishBackward();
    void finishForward();
  protected:
    struct EmbKernelStats : public statistics::Group
    {
        EmbKernelStats(statistics::Group *parent);
        statistics::Scalar hits;
        statistics::Scalar misses;
        statistics::Formula hitRatio;
    } stats;
};

} // namespace gem5

#endif // __LEARNING_GEM5_PART2_SIMPLE_MEMOBJ_HH__
