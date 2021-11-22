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

#ifndef __DRAM_CACHE_CONTROL_HH__
#define __DRAM_CACHE_CONTROL_HH__

#include "mem/port.hh"
#include "mem/packet_queue.hh"
#include "params/DRAMCacheControl.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"
#include "SmartSSD/emb_table.hh"
#include "SmartSSD/emb_kernel.hh"

#include <fstream>

namespace gem5
{

  /**
 * A very simple memory object. Current implementation doesn't even cache
 * anything it just forwards requests and responses.
 * This memobj is fully blocking (not non-blocking). Only a single request can
 * be outstanding at a time.
 */
  class DRAMCacheControl : public SimObject
  {
  private:
    class MemSidePort : public RequestPort
    {
    private:
      /// The object that owns this object (DRAMCacheControl)
      DRAMCacheControl *owner;
      ReqPacketQueue &queue;

    public:
      /**
         * Constructor. Just calls the superclass constructor.
         */
      MemSidePort(const std::string &name, DRAMCacheControl *owner, ReqPacketQueue &queue) : RequestPort(name, owner), owner(owner), queue(queue)
      {
      }

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
    bool handleResponse(PacketPtr pkt);
    void handleFunctional(PacketPtr pkt);
    AddrRangeList getAddrRanges() const;

    void sendRangeChange();

    /// Instantiation of the memory-side port
    MemSidePort memPort;
    MemSidePort ssdPort;

    RequestorID requestID;

    ReqPacketQueue memReqQueue;
    ReqPacketQueue ssdReqQueue;

    // Main event for cache management
    EventFunctionWrapper prefetch_event, check_event;
    std::vector<EventFunctionWrapper> cleanup_events;

    std::vector<EmbKernel *> kernels; //Embedding kernels to call

    /// Keep a count on all the packets we sent.
    uint packet_count = 0;
    //All requests for this iteration have been sent
    bool request_sent = false;

    //Cannot start the next forward since the transactions are not finished yet
    bool pending_forward = false; 

    std::ifstream emb_trace;

    Addr ssd_offset, dram_offset;

    uint _iter_idx_fin = 0;  //The index of the finished iteration.
    uint _req_per_entry = 0; //Number of memory requests we should issue per table entry

    std::map<uint64_t, Addr> cache_table; //The table to store the current entries inside the DRAM.
    std::vector<emb_table> emb_tables;

    Tick processLatency;        //The latency between sending memory requests in ps (Tick)
    uint _n_tables_cleaned = 0; //Number of tables that have been cleaned up

    void prefetch();
    void cache_memcpy(Addr src, Addr dst, Tick when); //Handy function for mem copy
    void checkDone();
  protected:
    struct DRAMCacheControlStats : public statistics::Group
    {
        DRAMCacheControlStats(statistics::Group *parent);
        statistics::Scalar hits;
        statistics::Scalar misses;
        statistics::Formula hitRatio;
    } stats;

  public:
    DRAMCacheControl(const DRAMCacheControlParams &params);
    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    void init(System * sys)
    {
      requestID = sys->getRequestorId(this);
      for(auto it : kernels)
      {
        it->init(sys);
      }
    }
    bool isReady(uint iter_id) const;

    emb_table &getEmbTable(uint table_id)
    {
      for (int i=0; i<emb_tables.size(); i++)
      {
        if (emb_tables[i].table_id == table_id)
          return emb_tables[i];
      }
    }

    uint getReqPerEntry() const
    {
      return _req_per_entry;
    }

    EmbKernel* getKernel(uint table_id)
    {
      for (auto ek:kernels)
      {
        if(ek->table_id == table_id)
        {
          return ek;
        }
      }
      
      return nullptr;
    }

    bool checkDone(uint table_id, uint sample_id)
    {
      for (auto it : emb_tables)
        if (it.table_id == table_id)
          //Now check the fin_counter of the kernel
          for (auto ek : kernels)
            if (ek->table_id == table_id)
              return ek->checkDone(sample_id);

      //If we do not find it in the emb_tables, these samples are in on-chip buffer and has finished
      return true;
    }

    void startup();
    void cleanup(uint table_id); //Called by the kernel
  };

} // namespace gem5

#endif // __LEARNING_GEM5_PART2_SIMPLE_MEMOBJ_HH__
