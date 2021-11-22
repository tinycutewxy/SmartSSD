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

#include "SmartSSD/dramcache_control.hh"

#include "base/trace.hh"
#include "debug/DRAMCacheControl.hh"
#include "sim/system.hh"

namespace gem5
{

DRAMCacheControl::DRAMCacheControl(const DRAMCacheControlParams &params) :
    SimObject(params),
    //requestID(params.system->getRequestorId(this)),
    memPort(params.name + ".mem_side", this, memReqQueue),
    ssdPort(params.name + ".ssd_side", this, ssdReqQueue),
    processLatency(params.process_lat),
    memReqQueue(*this, memPort, params.name + ".mem_queue"),
    ssdReqQueue(*this, ssdPort, params.name + ".ssd_queue"),
    ssd_offset(0), dram_offset(0), 
    prefetch_event([this]{prefetch();}, name() + ".prefetch"),
    check_event([this]{checkDone();}, name()+ ".check"),
    kernels(params.emb_kernels),
    stats(this)
    //cleanup_event([this]{cleanup();}, name() + ".cleanup")
{
    emb_trace.open(params.trace_file, std::ios::in);
    panic_if(!emb_trace.good(), "Cannot open the trace file for the DRAM Cache!");

    //Determine how many requests we should issue per embedding table entry
    _req_per_entry = params.vlen / 64;

    //Calculate the cache address for each table
    Addr cache_start = params.batch_size * params.vlen + memPort.getAddrRanges().front().start();
    size_t cache_size = memPort.getAddrRanges().front().end() - cache_start;

    //Initialize the emb table objects to obtain the address in ssd.
    Addr table_head = ssdPort.getAddrRanges().front().start();
    for(int i=0; i<params.n_tables; i++)
    {
        emb_tables.push_back(
            emb_table(params.table_size[i], params.vlen, (uint64_t)cache_size*params.cache_ratio[i],
            table_head, cache_start, params.table_ids[i])
            );
        cache_start += (uint64_t)cache_size*params.cache_ratio[i];
        table_head += emb_tables[i].getAddr(params.table_size[i] - 1);
    }

    //Set up the emb kernels with the embedding table
    for(int i=0; i<params.n_tables; i++)
    {
        kernels[i]->init(&emb_tables[i], _req_per_entry);
    }

    for(auto it: kernels)
    {
        cleanup_events.emplace_back( 
            [this, it]{cleanup(it->table_id);},
            name() + ".cleanup" + std::to_string(it->table_id)
        );
    }
    //We should be good now.
}

void DRAMCacheControl::cache_memcpy(Addr src, Addr dst, Tick when)
{
    Request::Flags flags;
    for(int j= 0; j<_req_per_entry; j++)
    {
        PacketPtr rd, wr;

        rd = new Packet(std::make_shared<Request>(src + j * 64, 64, flags, requestID), MemCmd::ReadReq);
        wr = new Packet(std::make_shared<Request>(dst + j * 64, 64, flags, requestID), MemCmd::WriteReq);

        rd->allocate();
        wr->allocate();

        memReqQueue.schedSendTiming(rd, when);
        memReqQueue.schedSendTiming(wr, when + processLatency);
        packet_count += 2;    //No response for write
    }
}

//Prefetch the entries for the next iteration
void DRAMCacheControl::prefetch()
{
    //The main loop of the cache management process
    //First we need to read from the trace file
    //Defination
    //Each line is for one table and one iteration
    //one white line between iterations

    //First we need to check and update the current state

    //We have finished all processing
    if(emb_trace.eof()) 
        return;


    request_sent = false;
    Tick delays = 0;    //Accumulated delays for sending mem requests
    for(int i =0; i<emb_tables.size(); i++)
    {
        std::string line, token;
        std::getline(emb_trace, line);
        std::istringstream extract_line(line);
        DPRINTF(DRAMCacheControl, "Current iter %d, TableID %d, Raw Input indices: %s\n", _iter_idx_fin, i, line);
        while(std::getline(extract_line, token, ','))
        {
            uint64_t index = std::strtoul(token.c_str(), 0L, 10);
            token.clear();
            auto addresses = emb_tables[i].insertEntry(index);
            stats.hits++;
            if(addresses.size()==2)     //Need mem cpy
            {
                cache_memcpy(addresses[0] + dram_offset, addresses[1] + dram_offset, curTick() + delays);
                delays += 2 * processLatency;
                
            }else if(addresses.size()==1)   //Need to load from SSD
            {
                stats.hits--;
                stats.misses++;
                Addr ssd_addr = emb_tables[i].getAddr(index);
                Request::Flags flags;
                PacketPtr rd;

                packet_count ++;

                DPRINTF(DRAMCacheControl, "Load from SSD, Index %d, Addr: 0x%llx\n", index, ssd_addr);
                delays += processLatency;
                rd = new Packet(std::make_shared<Request>(ssd_addr + ssd_offset, _req_per_entry * 64, flags, requestID), MemCmd::ReadReq);
                rd->allocate();
                ssdReqQueue.schedSendTiming(rd, curTick() + delays);
                

                for(int j= 0; j<_req_per_entry; j++)
                {
                    PacketPtr wr;
                    wr = new Packet(std::make_shared<Request>(addresses[0] + dram_offset + j * 64, 64, flags, requestID), MemCmd::WriteReq);
                    wr->allocate();
                    packet_count ++;
                    delays += processLatency;
                    memReqQueue.schedSendTiming(wr, curTick()+delays);
                }
            }//else do nothing. Wait for pending process

        }

    }
    std::string tmp_line;
    std::getline(emb_trace, tmp_line);
    panic_if(tmp_line.size()!=0, "Missing the separator in the trace file!");

    if(_iter_idx_fin == 0)
    {
        request_sent = true;        //Do not need cleanup for iter 0
    }
    //For test purpose only
    //schedule(cleanup_event, curTick() + delays);
}

void DRAMCacheControl::checkDone()
{
    for(int i =0; i<kernels.size(); i++)
    {
        if(kernels[i]->fin_backward!=0)
        {
            DPRINTF(DRAMCacheControl, "Check kernel %d, able to cleanup\n", kernels[i]->table_id);
            schedule(cleanup_events[i], std::max(kernels[i]->fin_backward, curTick()));
            kernels[i]->fin_backward = 0;
        }
    }

    if(pending_forward && request_sent)
    {
        pending_forward = false;
        for(auto it : kernels)
            it->startForward();
        _iter_idx_fin++;
        if(!prefetch_event.scheduled())
            schedule(prefetch_event, curTick() + processLatency);    //Start to prefetch the next iter when ready.
    }
    if(!check_event.scheduled())
    {
        schedule(check_event, curTick()+100000);
    }
}

//Pop the iterations of the last iter and do cleanup
//Should called by the kernel when the backward has finished
void DRAMCacheControl::cleanup(uint table_id)
{
    DPRINTF(DRAMCacheControl, "Clean up for table %d\n", table_id);
    Tick delays = 0;    //Accumulated delays for sending mem requests
    for(int i =0; i<emb_tables.size(); i++)
    {
        if(emb_tables[i].table_id != table_id)
            continue;

        auto pending_requests = emb_tables[i].getPendingMemcopy();
        for(auto it: pending_requests)
        {
            cache_memcpy(it.first, it.second, curTick()+delays);
            delays += 2 * processLatency;
        }
        //Now we cleanup the writeback requests
        auto wbs = emb_tables[i].popOldIterEntries();
        for(auto it: wbs)
        {
            Addr ssd_addr = emb_tables[i].getAddr(it.first);
            Request::Flags flags;
            PacketPtr wr;

            for(int j= 0; j<_req_per_entry; j++)
            {
                PacketPtr rd;
                rd = new Packet(std::make_shared<Request>(it.second + dram_offset + j * 64, 64, flags, requestID), MemCmd::ReadReq);
                rd->allocate();
                delays += processLatency;
                memReqQueue.schedSendTiming(rd, curTick()+delays);
                packet_count++;
            }         
            wr = new Packet(std::make_shared<Request>(ssd_addr + ssd_offset, _req_per_entry * 64, flags, requestID), MemCmd::WriteReq);
            wr->allocate();
            ssdReqQueue.schedSendTiming(wr, curTick() + delays);
            delays += processLatency;
        }
    }

    _n_tables_cleaned++;

    if(_n_tables_cleaned == emb_tables.size() || _iter_idx_fin==0)  //Fix for the 0-th iteration
    {
        _n_tables_cleaned = 0;
        request_sent = true;    //All request sent, wait for response to be ready    
        DPRINTF(DRAMCacheControl, "All request done for iteration %d\n", _iter_idx_fin);
    }
}

void DRAMCacheControl::startup()
{
    schedule(prefetch_event, curTick());


    schedule(check_event, curTick()+processLatency);
}

Port &
DRAMCacheControl::getPort(const std::string &if_name, PortID idx)
{
    panic_if(idx != InvalidPortID, "This object doesn't support vector ports");

    // This is the name from the Python SimObject declaration (DRAMCacheControl.py)
    if (if_name == "dram_port") {
        return memPort;
    } else if (if_name == "ssd_side") {
        return ssdPort;
    } else {
        // pass it along to our super class
        return SimObject::getPort(if_name, idx);
    }
}

// void
// DRAMCacheControl::MemSidePort::sendPacket(PacketPtr pkt)
// {
//     // Note: This flow control is very simple since the memobj is blocking.

//     panic_if(blockedPacket != nullptr, "Should never try to send if blocked!");

//     // If we can't send the packet across the port, store it for later.
//     if (!sendTimingReq(pkt)) {
//         DPRINTF(DRAMCacheControl, "Packet is blocked for %llx", pkt->getAddr());
//         blockedPacket = pkt;
//     }
// }

bool
DRAMCacheControl::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    // Just forward to the memobj.
    return owner->handleResponse(pkt);
}

void
DRAMCacheControl::MemSidePort::recvReqRetry()
{
    queue.retry();    
}

void
DRAMCacheControl::MemSidePort::recvRangeChange()
{
    owner->sendRangeChange();
}

// bool
// DRAMCacheControl::handleRequest(PacketPtr pkt)
// {
//     if (blocked) {
//         // There is currently an outstanding request. Stall.
//         return false;
//     }

//     DPRINTF(DRAMCacheControl, "Got request for addr %#x\n", pkt->getAddr());

//     // This memobj is now blocked waiting for the response to this packet.
//     blocked = true;

//     // Simply forward to the memory port
//     memPort.sendPacket(pkt);

//     return true;
// }

bool
DRAMCacheControl::handleResponse(PacketPtr pkt)
{
    DPRINTF(DRAMCacheControl, "Got response for addr %#x, outstanding packets %d\n", pkt->getAddr(), packet_count);

    // The packet is now done. We're about to put it in the port, no need for
    // this object to continue to stall.
    // We need to free the resource before sending the packet in case the CPU
    // tries to send another request immediately (e.g., in the same callchain).

    packet_count--;
    if(packet_count==0 && request_sent)
    {
        for(auto it : kernels)
            it->startForward();
        _iter_idx_fin++;
        schedule(prefetch_event, curTick() + processLatency);    //Start to prefetch the next iter when ready.
    }
    else if(packet_count == 0){
        pending_forward = true;
    }

    return true;
}

void
DRAMCacheControl::handleFunctional(PacketPtr pkt)
{
    // Just pass this on to the memory side to handle for now.
    memPort.sendFunctional(pkt);
}

AddrRangeList
DRAMCacheControl::getAddrRanges() const
{
    DPRINTF(DRAMCacheControl, "Sending new ranges\n");
    // Just use the same ranges as whatever is on the memory side.
    return memPort.getAddrRanges();
}

void
DRAMCacheControl::sendRangeChange()
{
    DPRINTF(DRAMCacheControl, "Get Range changes. Now SSD Start: 0x%llx, DRAM Start 0x%llx\n", ssdPort.getAddrRanges().front().start(), memPort.getAddrRanges().front().start());
    ssd_offset = ssdPort.getAddrRanges().front().start();
    dram_offset = memPort.getAddrRanges().front().start();
}


DRAMCacheControl::DRAMCacheControlStats::DRAMCacheControlStats(statistics::Group *parent)
      : statistics::Group(parent),
      ADD_STAT(hits, statistics::units::Count::get(), "Number of DRAMCache hits"),
      ADD_STAT(misses, statistics::units::Count::get(), "Number of DRAMCache misses"),
      ADD_STAT(hitRatio, statistics::units::Ratio::get(),
               "The ratio of hits to the total accesses to the cache",
               hits / (hits + misses))
{

}

} // namespace gem5
