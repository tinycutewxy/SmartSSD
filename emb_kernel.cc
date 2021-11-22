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

#include "SmartSSD/emb_kernel.hh"
#include "SmartSSD/dramcache_control.hh"

#include "base/trace.hh"
#include "debug/EmbKernel.hh"
#include "sim/system.hh"
#include "mem/packet.hh"

namespace gem5
{

EmbKernel::EmbKernel(const EmbKernelParams &params) :
    ClockedObject(params),
    //requestID(params.system->getRequestorId(this)),
    entry_rd_port(params.name + ".entry_rd_port", this, entryRdQueue),
    entry_wr_port(params.name + ".entry_wr_port", this, entryWrQueue),
    grad_rd_port(params.name + ".grad_rd_port", this, gradRdQueue),
    result_wr_port(params.name + ".result_wr_port", this, resultWrQueue),
    dram_offset(0), batch_size(params.batch_size),
    entryRdQueue(*this, entry_rd_port, params.name + ".entry_rd_queue"),
    entryWrQueue(*this, entry_wr_port, params.name + ".entry_wr_queue"),
    gradRdQueue(*this, grad_rd_port, params.name + ".grade_rd_queue"),
    resultWrQueue(*this, result_wr_port, params.name + ".result_wr_queue"),
    table_id(params.table_id),
    buffer_size(params.buf_size),
    n_buf_slices(params.num_buf),
    //table_item(cache_ctrl->getEmbTable(table_id)),
    //_req_per_entry(cache_ctrl->getReqPerEntry()),
    start_for_event([this]{startForward();}, name() + ".start_for"),
    fin_for_event([this]{finishForward();}, name() + ".fin_for"),
    start_back_event([this]{startBackward();}, name() + ".start_back"),
    fin_back_event([this]{finishBackward();}, name() + ".fin_back"),
    stats(this)
{
    emb_trace.open(params.trace_file, std::ios::in);
    panic_if(!emb_trace.good(), "Cannot open the trace file for the embedding kernel!");
}

void EmbKernel::startBackward()
{
     DPRINTF(EmbKernel, "Start Backward for iter %d\n", _iter_id);
    start_backward = true;
    //schedule(fin_for_event, clockEdge());
    auto streamList = table_item->getBackwardEntries();
    if(_iter_id==0)     //We do backward for iter 0 at the next cycle
    {
        schedule(fin_back_event, clockEdge());
    }
    uint delays = 0;
    for(auto it : streamList)
    {
        Request::Flags flags;
        PacketPtr rd, wr;
        for (int j = 0; j<_req_per_entry;j++)
        {
            rd = new Packet(std::make_shared<Request>(it.second + dram_offset + j * 64, 64, flags, requestID), 
                            MemCmd::ReadReq);
            rd->allocate();
            gradRdQueue.schedSendTiming(rd, clockEdge(Cycles(delays)));
            ++delays;
        }
        ++delays;
        for (int j = 0; j<_req_per_entry;j++)
        {
            wr = new Packet(std::make_shared<Request>(it.second + dram_offset + j * 64, 64, flags, requestID), 
                            MemCmd::WriteReq);
            wr->allocate();
            entryWrQueue.schedSendTiming(wr, clockEdge(Cycles(delays)));
            ++delays;
            backward_packets ++;
        }
    }
    //schedule(EventFunctionWrapper([this]{finishBackward();}), clockEdge(delays));       
}

void EmbKernel::finishBackward(){
    DPRINTF(EmbKernel, "Finish Backward for iter %d\n", _iter_id);
    fin_backward = clockEdge();
    _iter_id++;
}

void EmbKernel::startForward()
{
    DPRINTF(EmbKernel, "Start Forward for iter %d\n", _iter_id);
    start_backward = false; //Fix for iter0
    fin_forward = false;
    //Clean the on-chip buffer and index queue
    while(!idx_order.empty()){idx_order.pop_front();}

    //Load the trace from file
    //Defination
    //Each line is for one sample
    //Separate the iterations by white line
    std::string line, token;
    uint proc_delay= 0, mem_delay = 0;
    for(int s_id = 0; s_id<batch_size; s_id++)
    {
        std::getline(emb_trace, line);
        std::istringstream extract_line(line);
        DPRINTF(EmbKernel, "Current iter %d, Raw Input indices: %s\n", _iter_id, line);
        while(std::getline(extract_line, token, ','))
        {
            uint64_t index = std::strtoul(token.c_str(), 0L, 10);
            token.clear();
            //First Check if this entry is  on-chip
            if(std::find(idx_order.begin(), idx_order.end(), index)!=idx_order.end())
            { ++proc_delay ;}
            else{
                Request::Flags flags;
                PacketPtr rd;
                for (int j = 0; j<_req_per_entry;j++)
                {
                    rd = new Packet(std::make_shared<Request>(
                        table_item->getCacheAddr(index) + dram_offset
                             + j * 64, 64, flags, requestID), MemCmd::ReadReq);
                    rd->allocate();
                    entryRdQueue.schedSendTiming(rd, clockEdge(Cycles(mem_delay)));
                    ++mem_delay;
                    forward_packets++;
                }
            }
        }
        extract_line.clear();
        extract_line.seekg(0);
        //Now we refresh the on-chip buffer
         while(std::getline(extract_line, token, ','))
        {
            uint64_t index = std::strtoul(token.c_str(), 0L, 10);
            token.clear();
            stats.hits++;
            //First Check if this entry is  on-chip
            if(std::find(idx_order.begin(), idx_order.end(), index)==idx_order.end())
            {
                stats.hits--;
                stats.misses++;
                        //Then we fakely insert this entry into the buffer
                if(idx_order.size()>=buffer_size/(64 * _req_per_entry))
                {
                    //Pop the head item first
                    idx_order.pop_front();
                }
                //Then push current one
                idx_order.push_back(index);
            }
        }
    }
    // We need to wait for the memory transactions finish to set the fin_forward
    schedule(fin_for_event, clockEdge(Cycles(std::max<uint>(proc_delay/n_buf_slices, mem_delay))));

    std::string tmp_line;
    std::getline(emb_trace, tmp_line);
    panic_if(tmp_line.size()!=0, "Missing the separator in the trace file!");        
}

void EmbKernel::finishForward()
{
    if(!(fin_forward))
    {
        DPRINTF(EmbKernel, "Can't start finForward for iter %d, fin_forward %d, start_backward %d\n", _iter_id, fin_forward, start_backward);
        return;         //We cannot start the writeback yet
    }
    //We have make sure the gradient has arrived and forward pending memory requests have finished
    //Now we need to add # of uint to add the gradient and write the result back to memory
    
    //We reset the counter here so that we can start the forward
    //Of the next iteration immediately when the prefetch is done
    //And do not need to wait for GPU to fetch the result
    DPRINTF(EmbKernel, "Finish Forward for iter %d, now writing back the result vector\n", _iter_id);
    fin_counter = 0;
    Request::Flags flags;
    PacketPtr wr;
    uint delay = 0;
    for(int i = 0; i<batch_size; i++)
    {
        for (int j = 0; j<_req_per_entry;j++)
        {
            //We use the head of dram to store the result
            wr = new Packet(std::make_shared<Request>(i + dram_offset
                        + j * 64, 64, flags, requestID), MemCmd::WriteReq);
            wr->allocate();
            resultWrQueue.schedSendTiming(wr, clockEdge(Cycles(delay)));
            delay ++;
        }
    }
    fin_forward = false;
}

bool
EmbKernel::handleResponse(PacketPtr pkt, RequestPort *port)
{   
    if(port == &entry_rd_port)  //If it's a entry read from forward
    {
        forward_packets--;
        if(forward_packets==0)
        {
            fin_forward = true;
            if(!fin_for_event.scheduled())
                schedule(fin_for_event, clockEdge());
        }
    }else if(port == &entry_wr_port)    //If it's a write response for backward
    {
        backward_packets--;
        if(backward_packets==0)
        {   if(!fin_back_event.scheduled())
                schedule(fin_back_event, clockEdge());
        }
    }else if(port == &result_wr_port)   //If it's a write response for result
    {
        DPRINTF(EmbKernel, "%d sample has been written to DRAM\n", fin_counter);
        fin_counter++;
    }
    return true;
}

void EmbKernel::startup()
{
    //Only for test purpose
    //schedule(prefetch_event, curTick());
}

Port &
EmbKernel::getPort(const std::string &if_name, PortID idx)
{
    panic_if(idx != InvalidPortID, "This object doesn't support vector ports");

    // This is the name from the Python SimObject declaration (EmbKernel.py)
    if (if_name == "entry_rd_port") {
        return entry_rd_port;
    } else if (if_name == "entry_wr_port") {
        return entry_wr_port;
    } 
    else if (if_name == "grad_rd_port") {
        return grad_rd_port;
    } 
    else if (if_name == "result_wr_port") {
        return result_wr_port;
    } else {
        // pass it along to our super class
        return SimObject::getPort(if_name, idx);
    }
}


bool
EmbKernel::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    // Just forward to the memobj.
    return owner->handleResponse(pkt, this);
}

void
EmbKernel::MemSidePort::recvReqRetry()
{
    queue.retry();    
}

void
EmbKernel::MemSidePort::recvRangeChange()
{
    owner->sendRangeChange();
}

void
EmbKernel::handleFunctional(PacketPtr pkt)
{
    // Just pass this on to the memory side to handle for now.
    entry_rd_port.sendFunctional(pkt);
}

AddrRangeList
EmbKernel::getAddrRanges() const
{
    DPRINTF(EmbKernel, "Sending new ranges\n");
    // Just use the same ranges as whatever is on the memory side.
    return entry_rd_port.getAddrRanges();
}

void
EmbKernel::sendRangeChange()
{
    DPRINTF(EmbKernel, "Get Range changes. Now DRAM Start 0x%llx\n", entry_rd_port.getAddrRanges().front().start());
    dram_offset = entry_rd_port.getAddrRanges().front().start();
}

EmbKernel::EmbKernelStats::EmbKernelStats(statistics::Group *parent)
      : statistics::Group(parent),
      ADD_STAT(hits, statistics::units::Count::get(), "Number of DRAMCache hits"),
      ADD_STAT(misses, statistics::units::Count::get(), "Number of DRAMCache misses"),
      ADD_STAT(hitRatio, statistics::units::Ratio::get(),
               "The ratio of hits to the total accesses to the cache",
               hits / (hits + misses))
{

}

} // namespace gem5
