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

#include "SmartSSD/if_ctrl.hh"
#include "SmartSSD/dramcache_control.hh"

#include "base/trace.hh"
#include "debug/InterfaceController.hh"
#include "sim/system.hh"
#include "mem/packet.hh"
#include "SmartSSD/smart_ssd.hh"

namespace gem5
{

InterfaceController::InterfaceController(const InterfaceControllerParams &params) :
    AbstractMemory(params),
    serve_event([this]{tryServe();}, params.name+".serve_event"),
    dram_port(params.name + ".dram_port", this, dramQueue),
    dramQueue(*this, dram_port, params.name + ".dram_queue"),
    bus_port(params.name + ".bus_port", this, busQueue),
    busQueue(*this, bus_port),
    batch_size(params.batch_size),
    cache_ctrl(params.dcache_ctrl),
    table_ids(params.table_ids),
    //requestID(params.system->getRequestorId(this)),
    _req_per_entry(cache_ctrl->getReqPerEntry()),
    dram_offset(0)
{ 
    for(auto id : table_ids)
    {
        _grad_done.emplace(id, 0);
    }
}

bool InterfaceController::handleResponse(PacketPtr pkt)
{
    //pkt should have been made response so we only need to forward it
    DPRINTF(InterfaceController, "Get response for %llx\n", pkt->getAddr());        
    if(pkt->isWrite())
    {
        //The gradient write is finished
        //We need to send response and start the backward
        auto res = pendingReqs.find(pkt->getAddr());
        panic_if(res == pendingReqs.end(), "The response is not found in the pending queue.");
        auto req = res->second;
        DPRINTF(InterfaceController, "Get response for gradient writeback %llx\n", req.offset);
        _grad_done[req.table_id]++;

        if(_grad_done[req.table_id] == _req_per_entry)
        {

            //We have received all gradients and is good to go backward
            auto kernel = cache_ctrl->getKernel(req.table_id);
            if(kernel!=nullptr) //If we have a kernel for this table
            {
                DPRINTF(InterfaceController, "Schedule the backward start event for kernel %d\n", req.table_id);
                //Start the backward process (as well as the speculative emb)
                kernel->schedule(kernel->start_back_event, clockEdge());
                kernel->schedule(kernel->fin_for_event, clockEdge() + 5);
            }
            _grad_done[req.table_id] = 0;
        }

        //Revert the original address range;
        pkt->setAddr(req.offset+range.start());

        busQueue.schedSendTiming(pkt, clockEdge());
        //Pending request is done, now remove it from the queue
        pendingReqs.erase(res);
    }else if(pkt->isRead())
    {
        auto res = pendingReqs.find(pkt->getAddr());
        panic_if(res == pendingReqs.end(), "The response is not found in the pending queue.");
        auto req = res->second;
        pkt->setAddr(req.offset+range.start());

        //For read request, we just need to send the response to bus
        busQueue.schedSendTiming(pkt, clockEdge());
        pendingReqs.erase(res);
    }

    return true;
}

bool InterfaceController::handleRequest(PacketPtr pkt)
{
    //Decode the request and insert into the incoming queue
    GPURequest req = {
        pkt, pkt->getAddr()-range.start(),
        table_ids[(pkt->getAddr()-range.start() ) / (batch_size * _req_per_entry * 64)],
        (pkt->getAddr()-range.start() ) % (batch_size * _req_per_entry * 64) / (_req_per_entry * 64)
    };

    DPRINTF(InterfaceController, "Received a request for Addr %llx, offset %d, table_id %d\n", pkt->getAddr(), req.offset, req.table_id);

    if(pkt->isWrite())
    {
        //Recalculate table_id
        req.table_id = table_ids[(req.offset - (batch_size * _req_per_entry * 64 * table_ids.size())) / (_req_per_entry*64)];
        req.sample_id = 0;
        DPRINTF(InterfaceController, "Gradient Write for Addr %llx, offset %d, table_id %d\n", pkt->getAddr(), req.offset, req.table_id);
        panic_if(std::find(table_ids.begin(), table_ids.end(), req.table_id) == table_ids.end(),
                "Incoming request is not in the range of the SmartSSD kernel.");

        pendingReqs.emplace(req.offset + dram_offset, req);

        pkt->setAddr(req.offset + dram_offset);
        //Receive a gradient writeback
        //Count the number of request until we can start the backward
         DPRINTF(InterfaceController, "Schdule a request for gradient write to %llx \n", pkt->getAddr());
        dramQueue.schedSendTiming(pkt, clockEdge());
        //Move this part to the response
    }else if(pkt->isRead()){
        if(std::find(table_ids.begin(), table_ids.end(), req.table_id) == table_ids.end())
        {
            pkt->makeResponse();
            busQueue.schedSendTiming(pkt, clockEdge());
        }
        //panic_if(std::find(table_ids.begin(), table_ids.end(), req.table_id) == table_ids.end(),
        //        "Incoming request is not in the range of the SmartSSD kernel.");
        incomingQueue.push(req);
    }
    if(!serve_event.scheduled())
        schedule(serve_event, clockEdge()); //Avoid collision

    return true;
}

void InterfaceController::tryServe()
{
    //We do a round-robin here
    if(!incomingQueue.empty())
    {
        //Check whether the head can be served
        if(cache_ctrl->checkDone(incomingQueue.front().table_id, incomingQueue.front().sample_id))
        {
            //Send the request to read memory and add to the pending queue
            GPURequest req = incomingQueue.front();
            //Conver the address into the internal range
            incomingQueue.pop();
            Addr dst_addr = dram_offset + req.offset;
            req.pkt->setAddr(dst_addr);

            pendingReqs.emplace(dst_addr, req);

            dramQueue.schedSendTiming(req.pkt, clockEdge());
            DPRINTF(InterfaceController, "Send a packet out to DRAM, Addr %llx, with dram offset being %llx\n", dst_addr, dram_offset);
            

        }else{
            //Pop and add to the holding queue
            holdingQueue.push(incomingQueue.front());
            incomingQueue.pop();
        }
    }

    //Now we check the holding queue 
    if(!holdingQueue.empty())
    {
        //Check whether the head can be served
        if(cache_ctrl->checkDone(holdingQueue.front().table_id, holdingQueue.front().sample_id))
        {
            //Send the request to read memory and add to the pending queue
            GPURequest req = holdingQueue.front();
            holdingQueue.pop();
            Addr dst_addr = dram_offset + req.offset;
            req.pkt->setAddr(dst_addr);

            DPRINTF(InterfaceController, "Send a packet out to DRAM, Addr %llx, with dram offset being %llx\n", dst_addr, dram_offset);
            
            pendingReqs.emplace(dst_addr, req);

            //We don't need to sent mutiple requests since the GPU request are also split into 64 Bytes
            dramQueue.schedSendTiming(req.pkt, clockEdge(Cycles(1)));
        }
    }

    //Keep Trying at the next cycle
    if(!holdingQueue.empty() || !incomingQueue.empty())
    {
        schedule(serve_event, clockEdge(Cycles(1)));    //Schedule when there are pending requests
    }
}


Port &
InterfaceController::getPort(const std::string &if_name, PortID idx)
{
    panic_if(idx != InvalidPortID, "This object doesn't support vector ports");

    // This is the name from the Python SimObject declaration (InterfaceController.py)
    if (if_name == "bus_port") {
        return bus_port;
    } else if (if_name == "dram_port") {
        return dram_port;
    } else {
        // pass it along to our super class
        return SimObject::getPort(if_name, idx);
    }
}

AddrRangeList
InterfaceController::BusSidePort::getAddrRanges() const
{
    return owner->getAddrRanges();
}

void
InterfaceController::BusSidePort::recvFunctional(PacketPtr pkt)
{
    // Just forward to the memobj.
    return owner->handleFunctional(pkt);
}

bool
InterfaceController::BusSidePort::recvTimingReq(PacketPtr pkt)
{
    //Accept all requests
    //TODO: Define a buffer size for the interface
    return owner->handleRequest(pkt);
    return true;
}

void
InterfaceController::BusSidePort::recvRespRetry()
{
    queue.retry();
}

bool
InterfaceController::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    // Just forward to the memobj.
    return owner->handleResponse(pkt);
}

void
InterfaceController::MemSidePort::recvReqRetry()
{
    queue.retry();    
}

void
InterfaceController::MemSidePort::recvRangeChange()
{
    owner->sendRangeChange();
}

void
InterfaceController::handleFunctional(PacketPtr pkt)
{
    // Just pass this on to the memory side to handle for now.
    pkt->setAddr(pkt->getAddr()-range.end() + dram_offset);
    dram_port.sendFunctional(pkt);
}

AddrRangeList
InterfaceController::getAddrRanges() const
{
    DPRINTF(InterfaceController, "Sending new ranges\n");
    // Just use the same ranges as whatever is on the memory side.
    AddrRangeList ranges;
    ranges.push_back(range);
    
    return ranges;
}

void
InterfaceController::sendRangeChange()
{
    DPRINTF(InterfaceController, "Get Range changes. Now DRAM Start 0x%llx\n", dram_port.getAddrRanges().front().start());
    dram_offset = dram_port.getAddrRanges().front().start();

    bus_port.sendRangeChange();
}

} // namespace gem5
