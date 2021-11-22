/*
 * Copyright (c) 2010-2013, 2015 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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

#include "SmartSSD/ssd_mem.hh"

#include "base/random.hh"
#include "base/trace.hh"
#include "debug/Drain.hh"
#include "debug/SimpleSSD.hh"

namespace gem5
{

SSDMem::SSDMem(const SSDMemParams &p) :
    memory::AbstractMemory(p),
    port(name() + ".port", *this), latency(p.latency),
    latency_var(p.latency_var), bandwidth(p.bandwidth), isBusy(false),
    retryReq(false), retryResp(false),
    releaseEvent([this]{ release(); }, name()),
    dequeueEvent([this]{ dequeue(); }, name())
{
    
    ssdptr.open(p.lat_trace.c_str(), std::ios::in);
    if (!ssdptr.good()) {
       DPRINTF(SimpleSSD, "warn: SimpleSSD latency trace not found. Will use pseudo latency for trace generation");
        ssdeof = true;
    } else {
        DPRINTF(SimpleSSD, "Latency trace found at %s.", p.lat_trace.c_str());
        ssdeof = false;
    }

    ssdtrgen.open(p.out_trace.c_str(), std::ios::out);
    panic_if(!ssdtrgen.good(), "Cannot write to the trace generate file at %s", p.out_trace.c_str());
}

SSDMem::~SSDMem()
{
    if(ssdptr.good())
        ssdptr.close();
    if(ssdtrgen.good())
        ssdtrgen.close();
}

void
SSDMem::init()
{
    memory::AbstractMemory::init();

    // allow unconnected memories as this is used in several ruby
    // systems at the moment
    if (port.isConnected()) {
        port.sendRangeChange();
    }
}

Tick
SSDMem::recvAtomic(PacketPtr pkt)
{
    panic_if(pkt->cacheResponding(), "Should not see packets where cache "
             "is responding");

    access(pkt);
    return getLatency();
}

Tick
SSDMem::recvAtomicBackdoor(PacketPtr pkt, MemBackdoorPtr &_backdoor)
{
    Tick latency = recvAtomic(pkt);
    getBackdoor(_backdoor);
    return latency;
}

void
SSDMem::recvFunctional(PacketPtr pkt)
{
    pkt->pushLabel(name());

    functionalAccess(pkt);

    bool done = false;
    auto p = packetQueue.begin();
    // potentially update the packets in our packet queue as well
    while (!done && p != packetQueue.end()) {
        done = pkt->trySatisfyFunctional(p->pkt);
        ++p;
    }

    pkt->popLabel();
}

bool
SSDMem::recvTimingReq(PacketPtr pkt)
{
    panic_if(pkt->cacheResponding(), "Should not see packets where cache "
             "is responding");

    panic_if(!(pkt->isRead() || pkt->isWrite()),
             "Should only see read and writes at memory controller, "
             "saw %s to %#llx\n", pkt->cmdString(), pkt->getAddr());

    // we should not get a new request after committing to retry the
    // current one, but unfortunately the CPU violates this rule, so
    // simply ignore it for now
    if (retryReq)
    {
        DPRINTF(SimpleSSD, "Packet for %llx received, but blocked for pending retry.\n", pkt->getAddr());
        return false;
    }

    // if we are busy with a read or write, remember that we have to
    // retry
    if (isBusy) {
        DPRINTF(SimpleSSD, "Packet for %llx received, but blocked for being busy.\n", pkt->getAddr());
        retryReq = true;
        return false;
    }

        // --- Grab data for SimpleSSD here --- //
    // Addr pkt->getAddr() OR Addr pkt->getBlockAddr()
    // Bool pkt->isRead()
    // Bool pkt->isWrite()
    // unsigned pkt->getSize()
    if (pkt->isRead())
    {
        ssdtrgen << curTick() << "," << "R" << "," << pkt->getAddr() - getAddrRange().start() << "," << pkt->getSize() <<std::endl;
        DPRINTF(SimpleSSD, "Read: %llu: %u\n", pkt->getAddr() - getAddrRange().start(), pkt->getSize());
    }
    if (pkt->isWrite())
    {
        ssdtrgen << curTick() << "," << "W" << "," << pkt->getAddr() - getAddrRange().start() << "," << pkt->getSize() <<std::endl;
        DPRINTF(SimpleSSD, "Write: %#llu: %u\n", pkt->getAddr() - getAddrRange().start(), pkt->getSize());
    }
    // ------------------------------------ //
    
    // technically the packet only reaches us after the header delay,
    // and since this is a memory controller we also need to
    // deserialise the payload before performing any write operation
    Tick receive_delay = pkt->headerDelay + pkt->payloadDelay;
    pkt->headerDelay = pkt->payloadDelay = 0;

    // update the release time according to the bandwidth limit, and
    // do so with respect to the time it takes to finish this request
    // rather than long term as it is the short term data rate that is
    // limited for any real memory

    // calculate an appropriate tick to release to not exceed
    // the bandwidth limit
    Tick duration = pkt->getSize() * bandwidth  + getLatency();

    // only consider ourselves busy if there is any need to wait
    // to avoid extra events being scheduled for (infinitely) fast
    // memories
    if (duration != 0) {
        schedule(releaseEvent, curTick() + duration);
        isBusy = true;
    }

    // go ahead and deal with the packet and put the response in the
    // queue if there is one
    bool needsResponse = pkt->needsResponse();
    recvAtomic(pkt);
    // turn packet around to go back to requestor if response expected
    if (needsResponse) {
        // recvAtomic() should already have turned packet into
        // atomic response
        assert(pkt->isResponse());

        Tick when_to_send = curTick() + receive_delay + getLatency();

        // typically this should be added at the end, so start the
        // insertion sort with the last element, also make sure not to
        // re-order in front of some existing packet with the same
        // address, the latter is important as this memory effectively
        // hands out exclusive copies (shared is not asserted)
        auto i = packetQueue.end();
        --i;
        while (i != packetQueue.begin() && when_to_send < i->tick &&
               !i->pkt->matchAddr(pkt))
            --i;

        // emplace inserts the element before the position pointed to by
        // the iterator, so advance it one step
        packetQueue.emplace(++i, pkt, when_to_send);

        if (!retryResp && !dequeueEvent.scheduled())
            schedule(dequeueEvent, packetQueue.back().tick);
    } else {
        pendingDelete.reset(pkt);
    }

    return true;
}

void
SSDMem::release()
{
    assert(isBusy);
    isBusy = false;
    if (retryReq) {
        retryReq = false;
        port.sendRetryReq();
    }
}

void
SSDMem::dequeue()
{
    assert(!packetQueue.empty());
    DeferredPacket deferred_pkt = packetQueue.front();

    retryResp = !port.sendTimingResp(deferred_pkt.pkt);

    if (!retryResp) {
        packetQueue.pop_front();

        // if the queue is not empty, schedule the next dequeue event,
        // otherwise signal that we are drained if we were asked to do so
        if (!packetQueue.empty()) {
            // if there were packets that got in-between then we
            // already have an event scheduled, so use re-schedule
            reschedule(dequeueEvent,
                       std::max(packetQueue.front().tick, curTick()), true);
        } else if (drainState() == DrainState::Draining) {
            DPRINTF(Drain, "Draining of SSDMem complete\n");
            signalDrainDone();
        }
    }
}

Tick
SSDMem::getLatency()
{
    if (ssdptr.good() && !ssdptr.eof()) {
        std::string line;
        std::string addr;
        std::getline(ssdptr, line, ',');
        std::getline(ssdptr, addr, ',');
        std::getline(ssdptr, line, ',');
        line.erase();
        std::getline(ssdptr, line);
        if(line.length() == 0){
            DPRINTF(SimpleSSD, "No trace information found for %s\n", addr.c_str());
            return 0;   //No Latency information for current request
        }
        unsigned psLatency = std::stoul(line, 0L, 10);
        Tick ssdLatency = (Tick)(psLatency); // Ticks are in ps
        DPRINTF(SimpleSSD, "Request to %s, the latency in trace is %dns\n", addr.c_str(), ssdLatency);
        return ssdLatency;
    }
    return latency +
        (latency_var ? random_mt.random<Tick>(0, latency_var) : 0);
}

void
SSDMem::recvRespRetry()
{
    assert(retryResp);

    dequeue();
}

Port &
SSDMem::getPort(const std::string &if_name, PortID idx)
{
    if (if_name != "port") {
        return AbstractMemory::getPort(if_name, idx);
    } else {
        return port;
    }
}

DrainState
SSDMem::drain()
{
    if (!packetQueue.empty()) {
        DPRINTF(Drain, "SSDMem Queue has requests, waiting to drain\n");
        return DrainState::Draining;
    } else {
        return DrainState::Drained;
    }
}

SSDMem::MemoryPort::MemoryPort(const std::string& _name,
                                     SSDMem& _memory)
    : ResponsePort(_name, &_memory), mem(_memory)
{ }

AddrRangeList
SSDMem::MemoryPort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(mem.getAddrRange());
    return ranges;
}

Tick
SSDMem::MemoryPort::recvAtomic(PacketPtr pkt)
{
    return mem.recvAtomic(pkt);
}

Tick
SSDMem::MemoryPort::recvAtomicBackdoor(
        PacketPtr pkt, MemBackdoorPtr &_backdoor)
{
    return mem.recvAtomicBackdoor(pkt, _backdoor);
}

void
SSDMem::MemoryPort::recvFunctional(PacketPtr pkt)
{
    mem.recvFunctional(pkt);
}

bool
SSDMem::MemoryPort::recvTimingReq(PacketPtr pkt)
{
    return mem.recvTimingReq(pkt);
}

void
SSDMem::MemoryPort::recvRespRetry()
{
    mem.recvRespRetry();
}

} // namespace gem5
