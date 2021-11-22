#include "SmartSSD/emb_table.hh"


namespace gem5{
std::vector<Addr> emb_table::insertEntry(uint index){
    std::vector<Addr> assigned;

    mapping_next.emplace(index, _tail);
    //Check whether this entry has been in DRAM
    auto match_old = mapping_old.find(index);
    if(match_old != mapping_old.end())    //Entry is being updated and we need to add it to the pending queue
    {
        pending_memcpy.push_back(std::pair<Addr, Addr>( match_old->second, _tail));
        _tail += _vlen;
        _tail %= _cache_size;
        return assigned;
    }

    auto match_prev = mapping_prev.find(index);
    if(match_prev != mapping_prev.end())    //Entry is used by current forward, we can perform memcpy thanks to the speculative embedding scheme
    {
        //src addr
        assigned.push_back(match_prev->second);
        assigned.push_back(_tail);
        _tail += _vlen;
        _tail %= _cache_size;
        return assigned;
    }

    //Entry is fresh, need to read from the ssd
    assigned.push_back(_tail);
    _tail += _vlen;
    _tail %= _cache_size;
    return assigned;
}

std::map<uint64_t, Addr> emb_table::popOldIterEntries()
{
    auto poped_entries = mapping_old;
    mapping_old.clear();
    //Containers do deep copies by default!
    mapping_old = mapping_prev;
    mapping_prev = mapping_next;
    //Deep Copy so this should be safe
    mapping_next.clear();

    _old_cache_head = _prev_cache_head;
    _prev_cache_head = _next_cache_head;
    _next_cache_head = _tail;
    return poped_entries;
}

}