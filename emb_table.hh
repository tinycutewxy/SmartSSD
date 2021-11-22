#ifndef __EMB_TABLE_HH__
#define __EMB_TABLE_HH__

#include <sys/types.h>
#include <base/types.hh>

#include <vector>
#include <map>

namespace gem5
{

    //The class to maintain the current status of the entries inside the DRAM cache
    class emb_table
    {
    private:
        uint64_t _size;  //Number of entries in the table
        uint _vlen;      //Length of each embedding vector in Bytes
        Addr _base_addr; //Base address of the table
        Addr _base_cache_addr;
        uint64_t _cache_size;  //Cache size in bytes
        Addr _old_cache_head;  //The head index for the entries being updated (n)
        Addr _prev_cache_head; //The head index for the entries being used for forward (n+1)
        Addr _next_cache_head; //The head index for the entries being prefetched (n+2)
        Addr _tail;            //Tail of the current queue

        std::map<uint64_t, Addr> mapping_old;
        std::map<uint64_t, Addr> mapping_prev;
        std::map<uint64_t, Addr> mapping_next;

        std::vector<std::pair<Addr, Addr> > pending_memcpy; //Pending copies, wait for the finish of the update

    public:
        uint table_id;
        emb_table(uint64_t size, uint vlen, uint64_t cache_size,
                  Addr base_addr, Addr cache_addr, uint table_id) : _size(size), _vlen(vlen), _base_addr(base_addr),
                                                     _base_cache_addr(cache_addr), _cache_size(cache_size), 
                                                     _old_cache_head(0), _prev_cache_head(0),
                                                     _next_cache_head(0), _tail(0), table_id(table_id)
        {
        }

        void setSSDBase(Addr base){
            _base_addr = base;
        }

        void setDRAMBase(Addr base){
            _base_cache_addr = base;
        }

        std::map<uint64_t, Addr>& getBackwardEntries()
        {
            return mapping_old;   
        }

        std::map<uint64_t, Addr>& getForwardEntries()
        {
            return mapping_prev;   
        }

        Addr getAddr(uint index)
        {
            assert(index < _size); //Index should be within the range  of the table
            return (Addr)((index * _vlen) + _base_addr);
        }

        Addr getCacheAddr(uint index)
        {
            auto addr = mapping_prev.find(index);
            if(addr == mapping_prev.end())      //Fix for the first iter
            {
                addr = mapping_next.find(index);
            }

            return addr->second;
        }

        Addr getCurrentIterHead() const
        {
            return _prev_cache_head;
        }

        //Get all the pending transactions due to the weight update
        //This function should be called before the pop up of the old entries
        std::vector<std::pair<Addr, Addr> > getPendingMemcopy()
        {
            std::vector<std::pair<Addr, Addr> > pending = pending_memcpy;
            pending_memcpy.clear(); //Clean the pending queue
            return pending;
        }

        //Pop the entries from the previous iteration
        //This function is called when the prefetch of the current iteration is done.
        std::map<uint64_t, Addr> popOldIterEntries();

        //Insert Entry to the cache
        //Using a circular queue for this
        //If we need to read and write -> return 2 values (src, dst)
        //If we need to read from ssd -> return 1 value (dst)
        //If the corresponding entry is being updated -> return 0 value, using pending queue to finish the transaction
        std::vector<Addr> insertEntry(uint index);
    };

}

#endif