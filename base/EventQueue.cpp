#include "base/EventQueue.h"
#include "base/MemoryControlSystem.h"
#include "base/DataBlock.h"
#include "base/Packet.h"

using namespace PCMCsim;

GlobalEventQueue::GlobalEventQueue( )
:escape(false), curr_tick(0), dbg_msg(false)
{

}

GlobalEventQueue::~GlobalEventQueue( )
{

}

void GlobalEventQueue::insertEvent(ncycle_t wakeupTime, Component* component)
{
    event_q[wakeupTime].insert(component);
}

void GlobalEventQueue::handle_events( )
{
    event_queue_t::iterator eq_iter = event_q.begin( );
    for ( ; eq_iter != event_q.end( ); )
    {
        PCMC_DBG(dbg_msg, "-------------------------------------------" 
            "------------------------------------[Tick %ld]\n", eq_iter->first);

        /* Events of components are executed as declaration order */
        curr_tick = eq_iter->first;
        std::set<Component*>::iterator cp_iter = eq_iter->second.begin( );
        for ( ; cp_iter != eq_iter->second.end( ); ) 
        {
            (*cp_iter)->handle_events(curr_tick);
            cp_iter = eq_iter->second.erase(cp_iter);
        }
        eq_iter = event_q.erase(eq_iter);

        if (escape)
            break;
    }
}

void GlobalEventQueue::handle_events(ncycle_t goal_tick)
{
    event_queue_t::iterator eq_iter = event_q.begin( );
    for ( ; eq_iter != event_q.end( ); )
    {
        if (eq_iter->first>goal_tick)
            break;

        PCMC_DBG(dbg_msg, "-------------------------------------------" 
            "------------------------------------[Tick %ld]\n", eq_iter->first);

        /* Events of components are executed as declaration order */
        curr_tick = eq_iter->first;
        std::set<Component*>::iterator cp_iter = eq_iter->second.begin( );
        for ( ; cp_iter != eq_iter->second.end( ); ) 
        {
            (*cp_iter)->handle_events(curr_tick);
            cp_iter = eq_iter->second.erase(cp_iter);
        }
        eq_iter = event_q.erase(eq_iter);

        if (escape)
            break;
    }
}

void GlobalEventQueue::set_dbg_msg(bool setup)
{
    dbg_msg = setup;
}

ncycle_t GlobalEventQueue::getCurrentTick( )
{
    return curr_tick;
}

LocalEvent::LocalEvent( )
:type(TX_EVENT), component(NULL), pkt(NULL), cb_arg(NULL), cb_priority(0)
{

}

LocalEvent::~LocalEvent( )
{
}

