#include "base/Component.h"
#include "base/MemoryControlSystem.h"
#include "base/EventQueue.h"
#include "base/DataBlock.h"
#include "base/Packet.h"
#include "base/Stats.h"

using namespace PCMCsim;

Component::Component( )
:id(0), dbg_msg(false),ready(false), ticks_per_cycle(1), 
memsys(NULL), geq(NULL), parent(NULL), child(NULL)
{
    stats = new Stats( );
}

Component::Component(MemoryControlSystem* memsys_)
:id(0), dbg_msg(false), ready(false), ticks_per_cycle(1), 
memsys(memsys_), geq(NULL), parent(NULL), child(NULL)
{
    if (memsys) 
        geq = memsys->getGlobalEventQueue( );
    else
    {
        std::cerr << "[Component] Error! MemoryControlSystem is not valid!" << std::endl;
        exit(1);
    }
    stats = new Stats( );
}

Component::~Component( )
{
    std::multimap<ncycle_t, LocalEvent*>::iterator e_it;
    for (e_it=resp_events.begin( ); e_it!=resp_events.end( ); )
    {
        delete e_it->second;
        e_it = resp_events.erase(e_it);
    }
    
    for (e_it=cb_events.begin( ); e_it!=cb_events.end( ); )
    {
        delete e_it->second;
        e_it = cb_events.erase(e_it);
    }
    
    for (e_it=req_events.begin( ); e_it!=req_events.end( ); )
    {
        delete e_it->second;
        e_it = req_events.erase(e_it);
    }

    std::list<LocalEvent*>::iterator aw_iter;
    for (aw_iter=await_resp.begin( ); aw_iter!=await_resp.end( ); )
    {
        delete (*aw_iter);
        aw_iter = await_resp.erase(aw_iter);
    }
    
    for (aw_iter=await_cb.begin( ); aw_iter!=await_cb.end( ); )
    {
        delete (*aw_iter);
        aw_iter = await_cb.erase(aw_iter);
    }
    
    for (aw_iter=await_req.begin( ); aw_iter!=await_req.end( ); )
    {
        delete (*aw_iter);
        aw_iter = await_req.erase(aw_iter);
    }

    delete stats;
}

bool Component::isReady(Packet* /*pkt*/)
{
    return ready;
}

void Component::recvRequest(Packet* pkt, ncycle_t delay)
{
    /* Only one number of cycle is allowable in req_events */
    ncycle_t wakeup = geq->getCurrentTick( ) + delay*ticks_per_cycle;

    /* Generate a new event */
    LocalEvent* new_event = new LocalEvent( );
    new_event->type = TX_EVENT;
    new_event->pkt = pkt;
    new_event->component = this;

    /* Stack event and schedule event */
    req_events.insert(std::pair<ncycle_t, LocalEvent*>(wakeup, new_event));
    geq->insertEvent(wakeup, this);
}

void Component::recvResponse(Packet* pkt, ncycle_t delay)
{
    /* Only one number of cycle is allowable in resp_events */
    ncycle_t wakeup = geq->getCurrentTick( ) + delay*ticks_per_cycle;

    /* Generate a new event */
    LocalEvent* new_event = new LocalEvent( );
    new_event->type = TX_EVENT;
    new_event->pkt = pkt;
    new_event->component = this;

    /* Stack event and schedule event */
    resp_events.insert(std::pair<ncycle_t, LocalEvent*>(wakeup, new_event));
    geq->insertEvent(wakeup, this);
}

void Component::setParent(Component* p)
{
    parent = p;
}

void Component::setChild(Component* c)
{
    child = c;
}

std::string Component::get_cmd_str(Packet* pkt)
{
    return get_cmd_str(pkt->cmd);
}

std::string Component::get_cmd_str(cmd_t cmd)
{
    std::string rv = "UNDEF";
    switch (cmd)
    {
        case CMD_READ:
        case CMD_BRD:
        case CMD_RDC:
            rv = "RD";
            break;
        case CMD_READ_PRE:
            rv = "RDPRE";
            break;
        case CMD_WRITE:
        case CMD_BWT:
        case CMD_WTC:
        case CMD_PKT_DEL:
            rv = "WR/DELETE";
        case CMD_WACK_ID:
            rv = "WR";
            break;
        case CMD_WRITE_PRE:
            rv = "WRPRE";
            break;
        case CMD_ACT:
            rv = "ACT";
            break;
        case CMD_PRE:
            rv = "PRE";
            break;
        case CMD_REFRESH:
            rv = "REFRESH";
            break;
        case CMD_APDE:
        case CMD_FPPDE:
        case CMD_SPPDE:
            rv = "PD-ENTER";
            break;
        case CMD_PDX:
            rv = "PD-EXIT";
            break;
        case CMD_SRE:
            rv = "SR-ENTER";
            break;
        case CMD_SRX:
            rv = "SR-EXIT";
            break;
        default:
            assert(0);
            break;
    }
    return rv;
}

void Component::prepare_events(ncycle_t curr_tick)
{
    std::multimap<ncycle_t, LocalEvent*>::iterator e_it;
    for (e_it=resp_events.begin( ); e_it!=resp_events.end( ); e_it++)
    {
        if (e_it->first==curr_tick)
        {
            //std::cout << "CHECK2 " << e_it->second->pkt << std::endl;
            await_resp.push_back(e_it->second);
        }
        else 
        {
            assert(e_it->first > geq->getCurrentTick( ));
            break;
        }
    }
    resp_events.erase(curr_tick);

    for (e_it=cb_events.begin( ); e_it!=cb_events.end( ); e_it++)
    {
        if (e_it->first == curr_tick)
        {
            /* Only get current cycle call backs XXX*/
            std::list<LocalEvent*>::iterator aw_iter = await_cb.begin( );
            for ( ; aw_iter!=await_cb.end( ); aw_iter++)
            {
                if (e_it->second->cb_priority > (*aw_iter)->cb_priority)
                {
                    await_cb.insert(aw_iter, e_it->second);
                    break;
                }
            }
            if( aw_iter == await_cb.end( ) )
                await_cb.insert(aw_iter, e_it->second);
        }
        else 
        {
            assert(e_it->first > geq->getCurrentTick( ));
            break;
        }
    }
    cb_events.erase(curr_tick);

    for (e_it=req_events.begin( ); e_it!=req_events.end( ); e_it++)
    {
        if (e_it->first==curr_tick)
        {
            //std::cout << "CHECK2 " << e_it->second->pkt << std::endl;
            await_req.push_back(e_it->second);
        }
        else 
        {
            assert(e_it->first > geq->getCurrentTick( ));
            break;
        }
    }
    req_events.erase(curr_tick);
}

void Component::handle_await_callbacks( )
{
    std::list<LocalEvent*>::iterator iter = await_cb.begin( );
    for ( ; iter != await_cb.end( ); )
    {
        CallbackPtr cb = (*iter)->cb_method;
        (this->*cb)((*iter)->cb_arg);

        /* Free event */
        delete (*iter);
        iter = await_cb.erase(iter);
    }
}

void Component::registerCallback(CallbackPtr cb, ncycle_t delay,
                                 int priority, void* cb_arg)
{
    /* Generate a new event directly */
    LocalEvent* new_event = new LocalEvent( );
    new_event->type = CB_EVENT;
    new_event->component = this;
    new_event->cb_method = cb;
    new_event->cb_priority = priority;
    if (cb_arg!=NULL)
        new_event->cb_arg = cb_arg;

    /* Stack event and schedule event */
    ncycle_t wakeup = geq->getCurrentTick( ) + delay*ticks_per_cycle;
    cb_events.insert(std::pair<ncycle_t, LocalEvent*>(wakeup, new_event));
    geq->insertEvent(wakeup, this);
}

void Component::registerCallbackAt(CallbackPtr cb, ncycle_t wakeup,
                                 int priority, void* cb_arg)
{
    assert(geq->getCurrentTick( ) < wakeup);

    /* Generate a new event directly */
    LocalEvent* new_event = new LocalEvent( );
    new_event->type = CB_EVENT;
    new_event->component = this;
    new_event->cb_method = cb;
    new_event->cb_priority = priority;
    if (cb_arg!=NULL)
        new_event->cb_arg = cb_arg;

    /* Stack event and schedule event */
    cb_events.insert(std::pair<ncycle_t, LocalEvent*>(wakeup, new_event));
    geq->insertEvent(wakeup, this);
}

ncycle_t Component::getTicksPerCycle( )
{
    return ticks_per_cycle;
}

GlobalEventQueue* Component::getGlobalEventQueue( )
{
    return geq;
}

void Component::print_stats(std::ostream& os)
{
    stats->print(os);
}

