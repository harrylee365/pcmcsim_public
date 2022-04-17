#include "base/Stats.h"

using namespace PCMCsim;

StatsContainer::StatsContainer(void* stat_ptr, std::string tname,
                      size_t tsize, std::string stat_name, std::string unit)
: name(stat_name), unit_name(unit), type_name(tname), type_size(tsize), sptr(stat_ptr)
{
}

Stats::~Stats( )
{
    std::list<StatsContainer*>::iterator s_it = slist.begin( );
    for ( ; s_it!=slist.end( ); )
    {
        delete (*s_it);
        s_it = slist.erase(s_it);
    }
}

void Stats::add_stat(void* stat_ptr, std::string type_name, 
              size_t type_size, std::string stat_name, std::string unit)
{
    StatsContainer* new_stat = new StatsContainer(stat_ptr, type_name,
                                                type_size, stat_name, unit);
    slist.push_back(new_stat);
}

void Stats::remove_stat(void* stat_ptr)
{
    std::list<StatsContainer*>::iterator s_it = slist.begin( );
    for ( ; s_it!=slist.end( ); s_it++)
    {
        if ((*s_it)->getStatPtr( )==stat_ptr)
        {
            /* 
             * Do not delete the object in the container! 
             * The master class will deal with it 
             */
            delete (*s_it);
            slist.erase(s_it);
            break;
        }
    }
}

void Stats::print(std::ostream& os)
{
    std::list<StatsContainer*>::iterator s_it = slist.begin( );
    for ( ; s_it!=slist.end( ); s_it++)
    {
        os << (*s_it)->getStatName( );
        if ((*s_it)->getUnit( )=="")
            os << " ";
        else
            os << "[" << (*s_it)->getUnit( ) << "] ";
        if ((*s_it)->getTypeName( )==typeid(int).name( ))
            os << *(static_cast<int*>((*s_it)->getStatPtr( )));
        else if ((*s_it)->getTypeName( )==typeid(float).name( ))
            os << *(static_cast<float*>((*s_it)->getStatPtr( )));
        else if ((*s_it)->getTypeName( )==typeid(double).name( ))
            os << *(static_cast<double*>((*s_it)->getStatPtr( )));
        else if ((*s_it)->getTypeName( )==typeid(uint64_t).name( ))
            os << *(static_cast<uint64_t*>((*s_it)->getStatPtr( )));
        else if ((*s_it)->getTypeName( )==typeid(std::string).name( ))
            os << *(static_cast<std::string*>((*s_it)->getStatPtr( )));
        else if ((*s_it)->getTypeName( )==typeid(ncycle_t).name( ))
            os << *(static_cast<ncycle_t*>((*s_it)->getStatPtr( )));
        else if ((*s_it)->getTypeName( )==typeid(nbyte_t).name( ))
            os << *(static_cast<nbyte_t*>((*s_it)->getStatPtr( )));
        else if ((*s_it)->getTypeName( )==typeid(id_t).name( ))
            os << *(static_cast<id_t*>((*s_it)->getStatPtr( )));
        else
            os << "UNIDENTIFIED type";
        os << std::endl;
    }
}

