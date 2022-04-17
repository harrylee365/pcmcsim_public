#include "base/PCMCTypes.h"
#include "base/Component.h" 
//TODO: maybe credit management is required (bypass now)

namespace PCMCsim
{
    class MemoryControlSystem;

    class Parser : public Component
    {
      public:
        Parser(MemoryControlSystem* memsys_, std::string cfg_header);
        ~Parser( );

        void recvRequest(Packet* pkt, ncycle_t delay=1) override;
        void recvResponse(Packet* pkt, ncycle_t delay=1) override;
        bool isReady(Packet* pkt) override;

        void handle_events(ncycle_t curr_tick) override;
        virtual void expand_path(Component* c);

        ncycle_t last_rdresp;
        
        std::vector<Component*> paths;
    };
};
