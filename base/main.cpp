#include "base/PCMCTypes.h"
#include "TraceExec/TraceExec.h"

using namespace PCMCsim;

int main(int argc, char* argv[])
{
    TraceExec* te = new TraceExec(argc, argv);

    te->exec( );

    delete te;

    return 0;
}

