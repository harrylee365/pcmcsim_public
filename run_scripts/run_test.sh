#!/bin/bash

# FIXME
RUN_PATH="(YOUR-PATH)/pcmcsim_public/"
RUN_BIN=$RUN_PATH"/pcmcsim_run.fast"

BENCH_ROOT=$RUN_PATH"/test_trace/"
BENCH_EXT=".input"
BENCH_NAME="test"

CONFIG_TYPE='pcmcsim_base_public'

# Example script
CONFIG_ROOT=$RUN_PATH'/configs/'
LOG_PATH=$RUN_PATH'/trace_logs/'

cd $RUN_PATH

CONFIG_PATH=$CONFIG_ROOT'/'$CONFIG_TYPE'.cfg'

BENCH_PATH=$BENCH_ROOT'/'$BENCH_NAME''$BENCH_EXT
LOG_FILE_PATH=$LOG_PATH'/'$CONFIG_TYPE'_'$BENCH_NAME'.log'

echo "$RUN_BIN -c $CONFIG_PATH -i $BENCH_PATH > $LOG_FILE_PATH"
$RUN_BIN -c $CONFIG_PATH -i $BENCH_PATH > $LOG_FILE_PATH

