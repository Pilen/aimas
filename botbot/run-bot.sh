#!/bin/bash

set -e

export GOPATH="$(realpath $(dirname "$0"))"

go build
java -jar "$GOPATH/../environment/server.jar" -c "$GOPATH/botbot" -l "$GOPATH/../environment/levels/MAsimple1.lvl" &
# echo $$ >> "$GOPATH/pid"
# echo $! >> "$GOPATH/pid"
