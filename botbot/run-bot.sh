#!/bin/bash

set -e

export GOPATH="$(realpath $(dirname "$0"))"

echo "Building..."
go build

rm -f outpipe
mkfifo outpipe
echo "Starting..."
java -jar "$GOPATH/../environment/server.jar" -c "$GOPATH/botbot" -l "$GOPATH/../environment/levels/MAsimple1.lvl" > outpipe &
javapid=$!
echo "Running..."
while read line < outpipe
do
    echo "${line}"
    if [[ "${line}" == *"* Runner completed"* ]]
    then
        kill "$javapid"
    fi
done
