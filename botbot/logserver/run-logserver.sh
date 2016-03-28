#!/bin/bash

set -e

export GOPATH="$(realpath $(dirname "$0"))"

go build

./logserver
