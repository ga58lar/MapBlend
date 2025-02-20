#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
tag=0.0.1

docker build -f $SCRIPT_DIR/../Dockerfile -t ga58lar/mapblend:$tag $SCRIPT_DIR/..
