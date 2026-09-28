#!/bin/bash

set -x

for d in $(ls -1 ./examples); do
    if [ -f "./examples/$d/Cargo.toml" ]; then
        echo "Building $d ..."
        (cd ./examples/$d && cargo build --release)
    fi
done
