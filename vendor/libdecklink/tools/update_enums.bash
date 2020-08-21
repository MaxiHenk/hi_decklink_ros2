#!/bin/bash
#
# This script will re-generate the enums using the specification in
# `enum_config.yaml`.

curdir = $(pwd)

echo -e "Entering tool directory ..."
cd $(dirname $0)

if [[ ! -d "mkenum/" ]]; then
    echo -e "Tool \`mkenum\` not found. Downloading now ..."
    git clone https://github.com/comkieffer/mkenum.git
fi

echo -e "Re-generating enumerations ..."
mkenum/mkenum/mkenum.py enum_config.yaml

cd ${curdir}
