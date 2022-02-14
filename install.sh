#!/usr/bin/env bash

# we need bash 4 for associative arrays
if [ "${BASH_VERSION%%[^0-9]*}" -lt "4" ]; then
  echo "BASH VERSION < 4: ${BASH_VERSION}" >&2
  exit 1
fi

# Install DYNAMIXELShield package
git clone --recursive https://github.com/ROBOTIS-GIT/DynamixelShield.git --branch $1 --single-branch

