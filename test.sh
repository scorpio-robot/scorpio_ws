#!/bin/bash
set -e

if [ -f install/setup.bash ]; then source install/setup.bash; fi
colcon test \
  --merge-install \
  --parallel-workers 10 \
  --packages-select loam_interface
colcon test-result --all --verbose
