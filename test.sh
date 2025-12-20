#!/bin/bash
set -e

if [ -f install/setup.bash ]; then source install/setup.bash; fi
colcon test \
  --merge-install \
  --symlink-install \
  --parallel-workers 10
colcon test-result --all --verbose
