#!/usr/bin/env bash
set -euo pipefail

grep -q '^id: xgc2-gazebo-sim-scout$' .xgc2/product.yml
grep -q '^version: 0.4.9-33$' .xgc2/product.yml
grep -q '<name>gazebo_sim_scout</name>' scout/package.xml
grep -q 'ros-noetic-xgc2-scout-description (>= 0.4.10-14)' .xgc2/product.yml
test -f scout/launch/mini_description.launch
test -f scout/urdf/mini.xacro
test -f scout/rviz/navigation.rviz

if rg -n '\$\(find scout_description\)/(launch|rviz|urdf)' scout/launch scout/urdf scout/rviz; then
  echo "gazebo_sim_scout still consumes nonvisual scout_description paths" >&2
  exit 1
fi

python3 -m unittest discover -s scout/test -v
echo "Package compliance checks passed."
