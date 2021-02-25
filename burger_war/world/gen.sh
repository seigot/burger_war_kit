#!/bin/bash
set -o verbose
empy burger_field.world.em > burger_field.world
empy -DGUI_CAMERA_POSE='"-0.03 0.13 2.76 0.0 1.57 2.26"' burger_field.world.em > burger_field_autotest.world
