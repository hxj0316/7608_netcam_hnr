#!/bin/sh
# author:hxj

input=$1


LinerFun(){
        echo "hnr_liner mode starting"
        export LD_LIBRARY_PATH=:/sharefs/tmp-used/lib/code/npu
        export ASCEND_AICPU_KERNEL_PATH=/sharefs/tmp-used/lib/code/npu
        export LD_LIBRARY_PATH=:/sharefs/tmp-used/lib/code/lib
        export LD_LIBRARY_PATH=:/sharefs/tmp-used/lib/code/lib_1
        export LD_LIBRARY_PATH=:/sharefs/tmp-used/lib/lib_ff
        export LD_LIBRARY_PATH=:/sharefs/bmp/lib
        /sharefs/08b10_hnr_liner &
        sleep 10
        /sharefs/os08b10_scene_auto /sharefs/param/sensor_os08b10_hnr 0 &
        echo "hnr_liner start OK"
}
WdrFun(){
        echo "hnr_wdr mode starting"
        export LD_LIBRARY_PATH=:/sharefs/tmp-used/lib/code/npu
        export ASCEND_AICPU_KERNEL_PATH=/sharefs/tmp-used/lib/code/npu
        export LD_LIBRARY_PATH=:/sharefs/tmp-used/lib/code/lib
        export LD_LIBRARY_PATH=:/sharefs/tmp-used/lib/code/lib_1
        export LD_LIBRARY_PATH=:/sharefs/tmp-used/lib/lib_ff
        export LD_LIBRARY_PATH=:/sharefs/bmp/lib
        /sharefs/08b10_hnr_wdr &
        sleep 10
        /sharefs/os08b10_scene_auto /sharefs/param/sensor_os08b10_hnr 1 &
        echo "hnr_wdr start OK"


