# Livox costom pointcloud format.
//! offset_time是 uint32 纳秒：最大 4.29 s 会回绕（当前 100 ms 帧无碍，多帧累积时要注意）。
uint32 offset_time      # offset time relative to the base time 
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8 reflectivity      # reflectivity, 0~255
uint8 tag               # livox tag                     dalin remark:未被使用
uint8 line              # laser number in lidar         dalin remark:未被使用

