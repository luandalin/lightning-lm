run_frontend_offline   ← 从这里开始，纯前端，最简
       ↓
run_loc_offline        ← 加入定位，但仍是离线
       ↓
run_loop_offline       ← 加入回环检测
       ↓
run_slam_offline       ← 完整 SLAM（前端+后端+建图）
       ↓
run_loc_online         ← 在线模式，引入实时性
       ↓
run_slam_online        ← 最复杂，所有模块 + 实时
