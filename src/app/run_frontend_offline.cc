//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "core/g2p5/g2p5.h"
#include "core/lio/laser_mapping.h"
#include "ui/pangolin_window.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"

#include <opencv2/opencv.hpp>
/*
rosbag 文件
    │
    ├──→ "imu_raw" 话题 ──→ ProcessIMU() ──→ ESKF 预测
    │
    └──→ "points_raw" 话题 ──→ ProcessPointCloud2() ──→ 运动补偿
                                    │
                                    ↓
                               lio.Run()
                                    │
                    ┌───────────────┼───────────────┐
                    ↓               ↓               ↓
              点云配准        ESKF 更新        更新局部地图
                    │               │               │
                    └───────────────┼───────────────┘
                                    ↓
                              判断新关键帧？
                                    │
                          ┌─────────┴─────────┐
                          ↓                   ↓
                    是：PushKeyframe     否：跳过
                          │
                          ↓
                    G2P5 栅格地图更新
                          │
                          ↓
                    （可选）显示栅格地图

    回放结束后：
    ├──→ SaveMap()        保存点云地图
    ├──→ imwrite()        保存栅格地图图片
    ├──→ Timer::PrintAll() 打印性能统计
    └──→ ui->Quit()       关闭窗口

*/
DEFINE_string(input_bag, "", "输入数据包");
DEFINE_string(config, "./config/default.yaml", "配置文件");
DEFINE_bool(show_grid_map, false, "是否展示栅格地图");

/// 运行一个LIO前端，带可视化
int main(int argc, char** argv) {
    //~ 日志初始化
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    //~ 参数校验
    google::ParseCommandLineFlags(&argc, &argv, true);
    if (FLAGS_input_bag.empty()) {
        LOG(ERROR) << "未指定输入数据";
        return -1;
    }

    using namespace lightning;
    //~ 创建核心对象
    RosbagIO rosbag(FLAGS_input_bag);

    LaserMapping lio;
    if (!lio.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init lio";
        return -1;
    };
    //~ 创建栅格地图
    g2p5::G2P5::Options map_opt;
    map_opt.online_mode_ = false;

    g2p5::G2P5 map(map_opt);
    map.Init(FLAGS_config);
    //~ 创建可视化窗口
    auto ui = std::make_shared<ui::PangolinWindow>();
    ui->Init();
    lio.SetUI(ui);
    //~ 关键帧指针
    Keyframe::Ptr cur_kf = nullptr;
    //~ 注册回调并执行回放（核心逻辑）
    rosbag
        .AddImuHandle("imu_raw", //! dalin remark:写死的imu话题名，和default.yaml中的不一样
                      [&lio](IMUPtr imu) {
                          lio.ProcessIMU(imu);
                          return true;
                      })
        .AddPointCloud2Handle("points_raw",
                              [&](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
                                  lio.ProcessPointCloud2(cloud);
                                  lio.Run();

                                  auto kf = lio.GetKeyframe();
                                  if (cur_kf != kf) {
                                      cur_kf = kf;

                                      // pcl::io::savePCDFile("./data/" + std::to_string(cur_kf->GetID()) + ".pcd",
                                      //                      *cur_kf->GetCloud());

                                      map.PushKeyframe(cur_kf);

                                      if (FLAGS_show_grid_map) {
                                          cv::Mat image = map.GetNewestMap()->ToCV();
                                          cv::imshow("map", image);
                                          cv::waitKey(10);
                                      }
                                  }

                                  return true;
                              })
        .Go();
    //~ 收尾                          
    lio.SaveMap();
    cv::Mat image = map.GetNewestMap()->ToCV();
    cv::imwrite("./data/map.png", image);

    Timer::PrintAll();

    ui->Quit();

    LOG(INFO) << "done";

    return 0;
}