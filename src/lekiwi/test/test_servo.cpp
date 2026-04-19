#include <iostream>
#include "SCServo.h"
#include <thread>
#include <chrono>



SMS_STS sm_st;

int main(int argc, char** argv)
{
  // 1. 检查是否输入了 ID
  if (argc < 2)
  {
    std::cout << "用法: " << std::endl;
    std::cout << "ros2 run lekiwi test_servo 5" << std::endl;
    std::cout << "读取 5 号电机状态" << std::endl;
    return 1;
  }

  // 2. 获取命令行输入的 ID
  int target_id = std::atoi(argv[1]);
  if (target_id < 0 || target_id > 255)
  {
    std::cout << "ID 必须在 0~255 之间" << std::endl;
    return 1;
  }

  // 3. 打开串口
  const char* port = "/dev/ttyACM0";
  std::cout << "serial: " << port << std::endl;

  if (!sm_st.begin(1000000, port))
  {
    std::cout << "Failed to init sms/sts motor!" << std::endl;
    return 1;
  }

  std::cout << "\n正在读取舵机 ID = " << target_id << " ..." << std::endl;

  // 4. 读取指定舵机
  if (sm_st.FeedBack(target_id) != -1)
  {
    int pos     = sm_st.ReadPos(target_id);
    int speed   = sm_st.ReadSpeed(target_id);
    int load    = sm_st.ReadLoad(target_id);
    double volt = sm_st.ReadVoltage(target_id) / 10.0;
    int temp    = sm_st.ReadTemper(target_id);
    int moving  = sm_st.ReadMove(target_id);

    std::cout << "读取成功！" << std::endl;
    std::cout << "  ID:        " << target_id << std::endl;
    std::cout << "  位置:  " << pos << " (0~4095)" << std::endl;
    std::cout << "  速度:      " << speed << std::endl;
    std::cout << "  负载:      " << load << std::endl;
    std::cout << "  电压:    " << volt << "V" << std::endl;
    std::cout << "  温度:      " << temp << "°C" << std::endl;
    std::cout << "  运动中:    " << (moving ? "是" : "否") << std::endl;
  }
  else
  {
    std::cout << "读取失败！舵机 ID = " << target_id << " 无响应" << std::endl;
  }

  sm_st.end();
  return 0;
}
