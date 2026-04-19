#include <iostream>
#include "SCServo.h"
#include <thread>
#include <chrono>

SMS_STS sm_st;

int main(int /*argc*/, char** /*argv*/)
{
  const char* port = "/dev/ttyACM0";
  std::cout << "串口: " << port << std::endl;

  // 打开串口
  if (!sm_st.begin(1000000, port))
  {
    std::cout << "串口打开失败！" << std::endl;
    return -1;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << "开始扫描 ID 0 ~ 20...\n" << std::endl;

  // 扫描 0 ~ 20 号电机
  for (int id = 0; id <= 20; id++)
  {
    int ret = sm_st.Ping(id);
    if (ret != -1)
    {
      std::cout << "找到电机 ID: " << ret << std::endl;
    }

    // 必须加延时，防止发太快
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  std::cout << "\n扫描完成！" << std::endl;

  sm_st.end();
  return 0;
}
