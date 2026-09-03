# OpenArm CAN 库

简体中文 | [English](README.md)

这是 OpenArm 的底层 CAN 通信库。本分支在保留原有达妙接口以兼容已有代码的同时，新增并供下游项目使用 ZK V1.4 CAN FD 协议。

## ZK 通信约定

- 电机 1-6：通过一帧 64 字节 CAN FD 多电机报文控制。
- 电机 7：通过单电机 MIT 模式报文控制。
- 电机 8：作为夹爪，使用与电机 7 相同的单电机 MIT 模式报文。
- 标称速率：1 Mbps。
- 数据速率：5 Mbps。
- 当前协议资料没有给出 CRC-8 的完整参数，因此第 63 字节保持为 0。

## 环境要求

- Linux 与 SocketCAN
- 支持 CAN FD 的 CAN 适配器
- CMake 3.22 或更高版本
- 支持 C++17 的编译器

## 配置 CAN FD

```bash
sudo ip link set can0 down
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up type can fd on bitrate 1000000 dbitrate 5000000
ip -details link show can0
```

如果接口不是 `can0`，请将以上命令和代码中的接口名一并替换。

## 构建与安装 C++ 库

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
sudo cmake --install build
```

## C++ 使用示例

```cpp
#include <openarm/can/socket/zk_openarm.hpp>
#include <openarm/damiao_motor/dm_motor_control.hpp>

#include <vector>

int main() {
    using openarm::damiao_motor::MITParam;

    openarm::can::socket::ZKOpenArm arm(
        "can0", {1, 2, 3, 4, 5, 6, 7, 8});

    arm.enable_all();
    arm.mit_control_all(std::vector<MITParam>(8, MITParam{0, 0, 0, 0, 0}));
    arm.refresh_all();
    arm.recv_all(5000);                       // 超时单位：微秒
    arm.disable_all();
}
```

`MITParam` 的字段顺序是 `kp`、`kd`、目标位置、目标速度、前馈力矩。它目前作为公共命令数据结构复用，不表示底层仍在发送达妙协议。

## Python 安装与使用

Python 接口仍是实验接口。先安装 C++ 库，再执行：

```bash
cd python
python3 -m pip install .
```

```python
import openarm_can as oa

arm = oa.ZKOpenArm("can0", [1, 2, 3, 4, 5, 6, 7, 8])
arm.enable_all()
arm.mit_control_all([oa.MITParam(0, 0, 0, 0, 0) for _ in range(8)])
arm.refresh_all()
arm.recv_all(5000)

if arm.last_received_count() != 8:
    raise RuntimeError("没有收到全部 8 个电机的反馈")

arm.disable_all()
```

## 常用接口

- `enable_all()`：使能已配置电机。
- `disable_all()`：停止命令并失能电机。
- `set_zero_all()`：按 ZK V1.4 时序执行失能、置零、使能和状态刷新。
- `mit_control_one()` / `mit_control_all()`：控制机械臂电机。
- `refresh_all()`：发送零 MIT 命令并请求最新反馈。
- `recv_all(timeout_us)`：接收反馈。
- `get_motors()`：读取位置、速度、力矩、温度和故障状态。

## 安全提示

首次连接真机时应架空机械臂或解除负载，先发送零力矩命令并检查所有电机 ID、方向和反馈值。`set_zero_all()` 会修改电机零点，仅应在机械臂处于已确认的机械零位时调用。

## 协议兼容性

旧的 `OpenArm`、`MotorType::DMxxxx` 和达妙控制类仍保留，供旧代码使用。ZK 项目应显式创建 `ZKOpenArm`；ZK 初始化只使用电机 ID，不使用达妙电机型号和收发 ID 对。

## 许可证

本项目使用 Apache License 2.0，详见 [LICENSE.txt](LICENSE.txt)。
