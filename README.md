# OpenArm CAN Library

[简体中文](README.zh-CN.md) | English

A C++ library for CAN communication with OpenArm robotic hardware. This fork
provides the ZK CAN FD backend used by downstream forks in place of the
original Damiao backend. The Damiao classes remain for source compatibility.
IDs 1-6 use one multi-joint frame; arm ID 7 and gripper ID 8 both use
single-joint MIT mode.
This library is a part of [OpenArm](https://github.com/enactic/openarm/). See detailed setup guide and docs [here](https://docs.openarm.dev/software/can).


## Quick Start

### Prerequisites

- Linux with SocketCAN support
- CAN interface hardware

### 1. Install

#### Ubuntu

* 22.04 Jammy Jellyfish
* 24.04 Noble Numbat
* 26.04 Resolute Raccoon

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository -y ppa:openarm/main
sudo apt update
sudo apt install -y \
  libopenarm-can-dev \
  openarm-can-utils
```

#### AlmaLinux, CentOS, Fedora, RHEL, and Rocky Linux

1. Enable [EPEL](https://docs.fedoraproject.org/en-US/epel/). (Not required for [Fedora](https://fedoraproject.org/))
   * AlmaLinux 8 / Rocky Linux 8
     ```bash
     sudo dnf install -y epel-release
     sudo dnf config-manager --set-enabled powertools
     ```
   * AlmaLinux 9 & 10 / Rocky Linux 9 & 10
     ```bash
     sudo dnf install -y epel-release
     sudo crb enable
     ```
   * CentOS Stream 9
     ```bash
     sudo dnf config-manager --set-enabled crb
     sudo dnf install -y https://dl.fedoraproject.org/pub/epel/epel{,-next}-release-latest-9.noarch.rpm
     ```
   * CentOS Stream 10
     ```bash
     sudo dnf config-manager --set-enabled crb
     sudo dnf install -y https://dl.fedoraproject.org/pub/epel/epel-release-latest-10.noarch.rpm
     ```
   * RHEL 8 & 9 & 10
     ```bash
     releasever="$(. /etc/os-release && echo $VERSION_ID | grep -oE '^[0-9]+')"
     sudo subscription-manager repos --enable codeready-builder-for-rhel-$releasever-$(arch)-rpms
     sudo dnf install -y https://dl.fedoraproject.org/pub/epel/epel-release-latest-$releasever.noarch.rpm
     ```
2. Install the package.
   ```bash
   sudo dnf update
   sudo dnf install -y \
     openarm-can-devel \
     openarm-can-utils
   ```

### 2. Setup CAN Interface

Configure your CAN interface using the provided script:

```bash
# CAN 2.0 (default)
openarm-can-configure-socketcan can0

# CAN-FD with 5Mbps data rate
openarm-can-configure-socketcan can0 -fd
```

For ZK motors, configure the exact nominal and data rates from the V1.4
protocol before starting a program:

```bash
sudo ip link set can0 down
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up type can fd on bitrate 1000000 dbitrate 5000000
```

### 3. CLI Tool

`openarm-can-cli` provides a command-line interface for motor configuration and diagnostics.

```bash
# Configure CAN interface (default: 1Mbps nominal, 5Mbps data, CAN-FD)
openarm-can-cli -i can0 can_configure

# Configure with 1Mbps data rate (Classic CAN)
openarm-can-cli -i can0 can_configure -d 1000000 --no-fd

# Discover motors on the bus
openarm-can-cli -i can0 discover

# Monitor motor status (arm motors 1-8 by default)
openarm-can-cli -i can0 monitor

# Monitor specific motors
openarm-can-cli -i can0 monitor --id 1,2,3
```

Run `openarm-can-cli -h` for full usage.

### 4. C++ Library

```cpp
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>

openarm::can::socket::OpenArm arm("can0", true);  // CAN-FD enabled
std::vector<openarm::damiao_motor::MotorType> motor_types = {
    openarm::damiao_motor::MotorType::DM4310, openarm::damiao_motor::MotorType::DM4310};
std::vector<uint32_t> send_can_ids = {0x01, 0x02};
std::vector<uint32_t> recv_can_ids = {0x11, 0x12};

openarm.init_arm_motors(motor_types, send_can_ids, recv_can_ids);
openarm.enable_all();
```

ZK uses one 64-byte host frame for arm motors 1-6. Motor 7 and gripper motor 8
both use 8-byte single-joint MIT CAN FD frames. All feedback frames use their
motor ID:

```cpp
#include <openarm/can/socket/zk_openarm.hpp>

openarm::can::socket::ZKOpenArm arm("can0", {1, 2, 3, 4, 5, 6, 7, 8});
arm.enable_all();
arm.mit_control_all(std::vector<openarm::damiao_motor::MITParam>(
    8, {0, 0, 0, 0, 0}));
arm.refresh_all();  // Send zero MIT commands to request fresh state.
arm.recv_all(5000);
```

`set_zero_all()` performs the hardware sequence validated for ZK V1.4:
disable, set zero, enable, then send zero torque for fresh feedback. Motors
1-6 use the multi-joint `0xFC`/`0xFD`/`0xEF` commands. Motors 7 and 8 use the
single-joint extension commands `8A 01`/`8A 00`/`82`, all as CAN FD frames.

See [dev/README.md](dev/README.md) for how to build.

### 4. Python (🚧 EXPERIMENTAL - TEMPORARY 🚧)

> [!WARNING]
>
> ⚠️ **WARNING: UNSTABLE API** ⚠️
> Python bindings are currently a direct low level **temporary port**, and will change **DRASTICALLY**.
> The interface may break between versions. Use at your own risk! Discussions on the interface are welcome.

**Build & Install:**

Please ensure that you install the C++ library first, as `1. Install` or [dev/README.md](dev/README.md).

```bash
cd python

# Create and activate virtual environment (recommended)
python -m venv venv
source venv/bin/activate

pip install .
```

**Usage:**

```python
# WARNING: This API is unstable and will change!
import openarm_can as oa

arm = oa.OpenArm("can0", True)  # CAN-FD enabled
arm.init_arm_motors([oa.MotorType.DM4310], [0x01], [0x11])
arm.enable_all()
```

The ZK backend is also available from the locally built wheel:

```python
import openarm_can as oa

arm = oa.ZKOpenArm("can0", [1, 2, 3, 4, 5, 6, 7, 8])
arm.enable_all()
arm.mit_control_all([oa.MITParam(0, 0, 0, 0, 0) for _ in range(8)])
arm.refresh_all()
arm.recv_all(5000)
assert arm.last_received_count() == 8
```

The protocol document does not define the CRC-8 parameters. Byte 63 remains
zero to match the tested ZK hardware; update it only when the manufacturer
provides the polynomial, initial value, reflection, and XOR value.

### Examples

- **C++**: `examples/demo.cpp` - Complete arm control demo
- **Python**: `python/examples/example.py` - Basic Python usage

## For developers

See [dev/README.md](dev/README.md).

## Related links

- 📚 Read the [documentation](https://docs.openarm.dev/software/can/)
- 💬 Join the community on [Discord](https://discord.gg/FsZaZ4z3We)
- 📬 Contact us through <openarm@enactic.ai>

## License

Licensed under the Apache License 2.0. See `LICENSE.txt` for details.

Copyright 2025-2026 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our [Code of Conduct](CODE_OF_CONDUCT.md).
