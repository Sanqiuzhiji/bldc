# AS5600 编码器接入与配置说明

## 1. 文档范围

本文只解释提交 `f76ba904d5e29a559b12bdcf2552e2099bd45a2f`（添加自定义编码器 AS5600）相对于其直接父提交 `0e2cd42fa329dcae2fd85efb7c145456f3eb5155` 的改动，即：

```bash
git diff 0e2cd42fa329dcae2fd85efb7c145456f3eb5155..f76ba904d5e29a559b12bdcf2552e2099bd45a2f
```

该提交只包含 AS5600 接入：6 个文件，新增约 330 行、删除 4 行。

| 文件 | 修改性质 | 作用 |
| --- | --- | --- |
| `encoder/enc_as5600.c` | 新增 | AS5600 I2C 驱动、采样线程和故障判断 |
| `encoder/enc_as5600.h` | 新增 | 驱动对外接口 |
| `encoder/encoder.c` | 修改 | 把 AS5600 接到 Custom Encoder 框架 |
| `encoder/encoder.h` | 修改 | 引入 AS5600 头文件 |
| `encoder/encoder.mk` | 修改 | 将 AS5600 源文件加入编译 |
| `conf_general.c` | 修改 | 把 Custom Encoder 加入 FOC 传感器自动检测 |

## 2. 整体接入思路

该提交没有增加 `SENSOR_PORT_MODE_AS5600`，而是复用了已有的 `SENSOR_PORT_MODE_CUSTOM_ENCODER`，并将该模式直接实现为 AS5600：

```text
AS5600
  │ I2C，7 位地址 0x36
  ▼
enc_as5600.c 后台采样线程
  │ 缓存机械角度、状态和错误计数
  ▼
Custom Encoder 回调
  ├─ encoder_read_deg()：向 FOC 提供角度
  ├─ encoder_check_faults()：检查故障
  └─ encoder 终端命令：输出诊断信息
  ▼
FOC 编码器检测和闭环控制
```

控制线程读取缓存角度，不直接发起 I2C 事务，避免实时控制路径阻塞等待 I2C。

## 3. AS5600 驱动详细解释

### 3.1 地址和寄存器

`encoder/enc_as5600.c` 定义：

```c
#define AS5600_ADDRESS         0x36
#define AS5600_REG_STATUS      0x0B
#define AS5600_REG_RAW_ANGLE_H 0x0C
```

- `0x36`：AS5600 的 7 位 I2C 地址。
- `0x0B`：状态寄存器。
- `0x0C`、`0x0D`：12 位原始角度的高、低字节。

驱动调用 `i2cMasterTransmitTimeout()`，先发送一个寄存器地址，再读取指定长度数据。每次访问前后使用 `i2cAcquireBus()` 和 `i2cReleaseBus()`，以便与同一 I2C 外设的其他使用者互斥。

### 3.2 角度读取与换算

从 `0x0C` 开始读取两个字节后，组合 12 位原始角度：

```c
uint16_t raw = ((uint16_t)(data[0] & 0x0F) << 8) | data[1];
angle_deg = (float)raw * (360.0f / 4096.0f);
```

因此：

- 原始值范围为 `0..4095`；
- `raw = 0` 对应 0°；
- `raw = 2048` 对应 180°；
- 每个计数约为 0.0879°；
- 输出角度范围为 `[0°, 360°)`。

驱动读取的是 `RAW ANGLE`，没有使用 AS5600 内部零位和最大角度映射后的缩放角度。电机角度偏移、方向和比例由 VESC 后续编码器检测处理。

### 3.3 采样线程

初始化成功后，驱动建立名为 `AS5600` 的 ChibiOS 线程：

| 配置 | 数值 | 含义 |
| --- | ---: | --- |
| 工作区 | 384 字节 | 线程栈及管理空间 |
| 优先级 | `NORMALPRIO + 5` | 高于普通优先级 |
| 目标采样率 | 1000 Hz | 每 1 ms 尝试读取一次角度 |
| 状态检查间隔 | 100 次 | 约每 100 ms 读取一次状态 |
| I2C 超时 | 2 ms | 单次事务最大等待时间 |

线程使用 `chThdSleepUntil(next)` 按绝对时间调度，减少相对延时的周期累计漂移。如果当前循环已经错过计划时间，就以当前时间重新计算下一周期。

1 kHz 只是目标调度频率。实际有效数据刷新率还受 AS5600 工作模式、内部滤波、I2C 总线速度、RTOS 调度和其他总线使用者影响。

### 3.4 驱动状态

`AS5600_state_t` 保存：

| 字段 | 含义 |
| --- | --- |
| `angle_deg` | 最近一次成功读取并换算的角度 |
| `raw_angle` | 最近一次原始角度 |
| `status` | 最近一次状态寄存器值 |
| `valid` | 是否得到过有效角度 |
| `stop_now` | 请求采样线程退出 |
| `is_running` | 线程是否仍在运行 |
| `update_count` | 成功读取角度的次数 |
| `error_count` | 累计通信错误次数 |
| `consecutive_errors` | 连续角度读取失败次数 |
| `last_update` | 最近一次成功更新角度的时间 |

这些字段使用 `volatile`，使不同线程每次从内存访问，但这不构成多字段的原子快照。

### 3.5 初始化和关闭

`enc_as5600_init()`：

1. 清空驱动状态；
2. 调用 `hw_start_i2c()` 配置管脚并启动目标硬件定义的 I2C；
3. 要求首次角度读取成功；
4. 要求首次状态读取成功；
5. 启动 AS5600 采样线程。

首次角度或状态读取失败时，初始化会停止 I2C 并返回 `false`。读取状态成功只代表 I2C 事务完成；磁铁是否正常由故障函数判断。

`enc_as5600_deinit()`：

1. 请求线程退出；
2. 每隔 1 ms 等待线程真正结束；
3. 将数据标记为无效；
4. 调用 `hw_stop_i2c()`。

如果其他功能共享同一个 `HW_I2C_DEV`，应检查其生命周期，避免 AS5600 关闭整个 I2C 外设后影响其他设备。

## 4. 故障检测配置

### 4.1 磁铁状态

驱动检查 `0x0B` 的三个标志：

| 位 | 名称 | 含义 |
| ---: | --- | --- |
| 5 | `MAGNET_DETECTED` | 检测到磁铁 |
| 4 | `MAGNET_TOO_WEAK` | 磁场过弱 |
| 3 | `MAGNET_TOO_STRONG` | 磁场过强 |

以下任一情况会使 `enc_as5600_has_fault()` 返回 `true`：

- 尚未得到有效角度；
- 连续 5 次角度读取失败；
- 超过 10 ms 没有成功更新角度；
- 未检测到磁铁；
- 磁场过弱；
- 磁场过强。

### 4.2 VESC 故障码

AS5600 故障函数被注册为 Custom Encoder 回调。VESC 检测到异常后调用：

```c
mc_interface_fault_stop(FAULT_CODE_ENCODER_FAULT, ...);
```

因此所有 AS5600 异常都统一表现为 `FAULT_CODE_ENCODER_FAULT`，主故障码不能区分通信失败、数据超时、无磁铁和磁场强弱问题。

## 5. 接入 VESC 编码器框架

### 5.1 加入编译

`encoder/encoder.mk` 将 `encoder/enc_as5600.c` 加入 `ENCSRC`；`encoder/encoder.h` 增加：

```c
#include "enc_as5600.h"
```

### 5.2 Custom Encoder 初始化

当 `m_sensor_port_mode` 为 `SENSOR_PORT_MODE_CUSTOM_ENCODER` 时，`encoder_init()` 执行：

```c
if (!enc_as5600_init()) {
    m_encoder_type_now = ENCODER_TYPE_NONE;
    return false;
}

encoder_set_custom_callbacks(
    enc_as5600_read_deg,
    enc_as5600_has_fault,
    enc_as5600_print_info
);
m_encoder_type_now = ENCODER_TYPE_CUSTOM;
```

| 回调 | AS5600 实现 | 用途 |
| --- | --- | --- |
| 角度 | `enc_as5600_read_deg()` | 返回缓存的机械角度 |
| 故障 | `enc_as5600_has_fault()` | 检查通信、数据时效和磁场 |
| 信息 | `enc_as5600_print_info()` | 返回角度、计数和磁铁状态文本 |

反初始化时调用 `enc_as5600_deinit()`，然后把三个 Custom Encoder 回调清空，防止模式切换后继续访问旧驱动。

### 5.3 UART 与 I2C 冲突处理

进入 Custom Encoder 分支时，如果应用模式是：

- `APP_UART`
- `APP_PPM_UART`
- `APP_ADC_UART`

代码会将其改成 `APP_NONE`，立即应用并写入持久化配置。

原因是许多 VESC 硬件的通信口 UART 和 I2C 复用同一组管脚，例如：

```text
UART TX = PB10  <-> I2C SCL = PB10
UART RX = PB11  <-> I2C SDA = PB11
```

这只是常见示例，实际外设和管脚由当前硬件的 `HW_I2C_*` 宏决定。

注意：`APP_NONE` 会被保存到 Flash。切换回其他编码器后，原 UART 类应用不会自动恢复，需要手动重新选择并保存。

## 6. FOC 自动检测如何识别 AS5600

`conf_general_autodetect_apply_sensors_foc()` 在原 AS5047 检测之前增加 Custom Encoder 检测，顺序成为：

```text
Custom Encoder / AS5600 → AS5047 → Hall → Sensorless
```

在定义 `HW_HAS_DUAL_MOTORS` 的双电机硬件上，这段 Custom Encoder 自动检测不会编译。

### 6.1 检测过程

1. 临时选择 `SENSOR_PORT_MODE_CUSTOM_ENCODER` 并应用配置，从而初始化 AS5600。
2. 用 `encoder_is_configured()` 确认初始化成功。
3. 用 1000 个、间隔 1 ms 的台阶，将开环锁定电流从 0 增加到检测电流。
4. 读取起始机械角度 `phase_start`。
5. 保持检测电流，把开环电角度从 0° 扫到 179°，每步等待 5 ms。
6. 在 90° 附近读取 `phase_mid`，结束后读取 `phase_end`。
7. 计算角度变化：

   ```c
   diff     = abs(angle_difference(phase_start, phase_end));
   diff_mid = abs(angle_difference(phase_mid, phase_end));
   ```

8. 使用条件：

   ```c
   diff > 2.0 && (diff_mid - diff / 2.0) < (diff / 4)
   ```

   判断编码器是否随电机产生了足够且趋势合理的运动。

9. 条件通过后调用 `mcpwm_foc_encoder_detect()`，检测 `offset`、`ratio` 和 `inverted`。
10. 把结果写入电机配置：

    ```c
    m_sensor_port_mode   = SENSOR_PORT_MODE_CUSTOM_ENCODER;
    foc_sensor_mode      = FOC_SENSOR_MODE_ENCODER;
    foc_encoder_offset   = offset;
    foc_encoder_ratio    = ratio;
    foc_encoder_inverted = inverted;
    ```

11. 返回结果 `2`。对应注释也从“AS5147 detected successfully”改成了更通用的“Encoder detected successfully”。

如果 AS5600 初始化失败或运动条件未通过，检测流程会继续尝试 AS5047，而不是直接结束。

### 6.2 检测参数

| 配置项 | 作用 |
| --- | --- |
| `foc_encoder_offset` | 电角度与编码器机械零位之间的偏移 |
| `foc_encoder_ratio` | 机械角度到电角度的换算比例，通常与电机极对数相关 |
| `foc_encoder_inverted` | 编码器方向是否与电机电角度方向相反 |
| `foc_sensor_mode` | 设为 Encoder 后，FOC 使用编码器反馈 |
| `m_sensor_port_mode` | 设为 Custom Encoder 后，启动本提交的 AS5600 驱动 |

这些参数应通过检测获得，不建议仅凭猜测填写。

## 7. 硬件接线

本实现使用 I2C，不读取模块的模拟/PWM `OUT` 信号。

| AS5600 | 控制器 | 注意事项 |
| --- | --- | --- |
| VCC | 与模块和控制器兼容的电源 | 确认模块稳压和逻辑电平 |
| GND | GND | 必须共地 |
| SCL | `HW_I2C_SCL_PORT/PIN` | 常见为通信口 TX/PB10，仅为示例 |
| SDA | `HW_I2C_SDA_PORT/PIN` | 常见为通信口 RX/PB11，仅为示例 |
| OUT | 不接 | 本驱动不使用 |
| DIR | 按模块要求固定 | 不应悬空；软件还能检测方向反转 |

### 7.1 确认实际管脚

1. 确认固件的硬件目标和 `HW_HEADER`。
2. 打开对应 `hwconf` 头文件。
3. 查找：

   ```c
   HW_I2C_DEV
   HW_I2C_GPIO_AF
   HW_I2C_SCL_PORT
   HW_I2C_SCL_PIN
   HW_I2C_SDA_PORT
   HW_I2C_SDA_PIN
   ```

4. 根据控制器原理图把 MCU 管脚映射到实际插座针脚。
5. 检查是否与 UART、扩展模块或板载功能冲突。

不能只根据 `PB10/PB11` 示例接线，不同硬件可能使用不同管脚。

### 7.2 上拉和电平

SCL、SDA 是开漏信号，需要上拉。很多 AS5600 模块已有上拉；裸芯片或无上拉模块应把两根线分别上拉到 MCU I/O 电压，通常为 3.3 V。

还应确认：

- 模块是否把 I2C 上拉到 5 V，MCU 管脚能否承受；
- 多个设备的并联上拉是否过强；
- 总线上是否已有地址 `0x36`；
- 线缆是否过长或靠近电机相线等噪声源。

## 8. 配置与验证步骤

1. 编译并烧录包含 `f76ba904` 的固件。
2. 根据目标硬件的 `HW_I2C_*` 定义连接 AS5600。
3. 确认磁铁与芯片同轴、间距合适，磁场不过强或过弱。
4. 确认不再需要与 I2C 共用管脚的 UART 类应用。
5. 将 Motor Sensor Port Mode 选择为 `Custom Encoder`。
6. 将 FOC Sensor Mode 选择为 `Encoder`，或运行完整 FOC 传感器检测自动设置。
7. 使用适合电机的低检测电流执行编码器检测并保存结果。
8. 在空载或低风险负载下低速运行，检查方向、连续性和故障状态。
9. 逐步提高转速和负载，验证电机干扰环境下的 I2C 稳定性。

自动检测时电机会锁定并转动。测试前应固定电机和控制器、移除危险负载，并确保转子周围安全。

## 9. 诊断信息及当前缺陷

`enc_as5600_print_info()` 计划输出：

```text
AS5600 raw: 1234, angle: 108.46 deg, updates: 10000, errors: 0, magnet: detected
```

但该提交在 `encoder.c` 中写成：

```c
commands_printf("%s", m_enc_custom_print_info);
```

这里传入的是函数指针。正确写法应为：

```c
commands_printf("%s", m_enc_custom_print_info());
```

修复前不应依赖 `encoder` 终端命令输出 AS5600 信息；现有写法可能打印乱码或访问非法地址。本文只解释该提交，没有修改源代码。

## 10. 当前实现的限制

- **Custom Encoder 被固定为 AS5600**：选择该模式会无条件初始化 AS5600，其他自定义编码器需要新的选择机制。
- **共享 I2C 生命周期**：总线锁只解决同时访问；`hw_stop_i2c()` 仍可能影响同一外设上的其他设备。
- **1 kHz 不是硬实时保证**：单次 I2C 超时为 2 ms，已可能超过 1 ms 目标周期。
- **故障分类笼统**：全部映射为 `FAULT_CODE_ENCODER_FAULT`。
- **状态读取失败处理有限**：状态读取失败只增加累计错误，不增加 `consecutive_errors`；角度正常时可能持续使用旧状态。
- **自动检测是启发式判断**：需要在目标电机、极对数、传动结构和机械安装上验证。
- **UART 配置会永久改写**：冲突应用会保存为 `APP_NONE`，切换传感器后不会自动恢复。
- **多字段状态不是一致快照**：`volatile` 不等于线程同步，诊断文本可能组合到不同采样时刻的数据。

## 11. 修改总结

`f76ba904` 相对 `0e2cd42f` 完成了三层接入：

1. **器件层**：新增 AS5600 I2C 读取、角度换算、后台采样和状态检查。
2. **编码器框架层**：绑定 `SENSOR_PORT_MODE_CUSTOM_ENCODER`，注册角度、故障和诊断回调，并处理 UART/I2C 管脚冲突。
3. **FOC 配置层**：在自动检测中优先尝试 AS5600，检测运动后标定 offset、ratio 和 inverted，启用编码器闭环。

该提交形成了从硬件读取到 FOC 配置的基本完整链路，但终端诊断调用、共享 I2C 生命周期、故障分类和 Custom Encoder 可扩展性仍有改进空间。
