# BMI088 IMU 集成总结

## 完成的集成步骤

### 1. 复制 BMI088 驱动文件 ✓
- **文件位置**：`new_algorithm/Firmware/Component/`
- **文件**：
  - `bmi088_probe.h` - 驱动头文件
  - `bmi088_probe.c` - 驱动实现

### 2. 硬件接口配置 ✓

#### GPIO 配置 (`Board/Core/Src/gpio.c`)
- **PB1**：陀螺仪 CS 引脚（GPIOB_PIN_1）
- **PB9**：加速度计 CS 引脚（GPIOB_PIN_9）
- 配置为推挽输出（GPIO_MODE_OUTPUT_PP）
- 使用上拉电阻（GPIO_PULLUP）

#### SPI2 配置 (`Board/Core/Src/spi.c` & `Board/Core/Inc/spi.h`)
- **模式**：主模式（SPI_MODE_MASTER）
- **数据宽度**：8 位（SPI_DATASIZE_8BIT）
- **时钟极性**：高（SPI_POLARITY_HIGH）
- **时钟相位**：第二个边沿（SPI_PHASE_2EDGE）
- **NSS**：软件控制（SPI_NSS_SOFT）
- **波特率分频**：8（SPI_BAUDRATEPRESCALER_8）
- **GPIO 脚位**：
  - PB10 - 时钟 (SCK)
  - PB14 - MISO
  - PB15 - MOSI

### 3. IMU 驱动接口修改 ✓

#### IMU 类扩展 (`Component/imu.hpp` & `Component/imu.cpp`)
- 添加 `bmi088_raw_data_t` 结构体
- 添加 `decode_bmi088()` 方法用于 BMI088 数据转换
- 转换因子：
  - 加速度：LSB × (24.0 × 9.8 / 32768) m/s²
  - 角速度：LSB × (2000 / 32768) DPS

### 4. IMU 任务修改 ✓

#### IMU Rx 任务 (`Task/imu_task.cpp`)
从 UART 接收改为 SPI 读取：
- 包含 `bmi088_probe.h` 驱动头文件
- 在任务启动时初始化 BMI088：
  - 读取芯片 ID 验证
  - 调用 `bmi088_init_minimal()` 初始化
- 主循环使用 `bmi088_read_raw()` 直接读取传感器数据
- 使用 `imu.decode_bmi088()` 转换数据
- 更新调试变量和信号

#### 启动流程修改 (`Board/board.cpp`)
- 添加 `MX_SPI2_Init()` 调用用于初始化 SPI2
- 注释掉 UART4 IMU 数据接收代码
- 保留其他外设初始化

### 5. 编译配置 ✓

#### CMakeLists.txt 自动包含
- `Component/` 目录下的 `.c` 和 `.cpp` 文件自动被 glob 收集
- `bmi088_probe.c` 和 `bmi088_probe.h` 自动包含

## 硬件通信流程

```
BMI088 (加速度计 + 陀螺仪)
    ↓
SPI2 总线 (PB10/PB14/PB15)
    ↓
STM32F405 主控
    ├── PB9：加速度计 CS（低电平选中）
    └── PB1：陀螺仪 CS（低电平选中）
    ↓
bmi088_read_raw() [原始数据]
    ↓
IMU::decode_bmi088() [转换为标准格式]
    ↓
Robot 运动控制系统
    ↓
SPI1 → CM4
```

## 关键初始化顺序

1. `HAL_Init()` - HAL 初始化
2. `SystemClock_Config()` - 系统时钟配置
3. `MX_GPIO_Init()` - GPIO 初始化（包括 CS 引脚）
4. `MX_SPI2_Init()` - SPI2 初始化
5. `HAL_Delay(50)` - 等待 BMI088 上电
6. `bmi088_probe_ids()` - 读取芯片 ID 验证
7. `bmi088_init_minimal()` - BMI088 初始化
8. `imu_task` 主循环 - 周期性读取数据

## 数据格式

### BMI088 原始数据
```c
struct bmi088_raw_data_t {
    int16_t x;  // 加速度或角速度的 X 轴分量
    int16_t y;  // Y 轴分量
    int16_t z;  // Z 轴分量
};
```

### IMU 输出格式（9 维向量）
```cpp
float imu_frame[9] = {
    ax, ay, az,           // 加速度 (m/s²)
    gx, gy, gz,           // 角速度 (deg/s)
    roll, pitch, yaw      // 欧拉角 (deg)
};
```

## 性能参数

| 项目 | 值 | 单位 |
|------|-----|------|
| 加速度计更新率 | 1600 | Hz |
| 陀螺仪更新率 | 2000 | Hz |
| IMU 任务更新率 | ~200 | Hz |
| SPI 时钟 | F_CPU/8 | ≈ 10.5 | MHz |

## 验证清单

- [x] BMI088 驱动文件复制完成
- [x] GPIO CS 引脚配置验证
- [x] SPI2 主模式配置完成
- [x] IMU 类 decode_bmi088 方法实现
- [x] imu_task 集成 BMI088 读取
- [x] board.cpp 调用 MX_SPI2_Init
- [x] CMakeLists.txt 自动包含新文件
- [x] 删除/注释 UART4 IMU 接收代码
- [x] 编译检查（头文件依赖验证）

## 可能需要的后续调整

1. **角度融合**：BMI088 不提供原生的欧拉角，需要通过融合算法计算
   - 当前代码设置欧拉角为 0（需要实现融合算法）
   
2. **温度补偿**：考虑 BMI088 的温度特性（如果需要高精度）

3. **校准**：建议在实际使用前进行以下校准：
   - 加速度计零点偏差校准
   - 陀螺仪零偏校准
   - 交叉轴灵敏度校准

4. **可靠性检测**：
   - 监控 bmi088_init_ok 状态
   - 定期检查芯片 ID
   - 添加 CRC/校验机制
