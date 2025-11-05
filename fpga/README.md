# FPGA代码模块

本目录包含用于FPGA协处理器的代码，用于实现高频率步态计算和传感器数据融合。

## 文件说明

- `gait_calculator.v` - 步态计算模块，使用Verilog实现

## 功能

1. **高频率步态计算**
   - 实时计算6条腿的关节角度
   - 支持多种步态模式
   - 计算频率可达1000Hz

2. **传感器数据融合**
   - 处理IMU数据
   - 处理力传感器数据
   - 数据融合算法

3. **能量回收控制**
   - 检测足端接触力
   - 控制能量回收系统

4. **SPI通信接口**
   - 与Arduino主控制器通信
   - 接收传感器数据
   - 发送步态计算结果

## 开发环境

- Xilinx Vivado 或 Altera Quartus
- 支持的FPGA：Xilinx Artix-7, Zynq-7000 或 Altera Cyclone V

## 编译和综合

```bash
# 使用Vivado
vivado -mode batch -source build.tcl

# 或使用Quartus
quartus_sh --flow compile gait_calculator
```

## 接口说明

### 输入信号
- `clk` - 系统时钟（建议100MHz）
- `rst_n` - 复位信号（低有效）
- `spi_cs_n` - SPI片选信号
- `spi_clk` - SPI时钟
- `spi_mosi` - SPI主出从入
- `imu_data` - IMU传感器数据
- `force_data[5:0]` - 6个力传感器数据

### 输出信号
- `spi_miso` - SPI主入从出
- `gait_angles[17:0]` - 18个关节角度（6条腿 x 3关节）
- `gait_velocities[17:0]` - 18个关节速度
- `energy_recovery_enable[5:0]` - 能量回收使能
- `calc_ready` - 计算完成标志

## 通信协议

### SPI数据格式

**命令格式（Arduino -> FPGA）：**
- 字节0: 命令码
  - 0x01: 读取步态命令
  - 0x02: 能量回收控制
  - 0x03: 发送传感器数据
- 字节1-N: 数据载荷

**响应格式（FPGA -> Arduino）：**
- 字节0-35: 18个关节角度（每个2字节）
- 字节36-71: 18个关节速度（每个2字节）

## 注意事项

1. 本代码为示例实现，实际使用时需要根据具体硬件调整
2. 步态计算算法需要根据实际机器人参数优化
3. SPI通信协议可以根据需要扩展
4. 建议使用FPGA的DSP资源加速浮点运算

