# DWMAC ethernet driver
dwmac ethernet driver in Rust on board `StarFive VisionFive 2`.

## Quick Start

For instance, [dwmac ethernet driver in Rust on StarryOS](https://github.com/elliott10/arceos/commit/2360a3dce27592789b86168e43f0126a22befaf8)

* Initialize ethernet driver
```
let mut dwmac_device = DwmacNic::<DwmacHalImpl>::init(base_ptr, mmio_size).unwrap();
```

* Sending network packets
```
dwmac_device.transmit(tx_buf.packet());
```

* Receiving network packets
```
let recv_packets = dwmac_device.receive();

```

### Build

```
cargo build --target=riscv64gc-unknown-none-elf
```

## About ethernet
* MAC: "starfive,dwmac" "snps,dwmac-5.10a"
* PHY: Motorcomm YT8531C

---

## JH7110 GMAC 时钟配置流程

### 1. 时钟控制器概述

JH7110 SoC 使用 `starfive,jh7110-clkgen` 时钟控制器，包含三个时钟域：

| 时钟域 | 基地址 | 说明 |
|--------|--------|------|
| SYS (SYSCRG) | `0x13020000` | 系统时钟 |
| STG (STGCRG) | `0x10230000` | Storage 时钟 |
| AON (AONCRG) | `0x17000000` | Always-On 时钟 |

### 2. GMAC1 时钟配置

根据设备树 `ethernet@16040000` 节点，GMAC1 使用以下时钟：

```
clock-names = "gtx", "tx", "ptp_ref", "stmmaceth", "pclk", "gtxc", "rmii_rtx";
clocks = <&clkgen 100>, <&clkgen 105>, <&clkgen 102>, <&clkgen 97>, 
         <&clkgen 98>, <&clkgen 107>, <&clkgen 101>;
```

| 时钟名 | Clock ID | 偏移地址 | 说明 |
|--------|----------|----------|------|
| gtx | 100 (0x64) | 0x190 | GMAC1 GTX Clock (分频器) |
| tx | 105 (0x69) | 0x1A4 | GMAC5 TX Clock (MUX+Gate) |
| ptp_ref | 102 (0x66) | 0x198 | PTP 参考时钟 |
| stmmaceth | 97 (0x61) | 0x184 | STMMAC 以太网时钟 |
| pclk | 98 (0x62) | 0x188 | APB 时钟 |
| gtxc | 107 (0x6B) | 0x1AC | GTX 控制时钟 |
| rmii_rtx | 101 (0x65) | 0x194 | RMII RTX 时钟 |

> 时钟寄存器地址 = SYSCRG 基地址 + Clock ID × 4

### 3. TX 时钟树结构

```
┌─────────────────────────────────────────────────────────────┐
│                    JH7110 GMAC1 TX Clock Tree               │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   pll0_out (1000 MHz)                                       │
│        │                                                    │
│        ▼                                                    │
│   ┌────────────────────────────┐                            │
│   │   gmacusb_root (1000 MHz)  │  ◄── fix_factor(1:1)       │
│   │   父时钟: pll0_out         │      不分频，直接透传      │
│   │   用途: GMAC + USB 根时钟  │                            │
│   └────────────┬───────────────┘                            │
│                │                                            │
│      ┌─────────┴─────────┬──────────────┐                   │
│      ▼                   ▼              ▼                   │
│  gmac_src           usb_125m      其他外设...               │
│      │                                                      │
│      ▼                                                      │
│   ┌────────────────────────────┐                            │
│   │   gmac1_gtxclk (clk 100)   │  ◄── 分频器 (Divider)      │
│   │   reg: 0x13020190          │      父时钟: gmacusb_root  │
│   │   div = 8/40/400           │                            │
│   └────────────┬───────────────┘                            │
│                │                                            │
│                ▼                                            │
│   ┌────────────────────────────┐                            │
│   │   gmac5_clk_tx (clk 105)   │  ◄── MUX + Gate            │
│   │   reg: 0x130201A4          │                            │
│   │   mux = 0 (gmac1_gtxclk)   │                            │
│   └────────────┬───────────────┘                            │
│                │                                            │
│                ▼                                            │
│           GMAC1 MAC                                         │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

> **gmacusb_root 说明**：这是一个"固定分频"时钟 (`fix_factor`)，从 `pll0_out` (1GHz) 以 1:1 比例派生，
> 作为 GMAC 和 USB 相关外设的根时钟源。由于分频比是 1:1，所以频率与 `pll0_out` 相同，都是 1000MHz。

### 4. 时钟寄存器格式

每个时钟寄存器是 32 位，格式如下：

```
┌──────┬──────────┬──────────────────────────┐
│ Bit  │  字段    │  说明                    │
├──────┼──────────┼──────────────────────────┤
│  31  │ ENABLE   │ 时钟使能位 (1=使能)      │
│29:24 │ MUX      │ 时钟源选择 (组合时钟用)  │
│ 23:0 │ DIV      │ 分频比 (ONE_BASED)       │
└──────┴──────────┴──────────────────────────┘
```

### 5. TX 时钟频率与 PHY 速度对应关系

| PHY 速度 | 所需 TX 时钟 | 分频比 | 计算方式 |
|----------|--------------|--------|----------|
| 1000 Mbps | 125 MHz | 8 | 1000MHz ÷ 125MHz = 8 |
| 100 Mbps | 25 MHz | 40 | 1000MHz ÷ 25MHz = 40 |
| 10 Mbps | 2.5 MHz | 400 | 1000MHz ÷ 2.5MHz = 400 |

### 6. 时钟配置代码流程

时钟配置在 PHY 自动协商完成后执行，对应 U-Boot 的 `eqos_set_tx_clk_speed_jh7110()` 函数：

```rust
// 位于 src/phy.rs

pub fn jh7110_set_tx_clk_speed(speed_mbps: u32) -> Result<(), &'static str> {
    // 1. 根据 PHY 协商速度确定分频比
    let divider = match speed_mbps {
        1000 => 8,    // 1000MHz / 125MHz = 8
        100 => 40,    // 1000MHz / 25MHz = 40
        10 => 400,    // 1000MHz / 2.5MHz = 400
        _ => return Err("Invalid speed"),
    };

    // 2. 配置 gmac1_gtxclk 分频器 (0x13020190)
    //    - 清除 bit[23:0] 分频比
    //    - 设置 bit[31] 使能位
    //    - 设置新分频比
    let gtxclk_addr = 0x13020000 + 0x190;  // SYSCRG + offset
    let gtxclk_val = (1 << 31) | divider;   // ENABLE | DIV
    write_volatile(gtxclk_addr, gtxclk_val);

    // 3. 配置 gmac5_clk_tx MUX+Gate (0x130201A4)
    //    - 清除 bit[29:24] MUX (选择 gmac1_gtxclk)
    //    - 设置 bit[31] 使能位
    let clk_tx_addr = 0x13020000 + 0x1A4;
    let clk_tx_val = (1 << 31);  // ENABLE, MUX=0
    write_volatile(clk_tx_addr, clk_tx_val);

    Ok(())
}
```

### 7. 完整初始化流程

```
┌────────────────────────────────────────────────────────────────┐
│                  GMAC + PHY 初始化流程                         │
├────────────────────────────────────────────────────────────────┤
│                                                                │
│  1. 平台初始化                                                 │
│     ├── 配置系统时钟 (SYSCRG)                                  │
│     ├── 解除 GMAC 复位                                         │
│     └── 配置 GMAC RGMII 接口                                   │
│                                                                │
│  2. MAC 初始化                                                 │
│     ├── DMA 软复位                                             │
│     ├── 配置 DMA 描述符环                                      │
│     └── 配置 MAC 基本参数                                      │
│                                                                │
│  3. PHY 初始化 (phy::init_phy)                                 │
│     ├── PHY 软复位 (BMCR.RESET)                                │
│     ├── 验证 PHY ID (YT8531: 0x4f51e91b)                       │
│     ├── 配置 YT8531 扩展寄存器                                 │
│     │   ├── 0xa012: SYNCE_CFG                                  │
│     │   ├── 0xa001: CHIP_CONFIG                                │
│     │   ├── 0xa003: RGMII_CONFIG1 (延迟配置)                   │
│     │   └── 0xa010: CLK_TX_INVERT (时钟反转)                   │
│     ├── 配置自动协商 (genphy_config_aneg)                      │
│     │   ├── 设置 MII_ADVERTISE (10/100Mbps 能力)               │
│     │   └── 设置 MII_CTRL1000 (1000Mbps 能力)                  │
│     ├── 等待链路建立                                           │
│     ├── 等待自动协商完成                                       │
│     └── 解析 PHY 状态 (速度/双工)                              │
│                                                                │
│  4. MAC 链路调整 (phy::adjust_link)                            │
│     ├── 根据 PHY 速度设置 MAC_CONFIG                           │
│     │   ├── PS (Port Select): 10/100M=1, 1000M=0               │
│     │   ├── FES (Fast Ethernet): 100M=1, 10M/1000M=0           │
│     │   └── DM (Duplex Mode): Full=1, Half=0                   │
│     └── 设置 TX 时钟频率 ◄───────────────────────────────┐     │
│                                                          │     │
│  5. TX 时钟配置 (jh7110_set_tx_clk_speed) ◄──────────────┘     │
│     ├── 计算分频比 (8/40/400)                                  │
│     ├── 配置 gmac1_gtxclk (0x13020190)                         │
│     └── 配置 gmac5_clk_tx (0x130201A4)                         │
│                                                                │
│  6. 启用 MAC 收发                                              │
│     ├── 启用 TX/RX                                             │
│     └── 启用 DMA                                               │
│                                                                │
└────────────────────────────────────────────────────────────────┘
```

### 8. 设备树参考

GMAC1 设备树节点 (`ethernet@16040000`)：

```dts
ethernet@16040000 {
    compatible = "starfive,dwmac", "snps,dwmac-5.10a";
    reg = <0x00 0x16040000 0x00 0x10000>;
    
    /* 时钟配置 */
    clock-names = "gtx", "tx", "ptp_ref", "stmmaceth", "pclk", "gtxc", "rmii_rtx";
    clocks = <&clkgen 100>, <&clkgen 105>, <&clkgen 102>, 
             <&clkgen 97>, <&clkgen 98>, <&clkgen 107>, <&clkgen 101>;
    
    /* 复位配置 */
    resets = <&rstgen 67>, <&rstgen 66>;
    reset-names = "ahb", "stmmaceth";
    
    /* PHY 模式 */
    phy-mode = "rgmii-id";
    
    /* PHY 子节点 (YT8531) */
    ethernet-phy@1 {
        tx_delay_sel = <0x00>;
        rx_delay_sel = <0x02>;
        tx_inverted_1000 = <0x00>;
        tx_inverted_100 = <0x01>;
        tx_inverted_10 = <0x01>;
    };
};
```

### 9. 常见问题

#### Q: 网卡初始化后无法收发包？
A: 检查以下几点：
1. TX 时钟是否根据 PHY 协商速度正确设置
2. MAC CONFIG 寄存器的 PS/FES 位是否与 PHY 速度匹配
3. PHY 是否完成自动协商

#### Q: 如何确认时钟配置正确？
A: 读取时钟寄存器并验证：
```rust
// gmac1_gtxclk (0x13020190)
// 1000Mbps: 应该是 0x80000008 (使能 + div=8)
// 100Mbps:  应该是 0x80000028 (使能 + div=40)
// 10Mbps:   应该是 0x80000190 (使能 + div=400)
```

---

## 参考资料

- [StarFive JH7110 Technical Reference Manual](https://doc-en.rvspace.org/JH7110/TRM/)
- [U-Boot dwc_eth_qos.c](https://github.com/starfive-tech/u-boot/blob/main/drivers/net/dwc_eth_qos.c)
- [U-Boot clk-jh7110.c](https://github.com/starfive-tech/u-boot/blob/main/drivers/clk/starfive/clk-jh7110.c)
- [Linux stmmac driver](https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/tree/drivers/net/ethernet/stmicro/stmmac)
