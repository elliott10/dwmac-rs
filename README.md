# DWMAC ethernet driver
dwmac ethernet driver in Rust on board `StarFive VisionFive 2`.

## Quick Start

For instance, [dwmac ethernet driver in Rust on StarryOS]( )

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
cargo build --target=aarch64-unknown-none-softfloat
```

## About ethernet
* MAC: "starfive,dwmac" "snps,dwmac-5.10a"
* PHY: Motorcomm YT8531C
