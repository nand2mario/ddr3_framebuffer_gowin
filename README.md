# DDR3 Frame Buffer for Sipeed Tang Console 60K / 138K

This example demonstrates using DDR3 memory as a high-performance frame buffer with integrated upscaling. The module provides the following features:

### **Key Features**

* Outputs **720p HDMI** video.
* Maintains a **user-configurable frame buffer** (e.g., 640×480 in this demo) stored in DDR3.
* Supports multiple **color depths**: 24, 18, 15, and 12 bits.
* **Upscales** the frame buffer to 720p while preserving aspect ratio.
* Supports **dynamic changes** to both frame buffer dimensions and display resolution.
* Allows frame buffer updates from **any clock domain**.
* Compatible with both **Tang Console 60K (GW5AT-60)** and **Tang Console 138K (GW5AST-138)**.
  *(Note: PLL resources differ slightly, but the DDR3 and HDMI subsystems are identical.)*

### **Implementation Details**

* Uses the **Gowin DDR3 memory controller IP**. DDR3 operates at **297 MHz**—exactly **4×** the 720p pixel clock of **74.25 MHz** (i.e., DDR3-594).
  *(Per [Micron’s datasheet](https://forum.digilent.com/topic/25816-should-max-clock-period-be-min-clock-period/), the minimum rated frequency is 300 MHz, but 297 MHz works reliably in practice.)*
* Streams data from DDR3 **\~40 pixels ahead** of the current scanout and buffers **16 pixels** locally to hide DDR latency.
  *(This delay is adjustable at runtime.)*
* **Disables DDR3 auto-refresh** for consistent timing, which is safe since the buffer is refreshed at **60 fps**—far faster than the DRAM’s **64 ms** refresh requirement.
* Uses an **asynchronous FIFO** to bridge between the **user clock domain** and the **pixel clock domain**.

### **Resource Usage**

* \~3,600 LUTs
* \~4,600 registers
* 16 BRAMs *(including frame buffer staging, HDMI TX, and DDR3 controller buffers)*

### **Demo Project**

This design has been tested on the **Sipeed Tang Console 60K / 138K** and should work on similar board like the **Tang Mega 60K** with minimal changes.
When running the demo, the display shows several test patterns, for example a **moving green block**:

<img src="doc/ddr3_framebuffer.png" width=400>
