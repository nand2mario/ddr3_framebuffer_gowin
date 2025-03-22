# DDR3 frame buffer for Sipeed Tang Console

This is an example of using DDR3 memory as a frame buffer that also does upscaling. The module,

* Outputs 720p HDMI
* Maintains a frame buffer of user specified dimensions (640x480 in this demo) in DDR3.
* Supports various color depths: 24, 18, 15 and 12 bits.
* Upscales the frame buffer to 720p while maintaining aspect ratio.
* Supports dynamic change of the frame buffer dimensions and the display dimensions.
* Allows updates to the frame buffer from any clock domain.

Some implementation details,
* Uses Gowin IP for DDR3 memory controller. DDR3 is running at 297Mhz, 4x 720 pixel clock of 74.25Mhz (i.e. DDR3-594). Per [Micron datasheet](https://forum.digilent.com/topic/25816-should-max-clock-period-be-min-clock-period/), DDR3 lowest frequency is 300Mhz. However this is working fine for me...
* Streams data from DDR3 about 40 pixels ahead and fills a 16 pixel buffer, to hide DDR3 and controller latency. The delay is changable at runtime.
* DDR3 refresh is completely turned off for better timing consistency. It's fine as the buffer is written to at 60fps, much faster than the allows DRAM 64ms refresh intervals.
* An asynchronous FIFO is used to bridge the data transfer from the user clock domain to the pixel clock domain.

Resource usage: ~3600 LUTs, ~4600 REGs, 16 BRAMs (including the frame buffer, HDMI TX and DDR3 controller).
  
It works on the Sipeed Tang Console board (should also work on Tang Mega 60K or other boards with minor changes). When running the demo project looks like this (a moving green block):

<img src="doc/ddr3_framebuffer.png" width=400>
