# DDR3 frame buffer for Sipeed Tang Console

This is an example of using DDR3 memory as a frame buffer that also does upscaling. The project,

* Outputs 720p HDMI
* Maintains a frame buffer of user specified dimensions (640x480 in this demo) in DDR3.
* Color depth supported: 24, 18, 15 and 12 bits.
* Upscales the frame buffer to 720p while maintaining aspect ratio.

Some implementation details,
* Uses Gowin IP for DDR3 memory controller. DDR3 is running at 297Mhz, 4x 720 pixel clock of 74.25Mhz (i.e. DDR3-594). Per [Micron datasheet](https://forum.digilent.com/topic/25816-should-max-clock-period-be-min-clock-period/), DDR3 lowest frequency is 300Mhz. However this is working fine for me...
* Streams data from DDR3 about 40 pixels ahead and fills a 16 pixel buffer, to hide DDR3 and controller latency.
* DDR3 refresh is completely turned off for better timing consistency. It's fine as the buffer is written to at 60fps, much faster than the allows DRAM 64ms refresh intervals.

Resource usage: ~3400 LUTs, ~4200 REGs, 16 BRAMs
  
It works on the Sipeed Tang Console board (should work on Tang Mega 60K or other boards with minor changes). When running the demo project looks like this (a moving green block):

<img src="doc/ddr3_framebuffer.png" width=400>
