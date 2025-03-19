# DDR3 frame buffer for Sipeed Tang Console

This is an example of using DDR3 memory as a frame buffer that also does upscaling. The project,

* Outputs 720p HDMI
* Maintains a frame buffer of user specified dimensions (640x480 here) in DDR3.
* Uses Gowin IP for DDR3 memory controller. DDR3 is running at 297Mhz, 4x 720 pixel clock of 74.25Mhz (i.e. DDR3-594). Per [Micron datasheet](https://forum.digilent.com/topic/25816-should-max-clock-period-be-min-clock-period/), DDR3 lowest frequency is 300Mhz. However this is working fine for me...
* Streams data from DDR3 32 pixels ahead and fills a 64 pixel buffer, to hide DDR3 and controller latency. The controller's read latency is about 22 cycles in my testing.
* Upscales the frame buffer to 720p while maintaining aspect ratio.
* As a demo, updates the frame buffer at 60fps showing a moving green block.
* The pixel format is currently 15-bit RGB5 (fine for my retro gaming needs). Should be easy to change to 24-bit RGB.
* DDR3 refresh is turned off due to glithces it introduces for some reason. It's fine without refresh as the buffer is written to at 60fps, faster than the allows 64ms refresh intervals.
  
It works on the Sipeed Tang Console board (should work on Tang Mega 60K or other boards with minor changes). When running it looks like this:

<img src="doc/ddr3_framebuffer.png" width=400>
