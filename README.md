# DDR3 frame buffer for Sipeed Tang Console

This is an example of using DDR3 memory as a frame buffer. The project,

* Outputs 720p HDMI
* Maintains a frame buffer of user specified dimensions (640x480 here) in DDR3-800.
* Uses Gowin IP for DDR3 memory controller. 
* Upscale the frame buffer to 720p while maintaining aspect ratio.
* Updates the frame buffer at 60fps.
* Each pixel is now 15-bit RGB5. Should be easy to change to 24-bit RGB.
* DDR3 refresh is turned off due to glithces it introduces for some reason. It's fine without refresh as the buffer is written to at 60fps, faster than the allows 64ms refresh intervals.
  
It works on the Sipeed Tang Console board (should work on Tang Mega 60K or other boards with minor changes). When running it looks like this:

<img src="doc/ddr3_framebuffer.png" width=400>
