// DDR3-backed framebuffer for Tang Mega 60K and Tang Console 60K
// nand2mario, March 2025
//
// - A framebuffer of any size smaller than 1280x720, backed by a single 16-bit 
//   DDR3 chip. The image is automatically upscaled to 1280x720 and displayed on HDMI.
// - Color depth supported: 12, 15, 18, and 24 bits.
// - Dynamic change of framebuffer size is supported. After change the upscaling
//   logic will adapt to the new size.
// - Image is updated by vsync (fb_vsync) and then streaming every pixel (fb_data 
//   and fb_we).
// - Resource usage: 16 BRAMs, ~3000 LUTs, ~3000 REGs, including DDR3 and HDMI IPs.
//
// Internals,
// - Gowin DDR3 controller IP is used to access DDR3. Accesses are done in 4 pixel 
//   chunks (8x 16-bit words). Each pixel is max 32 bits.
// - 720p timings: 
//      1650=1280 + 110(front porch) + 40(sync) + 220(back porch)
//      750 =720  +   5(front porch) +  5(sync)  + 20(back porch)
//   https://projectf.io/posts/video-timings-vga-720p-1080p/#hd-1280x720-60-hz
// - Pixels are read from DDR3 32 pixels in advance, as DDR3 controller's read latency 
//   is about 22 cycles.
//   x                                0  4  8   ...   WIDTH-36 ... WIDTH
//   prefetch  0  4  8 12 16 20 24 28 32 36 40        WIDTH-4
// - Writes are handled in 4 pixel chunks too. Whenever we have 4 pixels
//   accumulated, we write them to DDR3. Reading takes precedence over writing.
module ddr3_framebuffer #(
    parameter WIDTH = 640,           // multiples of 4
    parameter HEIGHT = 480, 
    parameter COLOR_BITS = 24       // RGB666
)(
    input               clk_27,      // 27Mhz input clock
    input               clk_g,       // 50Mhz crystal
    input               pll_lock_27,
    input               rst_n,
    output              clk_out,     // 74.25Mhz pixel clock. could be used by user logic
    output              ddr_rst,     // output reset signal for clk_out
    output              init_calib_complete,

    // Framebuffer interface
    input               clk,         // any clock <= 74.25Mhz (or clk_out)
    input [10:0]        fb_width,    // actual width of the framebuffer
    input [9:0]         fb_height,   // actual height of the framebuffer
    input [10:0]        disp_width,  // display width to upscale to (e.g. 960 for 4:3 aspect ratio, 1080 for 3:2 aspect ratio)
    input  [9:0]  disp_height,
    input               fb_vsync,    // vertical sync signal
    input               fb_we,       // update a pixel and move to next pixel
    input [COLOR_BITS-1:0] fb_data,  // pixel data
    input [5:0]         ddr_prefetch_delay,    // dynamic DDR3 prefetch delay (1-63, default PREFETCH_DELAY), larger if memory pressure is higher 

    input [15:0]        sound_left,
    input [15:0]        sound_right,

    // DDR3 interface
    output [14:0]       ddr_addr,   
    output [3-1:0]      ddr_bank,       
    output              ddr_cs,
    output              ddr_ras,
    output              ddr_cas,
    output              ddr_we,
    output              ddr_ck,
    output              ddr_ck_n,
    output              ddr_cke,
    output              ddr_odt,
    output              ddr_reset_n,
    output [2-1:0]      ddr_dm,
    inout  [16-1:0]     ddr_dq,
    inout  [2-1:0]      ddr_dqs,     
    inout  [2-1:0]      ddr_dqs_n, 

    // HDMI output
	output              tmds_clk_n,
	output              tmds_clk_p,
	output [2:0]        tmds_d_n,
	output [2:0]        tmds_d_p
);

/////////////////////////////////////////////////////////////////////
// Clocks
wire hclk, hclk5;
wire memory_clk;
wire clk_x1 /* synthesis syn_keep=1 */;
assign clk_out = clk_x1;
wire pll_lock;

// dynamic reconfiguration port from DDR controller to framebuffer PLL
wire pll_stop;

// 74.25   pixel clock
// 371.25  5x pixel clock
// 297     DDR3 clock
pll_ddr3 pll_ddr3_inst(
    .lock(pll_lock), 
    .clkout0(), 
//    .clkout1(), 
    .clkout2(memory_clk), 
    .clkin(clk_27), 
    .reset(~pll_lock_27)
);

// 74.25 -> 371.25 TMDS clock
pll_hdmi pll_hdmi_inst(
    .clkout0(hclk5),
    .clkout1(hclk),
    .clkin(clk_x1)
);


/////////////////////////////////////////////////////////////////////
// DDR3 controller

// A single 16-bit 4Gb DDR3 memory chip
wire           app_rdy;             // command and data
reg            app_en;
reg    [2:0]   app_cmd;
reg   [27:0]   app_addr;        

wire           app_wdf_rdy;         // write data
reg            app_wdf_wren;
wire  [15:0]   app_wdf_mask = 0;
wire           app_wdf_end = 1; 
reg  [127:0]   app_wdf_data;        

wire           app_rd_data_valid;   // read data
wire           app_rd_data_end;
wire [127:0]   app_rd_data;     

wire           app_sre_req = 0;      
wire           app_ref_req = 0;
wire           app_burst = 1;
wire           app_sre_act;
wire           app_ref_ack;

DDR3_Memory_Interface_Top u_ddr3 (
    .memory_clk      (memory_clk),
    .pll_stop        (pll_stop),
    .clk             (clk_g),
    .rst_n           (1'b1),   
    //.app_burst_number(0),
    .cmd_ready       (app_rdy),
    .cmd             (app_cmd),
    .cmd_en          (app_en),
    .addr            (app_addr),
    .wr_data_rdy     (app_wdf_rdy),
    .wr_data         (app_wdf_data),
    .wr_data_en      (app_wdf_wren),
    .wr_data_end     (app_wdf_end),
    .wr_data_mask    (app_wdf_mask),
    .rd_data         (app_rd_data),
    .rd_data_valid   (app_rd_data_valid),
    .rd_data_end     (app_rd_data_end),
    .sr_req          (0),
    .ref_req         (0),
    .sr_ack          (app_sre_act),
    .ref_ack         (app_ref_ack),
    .init_calib_complete(init_calib_complete),
    .clk_out         (clk_x1),
    .pll_lock        (pll_lock), 
    //.pll_lock        (1'b1), 
    //ifdef ECC
    //.ecc_err         (ecc_err),
    //endif
    .burst           (app_burst),
    // mem interface
    .ddr_rst         (ddr_rst),
    .O_ddr_addr      (ddr_addr),
    .O_ddr_ba        (ddr_bank),
    .O_ddr_cs_n      (ddr_cs),
    .O_ddr_ras_n     (ddr_ras),
    .O_ddr_cas_n     (ddr_cas),
    .O_ddr_we_n      (ddr_we),
    .O_ddr_clk       (ddr_ck),
    .O_ddr_clk_n     (ddr_ck_n),
    .O_ddr_cke       (ddr_cke),
    .O_ddr_odt       (ddr_odt),
    .O_ddr_reset_n   (ddr_reset_n),
    .O_ddr_dqm       (ddr_dm),
    .IO_ddr_dq       (ddr_dq),
    .IO_ddr_dqs      (ddr_dqs),
    .IO_ddr_dqs_n    (ddr_dqs_n)
);

/////////////////////////////////////////////////////////////////////
// Audio

localparam AUDIO_RATE=48000;
localparam AUDIO_CLK_DELAY = 74250 * 1000 / AUDIO_RATE / 2;
logic [$clog2(AUDIO_CLK_DELAY)-1:0] audio_divider;
logic clk_audio;

always_ff@(posedge clk_x1) 
begin
    if (audio_divider != AUDIO_CLK_DELAY - 1) 
        audio_divider++;
    else begin 
        clk_audio <= ~clk_audio; 
        audio_divider <= 0; 
    end
end

reg [15:0] audio_sample_word [1:0], audio_sample_word0 [1:0];
always @(posedge clk_x1) begin       // crossing clock domain
    audio_sample_word0[0] <= sound_left;
    audio_sample_word[0] <= audio_sample_word0[0];
    audio_sample_word0[1] <= sound_right;
    audio_sample_word[1] <= audio_sample_word0[1];
end

/////////////////////////////////////////////////////////////////////
// HDMI TX

wire [10:0] cx;
wire [9:0] cy;
reg [23:0] rgb;

// HDMI output.
wire [2:0] tmds;
localparam VIDEOID = 4;
localparam VIDEO_REFRESH = 60.0;
localparam AUDIO_BIT_WIDTH = 16;
localparam AUDIO_OUT_RATE = 32000;

hdmi #( .VIDEO_ID_CODE(VIDEOID), 
        .DVI_OUTPUT(0), 
        .VIDEO_REFRESH_RATE(VIDEO_REFRESH),
        .IT_CONTENT(1),
        .AUDIO_RATE(AUDIO_OUT_RATE), 
        .AUDIO_BIT_WIDTH(AUDIO_BIT_WIDTH),
        .START_X(0),
        .START_Y(0) )

hdmi(   .clk_pixel_x5(hclk5), 
        .clk_pixel(clk_x1), 
        .clk_audio(clk_audio),
        .rgb(rgb), 
        .reset( ddr_rst ),
        .audio_sample_word(audio_sample_word),
        .tmds(tmds), 
        .tmds_clock(), 
        .cx(cx), 
        .cy(cy),
        .frame_width(),
        .frame_height() );

// Gowin LVDS output buffer
ELVDS_OBUF tmds_bufds [3:0] (
    .I({hclk, tmds}),
    .O({tmds_clk_p, tmds_d_p}),
    .OB({tmds_clk_n, tmds_d_n})
);

/////////////////////////////////////////////////////////////////////
// 720p Framebuffer
// 

localparam FB_SIZE = WIDTH * HEIGHT;
// localparam X_START = (1280-DISP_WIDTH)/2;
// localparam X_END = (1280+DISP_WIDTH)/2;



reg write_pixels_req;           // toggle to write 8 pixels
reg write_pixels_ack;
reg [9:0] wr_x, wr_y;           // write position
reg [27:0] wr_addr;
reg [27:0] rd_addr;

reg read_pixels_req;            // toggle to read 8 pixels
reg read_pixels_ack;

reg prefetch;                   // will start prefetch next cycle
reg [10:0] prefetch_x;
reg [$clog2(1280+WIDTH)-1:0] prefetch_x_cnt;
reg [$clog2(720+HEIGHT)-1:0] prefetch_y_cnt;
reg [$clog2(FB_SIZE*2)-1:0] prefetch_addr_line;   // current line to prefetch

reg [COLOR_BITS-1:0] pixels [0:31];       // buffer to 32 pixels
reg [$clog2(WIDTH)-1:0] ox;
reg [$clog2(HEIGHT)-1:0] oy;
reg [$clog2(1280+WIDTH)-1:0] xcnt;
reg [$clog2(720+HEIGHT)-1:0] ycnt;

// Framebuffer update - accumulate 4 pixels and then send to DDR3
reg [$clog2(WIDTH)-1:0] b_x;
reg [$clog2(HEIGHT)-1:0] b_y;
reg b_vsync_toggle, b_vsync_toggle_r, b_vsync_toggle_rr;
wire fifo_ready;
wire [COLOR_BITS-1:0] fifo_data;
reg [COLOR_BITS-1:0] b_data [0:2];
asyncfifo #(.BUFFER_ADDR_WIDTH(3), .DATA_WIDTH(COLOR_BITS)) u_asyncfifo (
  .reset(ddr_rst),
  .write_clk(clk), .write(fb_we), .write_data(fb_data), .can_write(),  
  .read_clk(clk_x1), .read(fifo_ready), .read_data(fifo_data),  .can_read(fifo_ready)
);


//////////////////////////////
// VSYNC POLARITY, REFRESH RATE DETECTION + TRIPLE BUFFER LOGIC
//////////////////////////////

reg fb_vsync_r;
always @(posedge clk_x1)
    fb_vsync_r <= fb_vsync;

//Vsync polarity detection (one-shot after 1024 samples)
reg [15:0] vsync_high_cnt = 0;
reg [15:0] vsync_low_cnt = 0;
reg [9:0]  vsync_sample_counter = 0;
reg        vsync_polarity;    // 0 = active high, 1 = active low
reg        polarity_valid = 0;

always @(posedge clk_x1) begin
    if (ddr_rst) begin
        vsync_high_cnt       <= 0;
        vsync_low_cnt        <= 0;
        vsync_sample_counter <= 0;
        polarity_valid       <= 0;
    end else if (!polarity_valid) begin
        if (fb_vsync)
            vsync_high_cnt <= vsync_high_cnt + 1;
        else
            vsync_low_cnt  <= vsync_low_cnt + 1;

        vsync_sample_counter <= vsync_sample_counter + 1;

        if (vsync_sample_counter == 10'd1023) begin
            polarity_valid   <= 1;
            vsync_polarity   <= (vsync_high_cnt > vsync_low_cnt);  // 1 = active low 
        end
    end
end

//Generate frame_start_edge based on detected polarity
wire frame_start_edge = polarity_valid && (
    vsync_polarity ? (fb_vsync_r && !fb_vsync) : (!fb_vsync_r && fb_vsync)
);

//Toggle frame write trigger when new frame starts
reg b_vsync_toggle;
always @(posedge clk_x1) begin
    if (ddr_rst)
        b_vsync_toggle <= 0;
    else if (frame_start_edge)
        b_vsync_toggle <= ~b_vsync_toggle;
end

/////////////// DETECT REFRESH RATE
localparam FRAME_60HZ_CYCLES = 1_237_500;

reg [23:0] frame_timer = 0;
reg [23:0] last_frame_period = 0;

always @(posedge clk_x1) begin
    frame_timer <= frame_timer + 1;

    if (frame_start_edge) begin
        last_frame_period <= frame_timer;
        frame_timer <= 0;
    end
end

reg frame_greater_than_60hz;                 // 1 if the source is above 60hz.  0 if the source is greater than 60hz

always @(posedge clk_x1) begin
    // Debounced check only after each new frame
    if (frame_start_edge) begin
        frame_greater_than_60hz <= (last_frame_period < FRAME_60HZ_CYCLES);
    end
end

///////////////// triple frame arbiter//////////////////////////////

parameter BASE_OFFSET = 28'h0100_000; // 16MB offset.   So around the lowest ~64MB are framebuffer logic.

// Next-frame wraparound function (1 → 2 → 3 → 1)
function [1:0] next_frame(input [1:0] f);
    begin
        if (f == 3)
            next_frame = 1;
        else
            next_frame = f + 1;
    end
endfunction

// Detect output frame edge
wire output_frame_done = (cx == 0 && cy == 0);

reg [1:0] current_write_frame = 2'd1;
reg [1:0] current_read_frame  = 2'd2;

wire [1:0] next_write_frame = next_frame(current_write_frame);
wire [1:0] next_read_frame  = next_frame(current_read_frame);

reg dropped_frame;
reg repeated_frame;

always @(posedge clk_x1) begin
    if (ddr_rst) begin
        current_write_frame <= 2'd1;
        current_read_frame  <= 2'd2;
        dropped_frame       <= 0;
        repeated_frame      <= 0;
    end else begin
        dropped_frame  <= 0;
        repeated_frame <= 0;

        // === WRITE SIDE LOGIC ===
        if (frame_start_edge) begin
            if (next_write_frame != current_read_frame) begin
                current_write_frame <= next_write_frame;
            end else begin
                dropped_frame <= 1;  // write too fast
            end
        end

        // === READ SIDE LOGIC ===
        if (output_frame_done) begin
            if (current_read_frame != current_write_frame &&
                next_read_frame != current_write_frame) begin
                current_read_frame <= next_read_frame;
            end else begin
                repeated_frame <= 1; // write too slow
            end
        end
    end
end

                                           // current read/write frame indexs at 1, and ends at 3. triple frame.
wire [27:0] frame_base_write_addr =  BASE_OFFSET + (current_write_frame * fb_width * fb_height);  //current_write_frame
wire [27:0] frame_base_read_addr  =  BASE_OFFSET + (current_read_frame * fb_width * fb_height); //current_read_frame


////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

// cross to clk_x1 domain
always @(posedge clk_x1) begin
    b_vsync_toggle_rr <= b_vsync_toggle_r;
    b_vsync_toggle_r <= b_vsync_toggle;
end

always @(posedge clk_x1) begin
    if (ddr_rst) begin
        wr_x <= 0; wr_y <= 0;
        write_pixels_req <= 0;
    end else begin
        if (b_vsync_toggle_rr != b_vsync_toggle_r) begin
            wr_x <= 0; wr_y <= 0;
        end
        if (fifo_ready) begin
            // accumulate 4 pixels and send to ddr
            if (wr_x[1:0] == 3) begin
                wr_addr <= frame_base_write_addr + {wr_y * fb_width + {wr_x[9:2], 2'b0}, 1'b0};
                app_wdf_data <= {(32-COLOR_BITS)'(1'b0), fifo_data, (32-COLOR_BITS)'(1'b0), b_data[2], 
                                 (32-COLOR_BITS)'(1'b0), b_data[1], (32-COLOR_BITS)'(1'b0), b_data[0]};
                write_pixels_req <= ~write_pixels_req;      // execute write
            end else begin
                b_data[wr_x[1:0]] <= fifo_data;
            end
            // move to next pixel
            wr_x <= wr_x + 1;
            if (wr_x + 1 >= fb_width) begin
                wr_x <= 0;
                if (wr_y +1 < fb_height) begin
                    wr_y <= wr_y + 1;
                end
            end
        end
    end
end

// upscaling and output RGB
reg [$clog2(WIDTH)-1:0] ox_r;
reg [10:0] x_start, x_end;      // determined by fb_width
reg [10:0] x_prefetch_start;
reg [9:0] y_start, y_end;
// ──────────────────────────────────────────────
// Nearest‑neighbour scaler and HDMI pixel drive
// ──────────────────────────────────────────────
always @(posedge clk_x1) begin
    if (ddr_rst) begin
        ox   <= 0;
        oy   <= 0;
        xcnt <= 0;
        ycnt <= 0;  
    end else begin


        // ----- end‑of‑active‑line bookkeeping -----
        if (cx == x_end) begin
            ox   <= 0;
            xcnt <= 0;

            if (cy == 0) begin
                // start of a new frame
                oy   <= 0;
                ycnt <= fb_height;
            end else begin
                ycnt <= ycnt + fb_height;
                if (ycnt >= disp_height) begin
                    ycnt <= ycnt - disp_height;
                    oy   <= oy + 1;
                end
            end
        end

        // ----- active‑pixel region -----
    if ( cx >= x_start && cx <  x_end &&
        cy >= y_start && cy <  y_end ) begin
            xcnt <= xcnt + fb_width;
            if (xcnt >= disp_width) begin
                xcnt <= xcnt - disp_width;
                ox   <= ox + 1;
            end
            // fetch pixel from 32‑pixel cache
            rgb <= torgb(pixels[cx == 0 ? 0 : ox[3:0]]);
        end else begin
            // letter / pillar box colour
            rgb <= 24'h202020;
        end
    end
end


// some precalculation
always @(posedge clk_x1) begin
    x_start <= (1280 - disp_width ) / 2;
    x_end   <= (1280 + disp_width ) / 2;

    x_prefetch_start <= x_start - ddr_prefetch_delay;

    // vertical centring
    y_start <= (720 - disp_height) / 2;
    y_end   <= (720 + disp_height) / 2;

end

// TODO: wrapping while prefetching is not implemented yet
// ──────────────────────────────────────────────
// DDR3 read‑side prefetch (32‑pixel look‑ahead)
// ──────────────────────────────────────────────
always @(posedge clk_x1) begin
    if (ddr_rst) begin
        prefetch           <= 1'b0;
        prefetch_x         <= 11'd0;
        prefetch_x_cnt     <= 0;
        prefetch_y_cnt     <= 0;
        prefetch_addr_line <= 0;
    end else begin
        prefetch <= 1'b0;   // default, asserted only on the 4‑pixel boundaries

        // ----- first fetch of a scanline -----
      if ( cx == x_prefetch_start && cy >= y_start && cy < y_end ) begin
            prefetch_x      <= 0;
            prefetch_x_cnt  <= fb_width;

            // update vertical accumulator
            if (cy == 0) begin
                prefetch_y_cnt     <= 0;
                prefetch_addr_line <= 0;
            end else begin
                prefetch_y_cnt <= prefetch_y_cnt + fb_height;
                if (prefetch_y_cnt >= disp_height) begin 
                    prefetch_y_cnt     <= prefetch_y_cnt - disp_height; 
                    prefetch_addr_line <= frame_base_read_addr + prefetch_addr_line + {fb_width,1'b0};
                end
            end

        // ----- continue horizontally across the line -----
        end else if (prefetch_x < fb_width) begin
            prefetch_x_cnt <= prefetch_x_cnt + fb_width;
            if (prefetch_x_cnt >= disp_width) begin
                prefetch_x_cnt <= prefetch_x_cnt - disp_width;
                prefetch_x     <= prefetch_x + 1;

                // issue DDR3 READ every four source pixels
                if (prefetch_x[1:0] == 2'b00) begin
                    prefetch        <= 1'b1;
                    read_pixels_req <= ~read_pixels_req;
                    rd_addr         <= {prefetch_x,1'b0} + prefetch_addr_line; // word addr
                end
            end
        end
    end
end




// actual framebuffer DDR3 read/write
always @(posedge clk_x1) begin
    app_en <= 0;
    app_wdf_wren <= 0;

    if (ddr_rst) begin
        read_pixels_ack <= 0;
    end else begin
        // prefetch 32 pixels in advance
        if (read_pixels_req ^ read_pixels_ack && app_rdy && app_wdf_rdy) begin
            app_en <= 1;
            app_cmd <= 3'b001;
            app_addr <= rd_addr;
            read_pixels_ack <= read_pixels_req;
        end else if (write_pixels_req ^ write_pixels_ack && app_rdy && app_wdf_rdy) begin
            // on write_pixels_req, write 8 pixels to framebuffer
            app_en <= 1;
            app_cmd <= 3'b000;
            app_addr <= wr_addr;
            app_wdf_wren <= 1;
            write_pixels_ack <= write_pixels_req;
        end else begin
            // Default/idle case.   If you notice on logic analyzer, there is a lot of IDLE time and module defaults to 111 when idle.
            app_cmd <= 3'b111;
            app_en <= 0;
        end
    end
end


// receive pixels from DDR3 and write to pixels[] in 8 cycles
reg [3:0] bram_addr;        // 32 pixels, receive 4 pixels per request
always @(posedge clk_x1) begin
    if (cx == x_start && cy >= y_start && cy < y_end)                                    // reset addr before line start
        bram_addr <= 0;

    if (app_rd_data_valid) begin
        for (int i = 0; i < 4; i++) begin
            pixels[bram_addr+i] <= app_rd_data[32*i+:COLOR_BITS];
        end
        bram_addr <= bram_addr + 4;
    end
end


// Convert color to RGB888
function [23:0] torgb(input [23:0] pixel);
    case (COLOR_BITS)
    12: torgb = {pixel[11:8], 4'b0, pixel[7:4], 4'b0, pixel[3:0], 4'b0};
    15: torgb = {pixel[14:10], 3'b0, pixel[9:5], 3'b0, pixel[4:0], 3'b0};
    18: torgb = {pixel[17:12], 2'b0, pixel[11:6], 2'b0, pixel[5:0], 2'b0};
    21: torgb = {pixel[20:14], 1'b0, pixel[13:7], 1'b0, pixel[6:0], 1'b0};
    24: torgb = pixel;
    default: torgb = 24'hbabeef;
    endcase
endfunction

endmodule



// From: https://log.martinatkins.me/2020/06/07/verilog-async-fifo/
// Async FIFO implementation
module asyncfifo(
  input                       reset,
  input                       write_clk,
  input                       write,
  input      [DATA_WIDTH-1:0] write_data,
  output reg                  can_write,
  input                       read_clk,
  input                       read,
  output reg [DATA_WIDTH-1:0] read_data,
  output reg                  can_read
);
    parameter DATA_WIDTH = 16;
    parameter BUFFER_ADDR_WIDTH = 8;
    parameter BUFFER_SIZE = 2 ** BUFFER_ADDR_WIDTH;

    // Our buffer as a whole is accessed by both the write_clk and read_clk
    // domains, but read_clk is only used to access elements >= read_ptr and
    // write_clk only for elements < read_ptr. We're expecting this buffer to
    // be inferred as a dual-port block RAM, so the board-specific top module
    // should choose a suitable buffer size to allow that inference.
    reg [DATA_WIDTH-1:0] buffer[BUFFER_SIZE-1:0] /*xx synthesis syn_ramstyle="block_ram" */ ;

    ///// WRITE CLOCK DOMAIN /////

    // This is an address into the buffer array.
    // It intentionally has one additional bit so we can track wrap-around by
    // comparing with the MSB of read_ptr (or, at least, with the grey-code
    // form that we synchronize over into this clock domain.)
    reg [BUFFER_ADDR_WIDTH:0] write_ptr;
    wire [BUFFER_ADDR_WIDTH-1:0] write_addr = write_ptr[BUFFER_ADDR_WIDTH-1:0]; // truncated version without the wrap bit

    // This is the grey-coded version of write_ptr in the write clock domain.
    reg [BUFFER_ADDR_WIDTH:0] write_ptr_grey_w;

    // This is the grey-coded version of read_ptr in the write clock domain,
    // synchronized over here using module read_ptr_grey_sync declared later.
    wire [BUFFER_ADDR_WIDTH:0] read_ptr_grey_w;

    // Write pointer (and its grey-coded equivalent) increments whenever
    // "write" is set on a clock, as long as our buffer isn't full.
    wire [BUFFER_ADDR_WIDTH:0] next_write_ptr = write_ptr + 1;
    wire [BUFFER_ADDR_WIDTH:0] next_write_ptr_grey_w = (next_write_ptr >> 1) ^ next_write_ptr;
    // Our buffer is full if the read and write addresses are the same but the
    // MSBs (wrap bits) are different. We compare the grey code versions here
    // so we can use our cross-domain-synchronized copy of the read pointer.
    wire current_can_write = write_ptr_grey_w != { ~read_ptr_grey_w[BUFFER_ADDR_WIDTH:BUFFER_ADDR_WIDTH-1], read_ptr_grey_w[BUFFER_ADDR_WIDTH-2:0] };
    wire next_can_write = next_write_ptr_grey_w != { ~read_ptr_grey_w[BUFFER_ADDR_WIDTH:BUFFER_ADDR_WIDTH-1], read_ptr_grey_w[BUFFER_ADDR_WIDTH-2:0] };
    always @(posedge write_clk or posedge reset) begin
        if (reset) begin
            write_ptr <= 0;
            write_ptr_grey_w <= 0;
            can_write <= 1;
        end else begin
            if (write && can_write) begin
                write_ptr <= next_write_ptr;
                write_ptr_grey_w <= next_write_ptr_grey_w;
                can_write <= next_can_write;
            end else begin
                can_write <= current_can_write;
            end
        end
    end

    // If "write" is set on a clock then we commit write_data into the current
    // write address.
    always @(posedge write_clk) begin
        if (write && can_write) begin
            buffer[write_addr] <= write_data;
        end
    end

    ///// READ CLOCK DOMAIN /////

    // This is an address into the buffer array.
    // It intentionally has one additional bit so we can track wrap-around by
    // comparing with the MSB of write_ptr (or, at least, with the grey-code
    // form that we synchronize over into this clock domain.)
    reg [BUFFER_ADDR_WIDTH:0] read_ptr;
    wire [BUFFER_ADDR_WIDTH-1:0] read_addr = read_ptr[BUFFER_ADDR_WIDTH-1:0]; // truncated version without the wrap bit

    // This is the grey-coded version of write_ptr in the read clock domain.
    reg [BUFFER_ADDR_WIDTH:0] read_ptr_grey_r;

    // This is the grey-coded version of write_ptr in the read clock domain,
    // synchronized over here using module write_ptr_grey_sync declared later.
    wire [BUFFER_ADDR_WIDTH:0] write_ptr_grey_r;

    // Read pointer (and its grey-coded equivalent) increments whenever
    // "read" is set on a clock, as long as our buffer isn't full.
    wire [BUFFER_ADDR_WIDTH:0] next_read_ptr = read_ptr + 1;
    wire [BUFFER_ADDR_WIDTH:0] next_read_ptr_grey_r = (next_read_ptr >> 1) ^ next_read_ptr;
    // Our buffer is empty if the read and write addresses are the same and the
    // MSBs (wrap bits) are also equal. We compare the grey code versions here
    // so we can use our cross-domain-synchronized copy of the write pointer.
    wire current_can_read = read_ptr_grey_r != write_ptr_grey_r;
    wire next_can_read = next_read_ptr_grey_r != write_ptr_grey_r;
    always @(posedge read_clk or posedge reset) begin
        if (reset) begin
            read_ptr <= 0;
            read_ptr_grey_r <= 0;
            read_data <= 0;
            can_read <= 0;
        end else begin
            if (read) begin
                if (can_read) begin
                    read_ptr <= next_read_ptr;
                    read_ptr_grey_r <= next_read_ptr_grey_r;
                end
                can_read <= next_can_read;
                if (next_can_read) begin
                    read_data <= buffer[next_read_ptr];
                end else begin
                    read_data <= 0;
                end
            end else begin
                can_read = current_can_read;
                if (current_can_read) begin
                    read_data <= buffer[read_addr];
                end else begin
                    read_data <= 0;
                end
            end
        end
    end

    ///// CROSS-DOMAIN /////

    // Synchronize read_ptr_grey_r into read_ptr_grey_w.
    crossdomain #(.SIZE(BUFFER_ADDR_WIDTH+1)) read_ptr_grey_sync (
        .reset(reset),
        .clk(write_clk),
        .data_in(read_ptr_grey_r),
        .data_out(read_ptr_grey_w)
    );

    // Synchronize write_ptr_grey_w into write_ptr_grey_r.
    crossdomain #(.SIZE(BUFFER_ADDR_WIDTH+1)) write_ptr_grey_sync (
        .reset(reset),
        .clk(read_clk),
        .data_in(write_ptr_grey_w),
        .data_out(write_ptr_grey_r)
    );

endmodule

// This is a generalization of the crossdomain module from earlier that
// now supports a customizable value size, so we can safely transmit multi-bit
// values as long as they are grey coded.
module crossdomain #(parameter SIZE = 1) (
  input reset,
  input clk,
  input [SIZE-1:0] data_in,
  output reg [SIZE-1:0] data_out
);

    reg [SIZE-1:0] data_tmp;

    always @(posedge clk) begin
        if (reset) begin
            {data_out, data_tmp} <= 0;
        end else begin
            {data_out, data_tmp} <= {data_tmp, data_in};
        end
    end

endmodule