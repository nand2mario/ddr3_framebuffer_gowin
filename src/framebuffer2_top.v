// DDR3-backed framebuffer example for Tang Mega 60K and Tang Console 60K
// nand2mario, March 2025
//
// - Goal: a framebuffer of any size smaller than 1280x720. it is automatically
//   upscaled to 1280x720 and displayed on HDMI. to make things simpler,
//   each pixel is at most 16 bits (so at most RGB5 for now).
// - One DDR3 and BRAM word is enough for one pixel.
// - Gowin DDR3 controller read latency is about 22 cycles. each read yields
//   8 pixels.
// - If we wants to support full 720p framebuffer, then the BRAM is actually
//   the bottleneck. it needs to maintain 100% read/write utilization, one
//   pixel per cycle.
// - For upscaling, we keep original pixel coordinates: ox and oy
// - 720p timings: 
//      1650=1280 + 110(front porch) + 40(sync) + 220(back porch)
//      750 =720  +   5(front porch) +  5(sync)  + 20(back porch)
//   https://projectf.io/posts/video-timings-vga-720p-1080p/#hd-1280x720-60-hz
//   ox has range OX_MAX = FB_WIDTH/1280*1650
// - 32 pixels in advance in ox, we start reading 8 pixels (128 bits) from DDR3
//   ox                          0   8   ...   FB_WIDTH-40 FB_WIDTH
//   prefetch  0    8   16   24  32  40        FB_WIDTH-8
// - writes are handled in 8 pixel chunks too. whenever we have 8 pixels
//   accumulated, we write them to DDR3. reading takes precedence over writing.
module top(
    input               clk_g,
    input               key,
    output  [7:0]       leds,

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

// Frame buffer dimensions
parameter FB_WIDTH = 640;       // has to be multiple of 8
parameter FB_HEIGHT = 480;
// Display dimensions, height is fixed 720. 
parameter DISP_WIDTH = 960;     // If < 1280, frame is centered with black bars on the sides.


/////////////////////////////////////////////////////////////////////
// Clocks

reg rst_n = 0;
reg [15:0] rst_cnt = 16'hffff;

always @(posedge clk_g) begin
    rst_cnt <= rst_cnt == 0 ? 0: rst_cnt - 1;
    if (rst_cnt == 0 && !key)
        rst_n <= 1;
end

wire clk27;
wire hclk, hclk5;
wire memory_clk;
wire clk_x1;
wire pll_lock_27;
wire pll_lock;
wire ddr_rst;
wire init_calib_complete;

// dynamic reconfiguration port from DDR controller to framebuffer PLL
reg wr;     // for mDRP
wire mdrp_inc;
wire [1:0] mdrp_op;
wire [7:0] mdrp_wdata;
wire [7:0] mdrp_rdata;
wire pll_stop;
reg pll_stop_r;

pll_27 pll_27_inst(
    .clkout0(clk27),
    .clkin(clk_g),
    .lock(pll_lock_27)
);

// 74.25   pixel clock
// 371.25  5x pixel clock
// 297     DDR3 clock
pll_ddr3 pll_ddr3_inst(
    .lock(pll_lock), 
    .clkout0(), 
//    .clkout1(), 
    .clkout2(memory_clk), 
    .clkin(clk27), 
    .reset(~pll_lock_27),
    .mdclk(clk_g), 
    .mdopc(mdrp_op),        // 0: nop, 1: write, 2: read
    .mdainc(mdrp_inc),      // increment register address
    .mdwdi(mdrp_wdata),     // data to be written
    .mdrdo(mdrp_rdata)      // data read from register
);

// 74.25 -> 371.25 TMDS clock
pll_hdmi pll_hdmi_inst(
    .clkout0(hclk5),
    .clkout1(hclk),
    .clkin(clk_x1)
);

reg mdrp_wr;
reg [7:0] pll_stop_count;
pll_mDRP_intf u_pll_mDRP_intf(
    .clk(clk_g),
    .rst_n(1'b1),
    .pll_lock(pll_lock),
    .wr(mdrp_wr),
    .mdrp_inc(mdrp_inc),
    .mdrp_op(mdrp_op),
    .mdrp_wdata(mdrp_wdata),
    .mdrp_rdata(mdrp_rdata)
);    

always@(posedge clk_g) begin
    pll_stop_r <= pll_stop;
    mdrp_wr <= pll_stop ^ pll_stop_r;
    if (pll_stop_r && !pll_stop && pll_stop_count != 8'hff) begin
        pll_stop_count <= pll_stop_count + 1;
    end
end

assign leds = ~{7'b0, init_calib_complete};

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
    .rst_n           (rst_n),   //rst_n
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
    //`ifdef ECC
    //.ecc_err         (ecc_err),
    //`endif
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
        .clk_audio(),           // TODO: add audio
        .rgb(rgb), 
        .reset( ddr_rst ),
        .audio_sample_word(),
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
// And a moving block as test pattern

localparam FB_SIZE = FB_WIDTH * FB_HEIGHT;
localparam X_START = (1280-DISP_WIDTH)/2;
localparam X_END = (1280+DISP_WIDTH)/2;
localparam RENDER_DELAY = 74_250_000 * 8 / FB_WIDTH / FB_HEIGHT / 60;   // 32

// RGB5
localparam GREY = 16'b0_10000_10000_10000;
localparam RED = 16'b1_11111_00000_00000;
localparam GREEN = 16'b0_00000_11111_00000;
localparam BLUE = 16'b0_00000_00000_11111;

reg [7:0] cursor_x, cursor_y;   // a green 8x8 block on grey background for demo
reg [7:0] cursor_delay;         // 32 cycles per write

reg write_pixels_req;           // toggle to write 8 pixels
reg write_pixels_ack;
reg [9:0] wr_x, wr_y;           // write position
reg [$clog2(FB_SIZE)-1:0] wr_addr;

reg read_pixels_req;            // toggle to read 8 pixels
reg read_pixels_ack;
reg [$clog2(FB_SIZE)-1:0] rd_addr;
reg prefetch;                   // will start prefetch next cycle
reg [10:0] prefetch_x;
reg [$clog2(DISP_WIDTH+FB_WIDTH)-1:0] prefetch_x_cnt;
reg [$clog2(720+FB_HEIGHT)-1:0] prefetch_y_cnt;
reg [$clog2(FB_SIZE-1)-1:0] prefetch_addr_line;   // current line to prefetch

reg [15:0] pixels [0:63];       // BRAM buffer to 64 pixels
reg [$clog2(FB_WIDTH)-1:0] ox;
reg [$clog2(FB_HEIGHT)-1:0] oy;
reg [$clog2(DISP_WIDTH+FB_WIDTH)-1:0] xcnt;
reg [$clog2(720+FB_HEIGHT)-1:0] ycnt;

// rendering to framebuffer
// - 1 write per 32 cycles
// - cycles per frame: 640*480/8*32 = 1.228M, about 60.5 fps
reg [4:0] bg_r = 0, bg_g = 31, bg_b = 16;
wire [15:0] bg_color = {bg_r[4:0], bg_g[4:0], bg_b[4:0]};

reg [15:0] frame_cnt;
always @(posedge clk_x1) begin
    if (ddr_rst) begin
        cursor_delay <= RENDER_DELAY-1;
        wr_x <= 0; wr_y <= 0;
        write_pixels_req <= 0;
    end else begin
        if (cx == 0 && cy == 0) 
            frame_cnt <= frame_cnt + 1;
        cursor_delay <= cursor_delay - 1;
        if (cursor_delay == 0) begin
            if (wr_x == 0 && wr_y == 0 /*&& frame_cnt[1:0] == 1*/) begin   // move cursor every 4 frames
                cursor_x <= cursor_x + 1;
                if (cursor_x == 640/8-1) begin
                    cursor_x <= 0;
                    cursor_y <= cursor_y + 4;
                    if (cursor_y >= 480/8-1)
                        cursor_y <= 0;
                end
                if (frame_cnt[3:0] == 0) begin
                    bg_r <= bg_r + 1;
                    bg_g <= bg_g + 2;
                    bg_b <= bg_b + 3;
                end
            end
            
            cursor_delay <= RENDER_DELAY-1;
            wr_x <= wr_x + 8;
            if (wr_x == FB_WIDTH - 8) begin
                wr_x <= 0;
                wr_y <= wr_y + 1;
                if (wr_y == FB_HEIGHT - 1) 
                    wr_y <= 0;
            end
            wr_addr <= wr_y * FB_WIDTH + wr_x;
            if (   wr_y[9:3] >= cursor_y && wr_y[9:3] < cursor_y+4 
                && wr_x[9:3] >= cursor_x && wr_x[9:3] < cursor_x+4) begin
                app_wdf_data <= {GREEN, GREEN, GREEN, GREEN, GREEN, GREEN, GREEN, GREEN};
            end else begin
                app_wdf_data <= {bg_color, bg_color, bg_color, bg_color, bg_color, bg_color, bg_color, bg_color};
            end
            write_pixels_req <= ~write_pixels_req;
        end
    end
end

// upscaling and output RGB
reg [$clog2(FB_WIDTH)-1:0] ox_r;
always @(posedge clk_x1) begin
    if (ddr_rst) begin
        ox <= 0; oy <= 0; xcnt <= 0; ycnt <= 0;
    end else begin
        // keep original pixel coordinates
        if (cx == X_END) begin
            ox <= 0; xcnt <= 0;
            if (cy == 0) begin
                oy <= 0;
                ycnt <= FB_HEIGHT;
            end else begin
                ycnt <= ycnt + FB_HEIGHT;
                if (ycnt + FB_HEIGHT > 720) begin
                    ycnt <= ycnt + FB_HEIGHT - 720;
                    oy <= oy + 1;
                end
            end
        end 
        if (cx >= X_START && cx < X_END) begin
            xcnt <= xcnt + FB_WIDTH;
            if (xcnt + FB_WIDTH > DISP_WIDTH) begin
                xcnt <= xcnt + FB_WIDTH - DISP_WIDTH;
                ox <= ox + 1;
            end
            rgb <= torgb(pixels[cx == 0 ? 0 : ox[5:0]]);
        end else
            rgb <= 24'h202020;

        // if (cy >= 300 && cy < 330)    // a blue bar in the middle for debug
        //     rgb <= 24'h4040ff;
    end
end

// prefetch timings
localparam PREFETCH_DELAY = 32 * DISP_WIDTH / FB_WIDTH;     // "upscaled" delay of 32 pixels, 48
// X_START is 160

generate 
    
if (X_START >= PREFETCH_DELAY) begin
    always @(posedge clk_x1) begin
        if (ddr_rst) begin
            prefetch <= 0;
        end else begin
            prefetch <= 0;
            if (cx == X_START - PREFETCH_DELAY) begin
                prefetch <= 1;
                prefetch_x <= 0;
                prefetch_x_cnt <= FB_WIDTH;
                if (cy == 0) begin
                    prefetch_y_cnt <= 0;
                    prefetch_addr_line <= 0;
                end else begin
                    prefetch_y_cnt <= prefetch_y_cnt + FB_HEIGHT;
                    if (prefetch_y_cnt + FB_HEIGHT >= 720) begin
                        prefetch_y_cnt <= prefetch_y_cnt + FB_HEIGHT - 720;
                        prefetch_addr_line <= prefetch_addr_line + FB_WIDTH;
                    end
                end
            end else if (prefetch_x < FB_WIDTH) begin
                prefetch_x_cnt <= prefetch_x_cnt + FB_WIDTH;
                if (prefetch_x_cnt + FB_WIDTH >= DISP_WIDTH) begin
                    prefetch_x_cnt <= prefetch_x_cnt + FB_WIDTH - DISP_WIDTH;
                    prefetch_x <= prefetch_x + 1;
                    if (prefetch_x[2:0] == 7)
                        prefetch <= 1;
                end
            end
        end
    end

    // prefetch pixels into pixels[]
    always @(posedge clk_x1) begin
        if (prefetch) begin
            read_pixels_req <= ~read_pixels_req;
            rd_addr <= {prefetch_x[10:1],1'b0} + prefetch_addr_line;  // 0, 8, 16, 24
        end
    end
end else begin
    $error("Line wrapping during prefetch is not implemented yet");
end

endgenerate


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
        end
    end
end

// receive pixels from DDR3 and write to pixels[] in 8 cycles
reg [5:0] bram_addr;        // 64 pixels, receive 8 pixels per request
reg [111:0] bram_sr;        // 8 pixels, 128 bits per request
always @(posedge clk_x1) begin
    if (cx == 0)                                    // reset addr before line start
        bram_addr <= 0;

    // FIXME: this is using a lot of registers and LUTs. The
    // data arrival can be as close as 4 cycles apart. So two shift registers
    // should be enough.
    if (app_rd_data_valid) begin
        for (int i = 0; i < 8; i++) begin
            pixels[bram_addr+i] <= app_rd_data[16*i+:16];
        end
        bram_addr <= bram_addr + 8;
    end
    // if (app_rd_data_valid) begin
    //     pixels[bram_addr] <= app_rd_data[15:0];     // write 1st pixel
    //     bram_sr <= app_rd_data[127:16];             // save the rest 7 pixels
    //     bram_addr <= bram_addr + 1;
    // end else if (bram_addr[2:0] != 0) begin
    //     pixels[bram_addr] <= bram_sr[15:0];         // shift and write remaining pixels
    //     bram_sr <= {16'h0, bram_sr[111:16]};
    //     bram_addr <= bram_addr + 1;
    // end
end

// RGB5 to RGB8
function [23:0] torgb(input [15:0] pixel);
    torgb = {pixel[15:11], 3'b0, pixel[10:5], 3'b0, pixel[4:0], 3'b0};
endfunction

endmodule
