// DDR3-backed framebuffer example for Tang Mega 60K and Tang Console 60K
// nand2mario, March 2025
//
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

reg rst_n = 0;
reg [15:0] rst_cnt = 16'hffff;

always @(posedge clk_g) begin
    rst_cnt <= rst_cnt == 0 ? 0: rst_cnt - 1;
    if (rst_cnt == 0 
        && !key
    )
        rst_n <= 1;
end

wire clk_27, clk_x1;
wire pll_lock_27;
wire ddr_rst, init_calib_complete;
reg vsync;
reg [18:0] fb_addr;     // 640*480
reg [17:0] fb_data;     // RGB666
reg fb_we;

wire overlay = ~key;
reg frame_end, frame_end_r;
reg overlay_r;
reg overlay_we;
reg [7:0] overlay_x, overlay_y;
reg [14:0] overlay_color;
reg [17:0] overlay_data;
reg [2:0] overlay_cnt;
reg vsync;

pll_27 pll_27_inst(
    .clkout0(clk_27),
    .clkin(clk_g),
    .lock(pll_lock_27)
);

ddr3_framebuffer #(
    .WIDTH(640),
    .HEIGHT(480),
    .COLOR_BITS(18),
    .PREFETCH_DELAY(44)             // larger than default 40 to avoid flickering at VGA resolution
) fb (
    .clk_27(clk_27),
    .pll_lock_27(pll_lock_27),
    .clk_g(clk_g),
    .clk_out(clk_x1),
    .rst_n(rst_n),
    .ddr_rst(ddr_rst),
    .init_calib_complete(init_calib_complete),
    
    // Framebuffer interface
    .clk(clk_g),
    .fb_width(overlay ? 256 : 640),
    .fb_height(overlay ? 224 : 480),
    .disp_width(overlay ? 1080 : 960),
    .fb_vsync(vsync),
    .fb_we(overlay ? overlay_we : fb_we),
    .fb_data(overlay ? overlay_data : fb_data),
    
    // DDR3 interface
    .ddr_addr(ddr_addr),
    .ddr_bank(ddr_bank),
    .ddr_cs(ddr_cs),
    .ddr_ras(ddr_ras),
    .ddr_cas(ddr_cas),
    .ddr_we(ddr_we),
    .ddr_ck(ddr_ck),
    .ddr_ck_n(ddr_ck_n),
    .ddr_cke(ddr_cke),
    .ddr_odt(ddr_odt),
    .ddr_reset_n(ddr_reset_n),
    .ddr_dm(ddr_dm),
    .ddr_dq(ddr_dq),
    .ddr_dqs(ddr_dqs),
    .ddr_dqs_n(ddr_dqs_n),
    
    // HDMI output
    .tmds_clk_n(tmds_clk_n),
    .tmds_clk_p(tmds_clk_p),
    .tmds_d_n(tmds_d_n),
    .tmds_d_p(tmds_d_p)
);

// rendering to framebuffer
// one pixel per 4 cycles
// - cycles per frame: 640*480*4 = 1.228M, about 60.5 fps
reg [5:0] bg_r = 0, bg_g = 63, bg_b = 32;
wire [17:0] bg_color = {bg_r, bg_g, bg_b};
reg [7:0] delay;
reg [9:0] block_x = 0, block_y = 4;
reg [9:0] wr_x, wr_y;
reg [15:0] frame_cnt;

//`define FIXED_BLOCK

reg [2:0] pattern;

localparam PATTERN_GRID = 0;
localparam PATTERN_GRADIENT = 1;
localparam PATTERN_SOLID = 2;

// RGB5
localparam GREY = 18'b100000_100000_100000;
localparam RED = 18'b111111_000000_000000;
localparam GREEN = 18'b000000_111111_000000;
localparam BLUE = 18'b000000_000000_111111;

always @(posedge clk_g) begin           // test use 50Mhz to drive updates
    if (ddr_rst) begin
        delay <= 3;
        wr_x <= 0; wr_y <= 0;
        fb_we <= 0;
    end else if (!overlay) begin
        vsync <= 0;
        fb_we <= 0;
        delay <= delay - 1;

        // generate vsync
        if (delay == 1 && wr_x == 639 && wr_y == 479) begin
            frame_cnt <= frame_cnt + 1;
            vsync <= 1;
        end

        // generate pixel writes and cursor movement
        if (delay == 0) begin
            // output pixel
            wr_x <= wr_x + 1;
            if (wr_x == 639) begin
                wr_x <= 0;
                wr_y <= wr_y + 1;
                if (wr_y == 479) 
                    wr_y <= 0;
            end
            fb_we <= 1;

            // move block every frame
`ifndef FIXED_BLOCK
            if (wr_x == 0 && wr_y == 0) begin 
                block_x <= block_x + 1;
                if (block_x == 640/8-1) begin
                    block_x <= 0;
                    block_y <= block_y + 4;
                    if (block_y >= 480/8-1) begin
                        block_y <= 0;
                        pattern <= pattern + 1;
                        if (pattern == 2)
                            pattern <= 0;
                    end
                end
                if (frame_cnt[2:0] == 0) begin  // change color every 8 frames
                    bg_r <= bg_r + 1;
                    bg_g <= bg_g + 2;
                    bg_b <= bg_b + 3;
                end
            end
`endif
            delay <= 3;
        end
    end else begin
        overlay_r <= overlay;
        vsync <= 0;
        overlay_we <= 0;
        
        // send overlay data to framebuffer
        // 50Mhz, 8 cycles per pixel -> 109fps
        overlay_cnt <= overlay_cnt + 1;
        case (overlay_cnt)
        0: if (overlay_x == 0 && overlay_y == 0)
            vsync <= 1;
        4: begin
            overlay_we <= 1;
            // BGR5 to RGB6
            overlay_data <= {overlay_color[4:0], 1'b0, overlay_color[14:10], 1'b0, overlay_color[9:5], 1'b0};
            overlay_x <= overlay_x + 1;
            if (overlay_x == 255) begin
                overlay_y <= overlay_y + 1;
                if (overlay_y == 223) 
                    overlay_y <= 0;
            end
        end
        default: ;
        endcase
    end
end

always @* begin
    if (   wr_y[9:3] >= block_y && wr_y[9:3] < block_y+4 
        && wr_x[9:3] >= block_x && wr_x[9:3] < block_x+4) begin
        fb_data = GREEN;
    end else case (pattern)
        PATTERN_SOLID: 
            fb_data = bg_color;
        PATTERN_GRADIENT:
            fb_data = {wr_x[8:3], wr_y[8:3], (wr_x[8:3] + wr_y[8:3])};
        PATTERN_GRID:
            fb_data = (wr_x[3:0] == 0 || wr_y[3:0] == 0) ? {18{1'b1}} : 18'h0;
        default:
            fb_data = (wr_x[3:0] == 0 || wr_y[3:0] == 0) ? {18{1'b1}} : 18'h0;
    endcase    
end

always @* begin
    if (overlay_x == 0 || overlay_x == 255 || overlay_y == 0 || overlay_y == 223)
        overlay_color = 15'b11111_11111_11111;
    else
        overlay_color = 15'b0;
end

assign leds = ~{6'b0, ~key, init_calib_complete};

endmodule
