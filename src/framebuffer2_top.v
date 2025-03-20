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

wire clk_27, clk_x1;
wire pll_lock_27;
reg [18:0] fb_addr;     // 640*480
reg [4*18-1:0] fb_data; // 4x RGB666
reg fb_we;

pll_27 pll_27_inst(
    .clkout0(clk_27),
    .clkin(clk_g),
    .lock(pll_lock_27)
);

ddr3_framebuffer #(
    .WIDTH(640),
    .HEIGHT(480),
    .COLOR_BITS(18),
    .DISP_WIDTH(960)
) fb (
    .clk_27(clk_27),
    .pll_lock(pll_lock_27),
    .clk_out(clk_x1),
    
    // Framebuffer interface
    .clk(clk_x1),
    .fb_width(640),
    .fb_height(480),
    .fb_vsync(vsync),
    .fb_we(fb_we),
    .fb_data(fb_data),
    
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
reg [7:0] cursor_delay;
reg [9:0] cursor_x, cursor_y;
reg [9:0] wr_x, wr_y;
reg [15:0] frame_cnt;

always @(posedge clk_x1) begin
    if (ddr_rst) begin
        cursor_delay <= 3;
        wr_x <= 0; wr_y <= 0;
        write_pixels_req <= 0;
    end else begin
        fb_vsync <= 0;

        cursor_delay <= cursor_delay - 1;
        if (cursor_delay == 0) begin
            // output pixel
            if (wr_x == 0 && wr_y == 0 && ~fb_vsync) begin
                frame_cnt <= frame_cnt + 1;
                fb_vsync <= 1;
            end else begin
                wr_x <= wr_x + 1;
                if (wr_x == 639) begin
                    wr_x <= 0;
                    wr_y <= wr_y + 1;
                    if (wr_y == 479) 
                        wr_y <= 0;
                end
                fb_we <= 1;
                if (   wr_y[9:3] >= cursor_y && wr_y[9:3] < cursor_y+4 
                    && wr_x[9:3] >= cursor_x && wr_x[9:3] < cursor_x+4) begin
                    fb_data <= GREEN;
                end else
                    fb_data <= bg_color;
            end

            // move block every frame
            if (wr_x == 0 && wr_y == 0) begin 
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
            
            cursor_delay <= 3;
        end
    end
end


endmodule
