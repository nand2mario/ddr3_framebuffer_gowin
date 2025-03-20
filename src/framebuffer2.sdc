
create_clock -name clkin -period 20 -waveform {0 10} [get_ports {clk_g}]
create_clock -name clk4x -period 3.367 -waveform {0 1.684} [get_pins {fb/pll_ddr3_inst/PLLA_inst/CLKOUT2}]
create_clock -name clk1x -period 13.47 -waveform {0 6.734} [get_pins {fb/u_ddr3/gw3_top/u_ddr_phy_top/fclkdiv/CLKOUT}]
set_clock_groups -asynchronous -group [get_clocks {clkin}] -group [get_clocks {clk4x}]
set_clock_groups -asynchronous -group [get_clocks {clk4x}] -group [get_clocks {clk1x}]
set_clock_groups -asynchronous -group [get_clocks {clkin}] -group [get_clocks {clk1x}]
