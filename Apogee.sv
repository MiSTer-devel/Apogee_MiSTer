//============================================================================
// Sinclair ZX Spectrum host board
// 
//  Port to DE10-nano board. 
//  Copyright (C) 2017 Sorgelig
//
//  Based on sample ZX Spectrum code by Goran Devic
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//============================================================================
module emu
(
	//Master input clock
	input         CLK_50M,

	//Async reset from top-level module.
	//Can be used as initial reset.
	input         RESET,

	//Must be passed to hps_io module
	inout  [37:0] HPS_BUS,

	//Base video clock. Usually equals to CLK_SYS.
	output        CLK_VIDEO,

	//Multiple resolutions are supported using different CE_PIXEL rates.
	//Must be based on CLK_VIDEO
	output        CE_PIXEL,

	//Video aspect ratio for HDMI. Most retro systems have ratio 4:3.
	output  [7:0] VIDEO_ARX,
	output  [7:0] VIDEO_ARY,

	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,    // = ~(VBlank | HBlank)

	output        LED_USER,  // 1 - ON, 0 - OFF.

	// b[1]: 0 - LED status is system status ORed with b[0]
	//       1 - LED status is controled solely by b[0]
	// hint: supply 2'b00 to let the system control the LED.
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,

	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S, // 1 - signed audio samples, 0 - unsigned
	input         TAPE_IN,

	//High latency DDR3 RAM interface
	//Use for non-critical time purposes
	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	//SDRAM interface with lower latency
	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE
);

assign AUDIO_S   = 0;

assign LED_USER  = ioctl_download | ioctl_erasing;
assign LED_DISK  = 0;
assign LED_POWER = 0;

assign VIDEO_ARX = status[8] ? 8'd16 : 8'd4;
assign VIDEO_ARY = status[8] ? 8'd9  : 8'd3;
assign CLK_VIDEO = clk_sys;

assign {DDRAM_CLK, DDRAM_BURSTCNT, DDRAM_ADDR, DDRAM_DIN, DDRAM_BE, DDRAM_RD, DDRAM_WE} = 0;

///////////////////   HPS I/O   //////////////////
wire [31:0] status;
wire  [1:0] buttons;
wire        forced_scandoubler;
wire        ps2_kbd_clk, ps2_kbd_data;

wire        ioctl_wr;
wire [24:0] ioctl_addr;
wire  [7:0] ioctl_data;
wire        ioctl_download;
wire        ioctl_erasing;
wire  [7:0] ioctl_index;

`include "build_id.v"
localparam CONF_STR = 
{
	"APOGEE;;",
	"-;",
	"F,RKARKRGAM;",
	"O5,Autostart,Yes,No;",
	"-;",
	"O1,Color,On,Off;",
	"O8,Aspect ratio,4:3,16:9;",
	"O23,Scandoubler Fx,None,CRT 25%,CRT 50%,CRT 75%;",
	"-;",
	"O4,CPU speed,Normal,Double;",
	"T6,Reset;",
	"V,v2.50.",`BUILD_DATE
};

hps_io #(.STRLEN(($size(CONF_STR)>>3))) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
   .conf_str(CONF_STR),

   .buttons(buttons),
   .forced_scandoubler(forced_scandoubler),

   .status(status),

	.ioctl_download(ioctl_download),
	.ioctl_erasing(ioctl_erasing),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_data),

   .ps2_kbd_clk(ps2_kbd_clk),
   .ps2_kbd_data(ps2_kbd_data),
	
	.ps2_kbd_led_use(0),
	.ps2_kbd_led_status(0),
	
	.sd_lba(0),
	.sd_rd(0),
	.sd_wr(0),
	.sd_conf(0),
	.sd_buff_din(0)
);


////////////////////   CLOCKS   ///////////////////
wire locked;
wire clk_sys;       // 96Mhz
reg  ce_f1,ce_f2;   // 1.78MHz/3.56MHz
reg  ce_pix;        // 8MHz
reg  ce_pit;        // 1.78MHz

pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys),
	.outclk_1(SDRAM_CLK),
	.locked(locked)
);

always @(negedge clk_sys) begin
	reg [3:0] clk_viddiv;
	reg [6:0] cpu_div = 0;
	reg       turbo = 0;

	clk_viddiv <= clk_viddiv + 1'd1;
	if(clk_viddiv == 11) clk_viddiv <=0;
	ce_pix <= (clk_viddiv == 0);

	cpu_div <= cpu_div + 1'd1;
	if(cpu_div == 53) begin 
		cpu_div <= 0;
		turbo <= status[4];
	end
	ce_f1  <= ((cpu_div == 0)  | (turbo & (cpu_div == 27)));
	ce_f2  <= ((cpu_div == 13) | (turbo & (cpu_div == 40)));
	ce_pit <=  (cpu_div == 8);

	startup <= reset|(startup&~addrbus[15]);
end


////////////////////   RESET   ////////////////////
reg       reset; // = 1;
wire      ext_reset = status[0] | status[6] | buttons[1] | reset_key[0];

always @(posedge clk_sys) begin
	integer   initRESET = 100000000;
	reg [3:0] reset_cnt;

	if ((!ext_reset && reset_cnt==4'd14) && !initRESET && !ioctl_download && !ioctl_erasing)
		reset <= 0;
	else begin
		if(initRESET && !ioctl_download) initRESET <= initRESET - 1;
		reset <= 1;
		reset_cnt <= reset_cnt+4'd1;
	end
end


////////////////////   MEM   ////////////////////
wire  [7:0] ram_dout;
reg   [7:0] ram_din;
reg  [24:0] ram_addr;
reg         ram_we;
reg         ram_rd;

always_comb begin
	casex({ioctl_download | ioctl_erasing, hlda})
		2'b1X:
			begin
				ram_din  <= ioctl_data;
				ram_addr <= ioctl_addr;
				ram_we   <= ioctl_wr;
				ram_rd   <= 0;
			end
		2'b01:
			begin
				ram_din  <= 0;
				ram_addr <= vid_addr;
				ram_we   <= 0;
				ram_rd   <= !dma_rd_n;
			end
		2'b00:
			begin
				ram_din  <= cpu_o;
				ram_addr <= addr;
				ram_we   <= !cpu_wr_n && !ppa2_a_acc;
				ram_rd   <= cpu_rd;
			end
	endcase
end

sdram ram
( 
	.*,
	.clk(clk_sys),
	.init(!locked),
	.dout(ram_dout),
	.din(ram_din),
	.addr(ram_addr),
	.we(ram_we),
	.rd(ram_rd),
	.ready()
);

wire [24:0] addr = ppa2_a_acc ? {3'b100, extaddr} : addrbus;

wire [7:0] rom_o;
bios   rom(.address({addrbus[11]|startup,addrbus[10:0]}), .clock(clk_sys), .q(rom_o));

wire [7:0] rom86_o;
bios86 rom86(.address(addrbus[10:0]), .clock(clk_sys), .q(rom86_o));


////////////////////   CPU   ////////////////////
wire [15:0] addrbus;
reg   [7:0] cpu_i;
wire  [7:0] cpu_o;
wire        cpu_sync;
wire        cpu_rd;
wire        cpu_wr_n;
wire        cpu_int = 0;
wire        cpu_inta_n;
wire        inte;
reg         startup;

reg mode86 = 0;
always @(posedge clk_sys) begin
	reg old_download;
	old_download <= ioctl_download;
	if(~old_download & ioctl_download) mode86 <= (ioctl_index>1);
	if(reset_key) mode86 <= reset_key[2];
end

reg ppa1_sel, ppa2_sel, pit_sel, crt_sel, dma_sel;
always_comb begin
	ppa1_sel =0;
	ppa2_sel =0;
	pit_sel  =0;
	crt_sel  =0;
	dma_sel  =0;
	casex({startup, mode86, addrbus[15:8]})

		// Apogee
		10'b0011101100: begin cpu_i <= pit_o;   pit_sel  <= 1; end
		10'b0011101101: begin cpu_i <= ppa1_o;  ppa1_sel <= 1; end
		10'b0011101110: begin cpu_i <= ppa2_o;  ppa2_sel <= 1; end
		10'b0011101111: begin cpu_i <= crt_o;   crt_sel  <= 1; end
		10'b001111XXXX: begin cpu_i <= rom_o;   dma_sel  <= !addrbus[11:8];   end
		10'b10XXXXXXXX: begin cpu_i <= rom_o;                  end

		// Radio
		10'b01100XXXXX: begin cpu_i <= ppa1_o;  ppa1_sel <= 1; end
		10'b0110100XXX: begin cpu_i <= pit_o;   pit_sel  <= 1; end
		10'b0110101XXX: begin cpu_i <= 0;                      end // sd_o
		10'b011011XXXX: begin cpu_i <= 0;                      end // ???
		10'b01110XXXXX: begin cpu_i <= crt_o;   crt_sel  <= 1; end
		10'b01111XXXXX: begin cpu_i <= rom86_o; dma_sel  <= 1; end
		10'b11XXXXXXXX: begin cpu_i <= rom86_o;                end
		
		default: cpu_i <= ram_dout;
	endcase
end

k580vm80a cpu
(
   .pin_clk(clk_sys),
   .pin_f1(ce_f1),
   .pin_f2(ce_f2),
   .pin_reset(reset),
   .pin_a(addrbus),
   .pin_dout(cpu_o),
   .pin_din(cpu_i),
   .pin_hold(hrq),
   .pin_hlda(hlda),
   .pin_ready(1),
   .pin_wait(),
   .pin_int(cpu_int),
   .pin_inte(inte),
   .pin_sync(cpu_sync),
   .pin_dbin(cpu_rd),
   .pin_wr_n(cpu_wr_n)
);


////////////////////   VIDEO   ////////////////////
wire  [7:0] dma_o;
wire        hlda;
wire        dma_rd_n;
wire        hrq;
wire        vid_drq;
wire        vid_dack;
wire  [7:0] crt_o;
wire [15:0] vid_addr;
wire        pix;
wire        vid_hilight;
wire  [1:0] vid_gattr;
wire  [3:0] bw_pix = {{1{pix}}, {3{pix & vid_hilight}}};
wire  [5:0] R_out, r_out;
wire  [5:0] G_out, g_out;
wire  [5:0] B_out, b_out;
wire        HSync, VSync, HBlank, VBlank, hs_out, vs_out;

k580vt57 dma
(
	.clk(clk_sys), 
	.ce_dma(ce_f2), 
	.reset(reset),
	.iaddr(addrbus[3:0]), 
	.idata(cpu_o), 
	.drq ({1'b0,vid_drq, 2'b00}),
	.dack({1'bZ,vid_dack,2'bZZ}), 
	.iwe_n(~dma_sel | cpu_wr_n), 
	.ird_n(1),
	.hlda(hlda), 
	.hrq(hrq), 
	.odata(dma_o), 
	.oaddr(vid_addr),
	.oiord_n(dma_rd_n) 
);

k580vg75 crt
(
	.clk_sys(clk_sys),
	.ce_pix(ce_pix),
	.iaddr(addrbus[0]),
	.idata(hlda ? ram_dout : cpu_o),
	.odata(crt_o),
	.iwe_n(~crt_sel | cpu_wr_n),
	.ird_n(~crt_sel | ~cpu_rd),
	.drq(vid_drq),
	.dack(vid_dack),
	.vrtc(VSync),
	.hrtc(HSync),
	.vblank(VBlank),
	.hblank(HBlank),
	.pix(pix),
	.hilight(vid_hilight),
	.gattr(vid_gattr),
	.charset(mode86 ? 1'b0 : inte),
	.scr_shift(alt_dir)
);

video_mixer #(.HALF_DEPTH(1)) video_mixer
(
	.*,
	.ce_pix_out(CE_PIXEL),
	.scanlines(status[3:2]),
	.hq2x(0),
	.scandoubler(status[3:2] || forced_scandoubler),
	.mono(0),

	.R(status[1] ? bw_pix : {4{pix & ~vid_hilight }}),
	.G(status[1] ? bw_pix : {4{pix & ~vid_gattr[1]}}),
	.B(status[1] ? bw_pix : {4{pix & ~vid_gattr[0]}})
);


////////////////////   KBD   ////////////////////
wire [7:0] kbd_o;
wire [2:0] kbd_shift;
wire [2:0] reset_key;
wire [4:0] alt_dir;

keyboard keyboard
(
	.clk(clk_sys), 
	.reset(reset),
	.downloading(ioctl_erasing & ~status[5]),
	.ps2_clk(ps2_kbd_clk),
	.ps2_dat(ps2_kbd_data),
	.addr(~ppa1_a), 
	.odata(kbd_o), 
	.shift(kbd_shift),
	.reset_key(reset_key),
	.alt_dir(alt_dir)
);


////////////////////   SYS PPA   ////////////////////
wire [7:0] ppa1_o;
wire [7:0] ppa1_a;
wire [7:0] ppa1_b;
wire [7:0] ppa1_c;

k580vv55 ppa1
(
	.clk_sys(clk_sys), 
	.reset(reset), 
	.addr(addrbus[1:0]), 
	.we_n(~ppa1_sel | cpu_wr_n),
	.idata(cpu_o), 
	.odata(ppa1_o), 
	.ipa(ppa1_a), 
	.opa(ppa1_a),
	.ipb(~kbd_o), 
	.opb(ppa1_b), 
	.ipc({~kbd_shift,tapein,ppa1_c[3:0]}), 
	.opc(ppa1_c)
);


//////////////////   EXTROM PPA   //////////////////
wire  [7:0] ppa2_o;
wire  [7:0] ppa2_b;
wire  [7:0] ppa2_c;

reg   [3:0] tm9;
wire [18:0] extaddr = {tm9, ppa2_c[6:0], ppa2_b};
wire        ppa2_a_acc = !mode86 && ((addrbus[15:8] == 8'hEE) && (addrbus[1:0] == 2'd0));

always @(posedge clk_sys) begin
	reg old_c7;
	old_c7 <= ppa2_c[7];
	if(~old_c7 & ppa2_c[7]) tm9<=ppa2_b[3:0];
end

k580vv55 ppa2
(
	.clk_sys(clk_sys), 
	.reset(reset), 
	.addr(addrbus[1:0]), 
	.we_n(~ppa2_sel | cpu_wr_n),
	.idata(cpu_o), 
	.odata(ppa2_o), 
	.ipa(ram_dout), 
	.ipb(ppa2_b), 
	.opb(ppa2_b), 
	.ipc(ppa2_c), 
	.opc(ppa2_c)
);


////////////////////   SOUND   ////////////////////
wire       tapein = 0;
wire [7:0] pit_o;
wire       pit_out0;
wire       pit_out1;
wire       pit_out2;

k580vi53 pit
(
	.reset(reset),
	.clk_sys(clk_sys),
	.clk_timer({ce_pit,ce_pit,ce_pit}),
	.addr(addrbus[1:0]),
	.wr(pit_sel & ~cpu_wr_n),
	.rd(pit_sel & cpu_rd),
	.din(cpu_o),
	.dout(pit_o),
	.gate(3'b111),
	.out({pit_out2, pit_out1, pit_out0})
);

wire [1:0] sample = ppa1_c[0] + (mode86 & inte) + pit_out0 + pit_out1 + pit_out2;

assign AUDIO_R = {8{sample}};
assign AUDIO_L = {8{sample}};
 
endmodule
