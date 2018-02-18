// ====================================================================
//                Radio-86RK FPGA REPLICA
//
//            Copyright (C) 2011 Dmitry Tselikov
//
// This core is distributed under modified BSD license. 
// For complete licensing information see LICENSE.TXT.
// -------------------------------------------------------------------- 
//
// An open implementation of Radio-86RK keyboard
//
// Author: Dmitry Tselikov   http://bashkiria-2m.narod.ru/
// 
//

module keyboard
(
	input           clk,
	input           reset,
	input           downloading,
	input    [10:0] ps2_key,
	input     [7:0] addr,
	output reg[7:0] odata,
	output    [2:0] shift,
	output reg[2:0] reset_key = 0,
	output reg[4:0] alt_dir
);

reg[7:0] keystate[10:0];
assign shift = keystate[8][2:0];

always @(addr,keystate) begin
	odata =
		(keystate[0] & {8{addr[0]}})|
		(keystate[1] & {8{addr[1]}})|
		(keystate[2] & {8{addr[2]}})|
		(keystate[3] & {8{addr[3]}})|
		(keystate[4] & {8{addr[4]}})|
		(keystate[5] & {8{addr[5]}})|
		(keystate[6] & {8{addr[6]}})|
		(keystate[7] & {8{addr[7]}});
end

reg  [2:0] c;
reg  [3:0] r;

wire [7:0] kcode   = {ps2_key[7:0]};
wire       pressed = ps2_key[9];

always @(*) begin
	case (kcode)
	8'h6C: {c,r} = 7'h00; // KP7 home
	8'h7D: {c,r} = 7'h10; // KP9 pgup
	8'h76: {c,r} = 7'h20; // esc
	8'h05: {c,r} = 7'h30; // F1
	8'h06: {c,r} = 7'h40; // F2
	8'h04: {c,r} = 7'h50; // F3
	8'h0C: {c,r} = 7'h60; // F4
	8'h03: {c,r} = 7'h70; // F5

	8'h0B: {c,r} = 7'h00; // F6 -> home
	8'h83: {c,r} = 7'h10; // F7 -> str

	8'h0D: {c,r} = 7'h01; // tab
	8'h71: {c,r} = 7'h11; // . del
	8'h5A: {c,r} = 7'h21; // enter
	8'h66: {c,r} = 7'h31; // bksp
	8'h6B: {c,r} = 7'h41; // KP4 left
	8'h75: {c,r} = 7'h51; // KP8 up
	8'h74: {c,r} = 7'h61; // KP6 right
	8'h72: {c,r} = 7'h71; // KP2 down

	8'h45: {c,r} = 7'h02; // 0
	8'h16: {c,r} = 7'h12; // 1
	8'h1E: {c,r} = 7'h22; // 2
	8'h26: {c,r} = 7'h32; // 3
	8'h25: {c,r} = 7'h42; // 4
	8'h2E: {c,r} = 7'h52; // 5
	8'h36: {c,r} = 7'h62; // 6
	8'h3D: {c,r} = 7'h72; // 7

	8'h3E: {c,r} = 7'h03; // 8
	8'h46: {c,r} = 7'h13; // 9
	8'h55: {c,r} = 7'h23; // =
	8'h0E: {c,r} = 7'h33; // `
	8'h41: {c,r} = 7'h43; // ,
	8'h4E: {c,r} = 7'h53; // -
	8'h49: {c,r} = 7'h63; // .
	8'h4A: {c,r} = 7'h73; // gray/ + /

	8'h4C: {c,r} = 7'h04; // ;
	8'h1C: {c,r} = 7'h14; // A
	8'h32: {c,r} = 7'h24; // B
	8'h21: {c,r} = 7'h34; // C
	8'h23: {c,r} = 7'h44; // D
	8'h24: {c,r} = 7'h54; // E
	8'h2B: {c,r} = 7'h64; // F
	8'h34: {c,r} = 7'h74; // G

	8'h33: {c,r} = 7'h05; // H
	8'h43: {c,r} = 7'h15; // I
	8'h3B: {c,r} = 7'h25; // J
	8'h42: {c,r} = 7'h35; // K
	8'h4B: {c,r} = 7'h45; // L
	8'h3A: {c,r} = 7'h55; // M
	8'h31: {c,r} = 7'h65; // N
	8'h44: {c,r} = 7'h75; // O

	8'h4D: {c,r} = 7'h06; // P
	8'h15: {c,r} = 7'h16; // Q
	8'h2D: {c,r} = 7'h26; // R
	8'h1B: {c,r} = 7'h36; // S
	8'h2C: {c,r} = 7'h46; // T
	8'h3C: {c,r} = 7'h56; // U
	8'h2A: {c,r} = 7'h66; // V
	8'h1D: {c,r} = 7'h76; // W

	8'h22: {c,r} = 7'h07; // X
	8'h35: {c,r} = 7'h17; // Y
	8'h1A: {c,r} = 7'h27; // Z
	8'h54: {c,r} = 7'h37; // [
	8'h52: {c,r} = 7'h47; // '
	8'h5B: {c,r} = 7'h57; // ]
	8'h5D: {c,r} = 7'h67; // \!
	8'h29: {c,r} = 7'h77; // space

	8'h12: {c,r} = 7'h08; // lshift
	8'h59: {c,r} = 7'h08; // rshift
	8'h14: {c,r} = 7'h18; // rctrl + lctrl
	8'h58: {c,r} = 7'h28; // caps

	default: {c,r} = 7'h7F;
	endcase
end

reg [7:0] auto[40] = '{
	255,
	0,0,0,0,
	{1'b1, 7'h26}, // R
	{1'b0, 7'h26}, // R
	{1'b1, 7'h02}, // 0
	{1'b0, 7'h02}, // 0
	{1'b1, 7'h43}, // ,
	{1'b0, 7'h43}, // ,
	{1'b1, 7'h12}, // 1
	{1'b0, 7'h12}, // 1
	{1'b1, 7'h02}, // 0
	{1'b0, 7'h02}, // 0
	{1'b1, 7'h21}, // enter
	{1'b0, 7'h21}, // enter
	0,0,
	{1'b1, 7'h74}, // G
	{1'b0, 7'h74}, // G
	{1'b1, 7'h21}, // enter
	{1'b0, 7'h21}, // enter
	255,255,255,
	0,0,0,0,0,0,0,0,
	{1'b1, 7'h74}, // G
	{1'b0, 7'h74}, // G
	{1'b1, 7'h21}, // enter
	{1'b0, 7'h21}, // enter
	255,255
};

reg auto_strobe;
reg [5:0] auto_pos = 0;
always @(negedge clk) begin
	integer div;
	div <= div + 1;
	auto_strobe <=0;
	if(div == 7000000) begin 
		div <=0;
		auto_strobe <=1;
	end
end

reg malt   = 0;
reg mctrl  = 0;
reg mshift = 0;

always @(posedge clk) begin
	reg old_reset, old_reset_key;
	reg old_downloading;
	reg old_stb;

	old_stb <= ps2_key[10];
	
	old_reset <= reset;
	if(!old_reset && reset) begin
		keystate[0] <= 0;
		keystate[1] <= 0;
		keystate[2] <= 0;
		keystate[3] <= 0;
		keystate[4] <= 0;
		keystate[5] <= 0;
		keystate[6] <= 0;
		keystate[7] <= 0;
		keystate[8] <= 0;
		keystate[9] <= 0;
		keystate[10]<= 0;
	end else begin
		if(auto[auto_pos] == 255) begin
			if (old_stb != ps2_key[10]) begin
				if (kcode==8'h11) malt   <= pressed;
				if (kcode==8'h14) mctrl  <= pressed;
				if (kcode==8'h12) mshift <= pressed;
				if (kcode==8'h59) mshift <= pressed;
				if (kcode==8'h78) reset_key <= (pressed) ? {malt, mshift, mctrl | mshift | malt} : 3'b0;
				if(!malt) begin
					if(r!=4'hF) keystate[r][c] <= pressed;
				end else begin
					case(kcode)
						8'h6B: alt_dir[0] <= pressed; // left
						8'h74: alt_dir[1] <= pressed; // right
						8'h75: alt_dir[2] <= pressed; // up
						8'h72: alt_dir[3] <= pressed; // down
						8'h5A: alt_dir[4] <= pressed; // enter - reset
					 default:;
					endcase
				end
			end
		end else if(auto_strobe) begin
			mshift <=0;
			mctrl  <=0;
			if(auto[auto_pos]) keystate[auto[auto_pos][3:0]][auto[auto_pos][6:4]] <= auto[auto_pos][7];
			auto_pos <= auto_pos + 1'd1;
		end

		if(old_reset_key   && !reset_key[1]) auto_pos <=1;
		if(old_downloading && !downloading)  auto_pos <=26;
		old_reset_key <= reset_key[1];
		old_downloading <= downloading;
	end
end

endmodule
