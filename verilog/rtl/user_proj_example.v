// SPDX-FileCopyrightText: 2020 Efabless Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0

`default_nettype none
/*
 *-------------------------------------------------------------
 *
 * user_proj_example
 *
 * This is an example of a (trivially simple) user project,
 * showing how the user project can connect to the logic
 * analyzer, the wishbone bus, and the I/O pads.
 *
 * This project generates an integer count, which is output
 * on the user area GPIO pads (digital output only).  The
 * wishbone connection allows the project to be controlled
 * (start and stop) from the management SoC program.
 *
 * See the testbenches in directory "mprj_counter" for the
 * example programs that drive this user project.  The three
 * testbenches are "io_ports", "la_test1", and "la_test2".
 *
 *-------------------------------------------------------------
 */

module user_proj_example #(
    parameter BITS = 16
)(
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // IOs
    input  [BITS-1:0] io_in,
    output [BITS-1:0] io_out,
    output [BITS-1:0] io_oeb,

    // IRQ
    output [2:0] irq
);
    wire clk;
    wire rst;

    wire [BITS-1:0] rdata; 
    wire [BITS-1:0] wdata;
    wire [BITS-1:0] count;

    wire valid;
    wire [3:0] wstrb;
    wire [BITS-1:0] la_write;

    // WB MI A
    assign valid = wbs_cyc_i && wbs_stb_i; 
    assign wstrb = wbs_sel_i & {4{wbs_we_i}};
    assign wbs_dat_o = {{(32-BITS){1'b0}}, rdata};
    assign wdata = wbs_dat_i[BITS-1:0];

    // IO
    assign io_out = MIPI_para;
    assign io_oeb = {(BITS){rst}};

    // IRQ
    assign irq = 3'b000;	// Unused

    // LA
    assign la_data_out = {{(128-BITS){1'b0}}, count};
    // Assuming LA probes [63:32] are for controlling the count register  
    assign la_write = ~la_oenb[63:64-BITS] & ~{BITS{valid}};
    // Assuming LA probes [65:64] are for controlling the count clk & reset  
    assign clk = (~la_oenb[64]) ? la_data_in[64]: wb_clk_i;
    assign rst = (~la_oenb[65]) ? la_data_in[65]: wb_rst_i;

    mipi_rx_raw10_select #(
        .BITS(BITS)
    ) mipi_rx_raw10_select(
        .wb_clk_i(wb_clk_i),
        .reset(rst),
        .valid(wbs_stb_i),
        .wstrb(wstrb),
        .data_i(data_i)
        .wdata(wbs_dat_i[BITS-1:0]),
        .la_write(la_write),
        .la_input(la_data_in[63:64-BITS]),
        .ready(ready)
        .rdata(rdata),
        .output_valid_o(output_valid_o)
        .output_o(output_o)
    );

endmodule

module mipi_rx_raw10_select #(
    parameter BITS = 16
)(          
    .wb_clk_i(wb_clk_i),
    .reset(reset),
    .wbs_stb_i(wbs_stb_i),
    .wstrb(wstrb),
    .data_i(data_i),
    .wdata(wdata),
    .la_write(la_write),
    .la_write(la_write),
    .ready(ready),
    .rdata(rdata),
    .output_valid_o(output_valid_o),
    .output_o(output_o);   

localparam [2:0]BYTES_PERPACK = 3'h5; // RAW 10 is packed <Sample0[9:2]> <Sample1[9:2]> <Sample2[9:2]> <Sample3[9:2]> <Sample0[1:0],Sample1[1:0],Sample2[1:0],Sample3[1:0]>
input clk_i;
input reset,
input data_valid_i;
input [3:0] wstrb,
input [31:0]data_i;
input [12:11] wdata
input [BITS-1:0] la_write,
input [BITS-1:0] la_write,
output reg ready,
output reg [BITS-1:0] rdata,
output reg output_valid_o;
output reg [39:0]output_o; 

reg [7:0]offset;
reg [2:0]byte_count;
reg [31:0]last_data_i;
wire [63:0]word;

//add by ISP_intergratot//
assign data_i = wdata
assign clk_i = clk
assign reset = rst 
assign data_valid_i= valid

assign word = {data_i,last_data_i}; //would need last bytes as well as current data to get full 4 pixel

always @(posedge clk_i)
begin
	
	if (data_valid_i)
	begin
		last_data_i <= data_i;
		//RAW 10 , Byte1 -> Byte2 -> Byte3 -> Byte4 -> [ LSbB1[1:0] LSbB2[1:0] LSbB3[1:0] LSbB4[1:0] ]
		output_o[39:30] <= 	{word [(offset + 7) -:8], 	word [(offset + 39) -:2]}; 		//lane 1
		output_o[29:20] <= 	{word [(offset + 15) -:8], 	word [(offset + 37) -:2]};		
		output_o[19:10] <= 	{word [(offset + 23) -:8], 	word [(offset + 35) -:2]};
		output_o[9:0] 	<= 	{word [(offset + 31) -:8], 	word [(offset  + 33) -:2]};		//lane 4
		
		if (byte_count < (BYTES_PERPACK))
		begin
			byte_count <= byte_count + 1'd1;
			if (byte_count )
			begin
				offset <= ((offset + 8'd8) & 8'h1F);
				output_valid_o <= 1'h1;
			end
		end
		else
		begin
			
			offset <= 8'h0;
			byte_count <= 4'b1;		//this byte is the first byte
			output_valid_o <= 1'h0;
		end
	end
	else
	begin
		output_o <= 40'h0;
		last_data_i <= 1'h0;
		offset <= 8'h0;
		byte_count <= 3'b0;
		output_valid_o <= 1'h0;
	end
end

endmodule

`default_nettype wire
