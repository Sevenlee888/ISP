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
    //parameter BITS = 16  ,disable on CT-14-2023
    parameter BITS = 32
)(
`ifdef USE_POWER_PINS
    inout vdda1,	// User area 1 3.3V supply
    inout vdda2,	// User area 2 3.3V supply
    inout vssa1,	// User area 1 analog ground
    inout vssa2,	// User area 2 analog ground
    inout vccd1,	// User area 1 1.8V supply
    inout vccd2,	// User area 2 1.8v supply
    inout vssd1,	// User area 1 digital ground
    inout vssd2,	// User area 2 digital ground
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
    input  [38-1:0] io_in,
    output [38-1:0] io_out,
    output [38-1:0] io_oeb,
 
    input   user_clock2,
    // IRQ
    output [2:0] irq
);
    // assign clk & rst OCT-15.2023
    assign clk = user_clock2;
    assign rst = wb_rst_i;

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

    // LA, disable count 10-15.2023
  // assign la_data_out = {{(128-BITS){1'b0}}, count};
    // Assuming LA probes [63:32] are for controlling the count register  
//    assign la_write = ~la_oenb[63:64-BITS] & ~{BITS{valid}};
 
    // Assuming LA probes [65:64] are for controlling the count clk & reset  
    assign clk = (~la_oenb[64]) ? la_data_in[64]: wb_clk_i;
    assign rst = (~la_oenb[65]) ? la_data_in[65]: wb_rst_i;
   assign la_data_out = 128'h0000_0000_0000_0000_0000_0000_0000_0000;
   
    wire  w_uart_txd ;
    wire  w_uart_rxd ;
   
   
    wire   w_spi_clk ;
    wire   w_spi_miso  ; 
    wire   w_spi_mosi  ;
    wire   [1:0] w_spi_ssel;
   
    wire  [19:0] w_gpi, w_gpo, w_gpd;
    
    wire  [1:0] w_pwm_o;
   
    wire  w_sda_i,w_scl_i,w_sda_o,w_scl_o,w_sda_t,w_scl_t;
    
    wire  w_lrclk,w_bclk,w_audo;
   
   wire w_ledcntrl;
   
   wire w_pwrled;
   
   wire w_uart_txd_dir,w_uart_rxd_dir;
   
   wire [4:0] w_spi_dir;
   
   wire w_scl_dir, w_sda_dir;
   
   wire [1:0] w_pwm_dir;
   wire w_led_dir;
   wire w_i2s_dir;
   
   wire ext_aud_mclk;
   
   wire jtag_mux;
   wire qspi_mux;
   
   wire TMS,TDI,TDO,TCK;
   
   wire q_io0_o;
   wire q_io1_o;
   wire q_io2_o;
   wire q_io3_o;
   
   wire q_io0_t;
   wire q_io1_t;
   wire q_io2_t;
   wire q_io3_t;
   
   wire q_io0_i;
   wire q_io1_i;
   wire q_io2_i;
   wire q_io3_i;
   
   wire q_spi_clk_o;
   wire q_spi_clk_t;
   
   wire q_spi_ssel_o;
   wire q_spi_ssel_t;

    // io_in ({io_in[37:27]}),gi
  
   // gpio mapping   gpio is muxed along with qspi and jtag
  // assign io_out[9:0]   = w_gpo[9:0];
  // assign io_oeb[9:0]   = w_gpd;
  // assign w_gpi[9:0]    = io_in[9:0] ;
   
  // assign io_out[13:10]  = jtag_mux ?  {1'b0,TMS,TDO,TCK}  : w_gpo[13:10] ;
  // assign io_oeb[13:10]  = jtag_mux ? 4'b1000 : w_gpd[13:10];
  // assign TDI            = jtag_mux ? io_in[13] : 1'b1;
  // assign w_gpi[13:10]   = jtag_mux ? 4'b0000 : io_in[13:10];
   
   assign io_out[19:14]  = qspi_mux ? {q_io3_o,q_io2_o,q_io1_o,q_io0_o,q_spi_clk_o,q_spi_ssel_o} : w_gpo[19:14];
   assign io_oeb[19:14]  = qspi_mux ? {q_io3_t,q_io2_t,q_io1_t,q_io0_t,q_spi_clk_t,q_spi_ssel_t} : w_gpd[19:14];
   assign w_gpi[19:14]   = qspi_mux ? 6'b000000 : io_in[19:14];
   
   assign q_io3_i = io_in[19] ;
   assign q_io2_i = io_in[18] ;
   assign q_io1_i = io_in[17] ;
   assign q_io0_i = io_in[16] ;
   
   
   
   
   
   // spi mapping
   
 //  assign io_out[24:20] = {w_spi_mosi,1'b1,w_spi_clk,w_spi_ssel};
 //  assign io_oeb[24:20] = w_spi_dir;
 //  assign w_spi_miso    = io_in[23];
   
   // i2c mapping
   assign io_out[26:25] = {w_sda_o,w_scl_o};
   assign io_oeb[26]    = w_sda_t ^ w_sda_dir; // the control from i2c will be inverted if w_sda_dir = 1
   assign io_oeb[25]    = w_scl_t ^ w_scl_dir;
   assign w_sda_i       = io_in[26];
   assign w_scl_i       = io_in[25];
   
   // i2s mapping
   assign io_out[29:27] = {w_lrclk,w_bclk,w_audo};
   assign io_oeb[29:27] = {w_i2s_dir,w_i2s_dir,w_i2s_dir};
   
   //pwm mapping
   
   assign io_out[31:30] = w_pwm_o;
   assign io_oeb[31:30] = {w_pwm_dir,w_pwm_dir};
   
   // led control
 //  assign io_out[32]    = w_ledcntrl;
 //  assign io_oeb[32]    = w_led_dir;
   
 //  assign io_out[33]    = 1'b1;
 //  assign io_oeb[33]    = !w_i2s_dir;
 //  assign ext_aud_mclk  = io_in[33];   // external master audio clock
   
   //uart mapping
 //  assign io_oeb[35:34] = {w_uart_rxd_dir,w_uart_txd_dir};
  // assign io_out[35:34] = {1'b1,w_uart_txd};
  // assign w_uart_rxd = io_in[35];

        //(UART0 rX,tX),disable port (6:5), oct-17-2023
    //.io_in ({io_in[17:16]})    //change to port (17:16)

   // power led
 //  assign io_out[36]    = w_pwrled;
 //  assign io_oeb[36]    = 1'b0;
   
   assign io_out[37] = 0;
   assign io_oeb[37] = 0;
   
  peripheral_top  u_peripheral_top(
    .wb_clk_i    (wb_clk_i ),
    .wb_rst_i    (wb_rst_i ),
    .wbs_stb_i   (wbs_stb_i),
    .wbs_cyc_i   (wbs_cyc_i),
    .wbs_we_i    (wbs_we_i ),
    .wbs_sel_i   (wbs_sel_i),
    .wbs_dat_i   (wbs_dat_i),
    .wbs_adr_i   (wbs_adr_i),
    .wbs_ack_o   (wbs_ack_o),
    .wbs_dat_o   (wbs_dat_o),
    
    .uart_txd    (w_uart_txd),
    .uart_rxd    (w_uart_rxd),
     
    .spi_clk     (w_spi_clk ),
    .spi_miso    (w_spi_miso),
    .spi_mosi    (w_spi_mosi),
    .spi_ssel    (w_spi_ssel),
    
    .gpi         (w_gpi    ),
    .gpo         (w_gpo    ),
    .gpd         (w_gpd    ),
                
    .pwm_o       (w_pwm_o),
    
    .sda_i       (w_sda_i),
    .scl_i       (w_scl_i),
    
    .sda_o       (w_sda_o),
    .scl_o       (w_scl_o),
    
    .sda_t       (w_sda_t),
    .scl_t       (w_scl_t),
    
    .ext_aud_mclk    (ext_aud_mclk),
    .lrclk       (w_lrclk),
    .bclk        (w_bclk),
    .audo        (w_audo),
    
    .ledcntrl    (w_ledcntrl),
    .pwrled      (w_pwrled),
    
    .uart_txd_dir (w_uart_txd_dir),
    .uart_rxd_dir (w_uart_rxd_dir),
    
    .spi_dir      (w_spi_dir),
    
    .scl_dir      (w_scl_dir),
    .sda_dir      (w_sda_dir),
    
    .pwm_dir      (w_pwm_dir),
    
    .led_dir      (w_led_dir),
    
    .i2s_dir      (w_i2s_dir),
    
   
   
 .q_io0_i(q_io0_i), 
 .q_io0_o(q_io0_o), 
 .q_io0_t(q_io0_t), 
 .q_io1_i(q_io1_i), 
 .q_io1_o(q_io1_o), 
 .q_io1_t(q_io1_t), 
 .q_io2_i(q_io2_i), 
 .q_io2_o(q_io2_o), 
 .q_io2_t(q_io2_t), 
 .q_io3_i(q_io3_i), 
 .q_io3_o(q_io3_o), 
 .q_io3_t(q_io3_t), 
 .q_spi_clk_i (1'b1 ), 
 .q_spi_clk_o (q_spi_clk_o ),  
 .q_spi_clk_t (q_spi_clk_t ), 
 .q_spi_ssel_i(1'b1),
 .q_spi_ssel_o(q_spi_ssel_o),
 .q_spi_ssel_t(q_spi_ssel_t),
   
  .TMS  	(TMS),
  .TCK   	(TCK),
  .TDO   	(TDO),
  .TDI 	(TDI),  
    .jtag_mux     (jtag_mux),
    .qspi_mux     (qspi_mux)
   
    );



//(MIPI_clk_N,MIPI_clk_P),(MIPI_D1_N,MIPI_D1_P),(MIPI_D0_N,MIPI_D0_P),

// Module "mipi_rx_raw10_select" replace by modle "mipi_csi_16_nx"
    mipi_csi_16_nx #
    (   .reset_in(rst),
		.mipi_clk_p_in(io_in[14:14]), //(MIPI_clk_N),
		.mipi_clk_n_in(io_in[15:15]),//(MIPI_clk_P,),
		.mipi_data_p_in(io_in[10:10]),//(MIPI_D0_N),
		.mipi_data_n_in(io_in[11:11]),//(MIPI_D0_P),
        .mipi_clk_p_in1(io_in[14:14]),//(MIPI_clk_N),
		.mipi_clk_n_in1(io_in[15:15]),//(MIPI_clk_P,),
		.mipi_data_p_in1(io_in[12:12]),//(,MIPI_D1_N),
		.mipi_data_n_in1(io_in[13:13]),//(MIPI_D1_P),
		.dummy_out(rdata),

		.pclk_o(pclk_o),  //data output on pos edge , should be latching into receiver on negedge
		.data_o(data_o),
		.fsync_o(fsync_o), //active high 
		.lsync_o(lsync_o), //active high
						
	//these pins may or many not be needed depeding on hardware config
		.cam_ctrl_in(cam_ctrl_in), //control camera control input from host
		.cam_pwr_en_o(cam_pwr_en_o), //enable camera power 
		.cam_reset_o(cam_reset_o),  //camera reset to camera
		.cam_xmaster_o(cam_xmaster_o) //camera master or slave 
	);
endmodule

//    mipi_rx_raw10_select #(
//        .BITS(BITS)
//    ) mipi_rx_raw10_select(
//       .wb_clk_i(wb_clk_i),
//        .reset(rst),
//        .valid(wbs_stb_i),
//        .wstrb(wstrb),
//        .data_i(data_i)
//        .wdata(wbs_dat_i[BITS-1:0]),
//        .la_write(la_write),
//        .la_input(la_data_in[63:64-BITS]),
//        .ready(ready)
//        .rdata(rdata),
//        .output_valid_o(output_valid_o)
//        .output_o(output_o)
//    );

//endmodule
// Module replace by modle "csi_dphy"
module mipi_rx_raw10_select #(
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
    .output_o(output_o)
);   

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
