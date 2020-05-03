module DE10_Nano_Computer (
	////////////////////////////////////
	// FPGA Pins
	////////////////////////////////////

	// Clock pins
	CLOCK1_50,
	CLOCK2_50,
	CLOCK3_50,
	CLOCK4_50,

	// ADC
	ADC_CS_N,
	ADC_DIN,
	ADC_DOUT,
	ADC_SCLK,

	// SDRAM
	DRAM_ADDR,
	DRAM_BA,
	DRAM_CAS_N,
	DRAM_CKE,
	DRAM_CLK,
	DRAM_CS_N,
	DRAM_DQ,
	DRAM_LDQM,
	DRAM_RAS_N,
	DRAM_UDQM,
	DRAM_WE_N,

	// Pushbuttons
	KEY,

	// LEDs
	LEDR,

	// Slider Switches
	SW,

	////////////////////////////////////
	// HPS Pins
	////////////////////////////////////
	
	// DDR3 SDRAM
	HPS_DDR3_ADDR,
	HPS_DDR3_BA,
	HPS_DDR3_CAS_N,
	HPS_DDR3_CKE,
	HPS_DDR3_CK_N,
	HPS_DDR3_CK_P,
	HPS_DDR3_CS_N,
	HPS_DDR3_DM,
	HPS_DDR3_DQ,
	HPS_DDR3_DQS_N,
	HPS_DDR3_DQS_P,
	HPS_DDR3_ODT,
	HPS_DDR3_RAS_N,
	HPS_DDR3_RESET_N,
	HPS_DDR3_RZQ,
	HPS_DDR3_WE_N,

	// Ethernet
	HPS_ENET_GTX_CLK,
	HPS_ENET_INT_N,
	HPS_ENET_MDC,
	HPS_ENET_MDIO,
	HPS_ENET_RX_CLK,
	HPS_ENET_RX_DATA,
	HPS_ENET_RX_DV,
	HPS_ENET_TX_DATA,
	HPS_ENET_TX_EN,

	// Flash
	HPS_FLASH_DATA,
	HPS_FLASH_DCLK,
	HPS_FLASH_NCSO,

	// Accelerometer
	HPS_GSENSOR_INT,
		
	// General Purpose I/O
	HPS_GPIO,
		
	// I2C
	HPS_I2C_CONTROL,
	HPS_I2C1_SCLK,
	HPS_I2C1_SDAT,
	HPS_I2C2_SCLK,
	HPS_I2C2_SDAT,

	// Pushbutton
	HPS_KEY,

	// LED
	HPS_LED,
		
	// SD Card
	HPS_SD_CLK,
	HPS_SD_CMD,
	HPS_SD_DATA,

	// SPI
	HPS_SPIM_CLK,
	HPS_SPIM_MISO,
	HPS_SPIM_MOSI,
	HPS_SPIM_SS,

	// UART
	HPS_UART_RX,
	HPS_UART_TX,

	// USB
	HPS_CONV_USB_N,
	HPS_USB_CLKOUT,
	HPS_USB_DATA,
	HPS_USB_DIR,
	HPS_USB_NXT,
	HPS_USB_STP
);

//=======================================================
//  PARAMETER declarations
//=======================================================


//=======================================================
//  PORT declarations
//=======================================================

////////////////////////////////////
// FPGA Pins
////////////////////////////////////

// Clock pins
input						CLOCK1_50;
input						CLOCK2_50;
input						CLOCK3_50;
input						CLOCK4_50;

// ADC
inout						ADC_CS_N;
output					ADC_DIN;
input						ADC_DOUT;
output					ADC_SCLK;

// SDRAM
output 		[12: 0]	DRAM_ADDR;
output		[ 1: 0]	DRAM_BA;
output					DRAM_CAS_N;
output					DRAM_CKE;
output					DRAM_CLK;
output					DRAM_CS_N;
inout			[15: 0]	DRAM_DQ;
output					DRAM_LDQM;
output					DRAM_RAS_N;
output					DRAM_UDQM;
output					DRAM_WE_N;

// Pushbuttons
input			[ 1: 0]	KEY;

// LEDs
output		[ 7: 0]	LEDR;

// Slider Switches
input			[ 3: 0]	SW;

////////////////////////////////////
// HPS Pins
////////////////////////////////////
	
// DDR3 SDRAM
output		[14: 0]	HPS_DDR3_ADDR;
output		[ 2: 0]  HPS_DDR3_BA;
output					HPS_DDR3_CAS_N;
output					HPS_DDR3_CKE;
output					HPS_DDR3_CK_N;
output					HPS_DDR3_CK_P;
output					HPS_DDR3_CS_N;
output		[ 3: 0]	HPS_DDR3_DM;
inout			[31: 0]	HPS_DDR3_DQ;
inout			[ 3: 0]	HPS_DDR3_DQS_N;
inout			[ 3: 0]	HPS_DDR3_DQS_P;
output					HPS_DDR3_ODT;
output					HPS_DDR3_RAS_N;
output					HPS_DDR3_RESET_N;
input						HPS_DDR3_RZQ;
output					HPS_DDR3_WE_N;

// Ethernet
output					HPS_ENET_GTX_CLK;
inout						HPS_ENET_INT_N;
output					HPS_ENET_MDC;
inout						HPS_ENET_MDIO;
input						HPS_ENET_RX_CLK;
input			[ 3: 0]	HPS_ENET_RX_DATA;
input						HPS_ENET_RX_DV;
output		[ 3: 0]	HPS_ENET_TX_DATA;
output					HPS_ENET_TX_EN;

// Flash
inout			[ 3: 0]	HPS_FLASH_DATA;
output					HPS_FLASH_DCLK;
output					HPS_FLASH_NCSO;

// Accelerometer
inout						HPS_GSENSOR_INT;

// General Purpose I/O
inout			[ 1: 0]	HPS_GPIO;

// I2C
inout						HPS_I2C_CONTROL;
inout						HPS_I2C1_SCLK;
inout						HPS_I2C1_SDAT;
inout						HPS_I2C2_SCLK;
inout						HPS_I2C2_SDAT;

// Pushbutton
inout						HPS_KEY;

// LED
inout						HPS_LED;

// SD Card
output					HPS_SD_CLK;
inout						HPS_SD_CMD;
inout			[ 3: 0]	HPS_SD_DATA;

// SPI
output					HPS_SPIM_CLK;
input						HPS_SPIM_MISO;
output					HPS_SPIM_MOSI;
inout						HPS_SPIM_SS;

// UART
input						HPS_UART_RX;
output					HPS_UART_TX;

// USB
inout						HPS_CONV_USB_N;
input						HPS_USB_CLKOUT;
inout			[ 7: 0]	HPS_USB_DATA;
input						HPS_USB_DIR;
input						HPS_USB_NXT;
output					HPS_USB_STP;

//=======================================================
// PIO state machine
//=======================================================
// OUTPUTs from the FPGA, INPUT to HPS
wire [31:0] pp_in_axi, pp_in_lw_axi ;

// INPUTS to the FPGA, OUTPUT from HPS
wire [31:0] pp_out_axi, pp_out_lw_axi ;

// for debugging:
// SW[0] chooses to look at AXI input or output on HEX display
// SW[1] chooses input+1 or zero to go back to HPS on AXI bus
// SW[2] chooses input+1 or zero to go back to HPS on light-weight AXI
// assign pp_in_axi = (SW[1])? (pp_out_axi + 32'd1) : 0 ;
// assign pp_in_lw_axi = (SW[2])? (pp_out_lw_axi + 32'd1) : 0 ;

//assign pp_in_axi = (pp_out_axi);
//assign pp_in_lw_axi = (pp_out_lw_axi);

reg clk;

assign clk = CLOCK1_50;

wire we;
wire [9:0] regNum; 
wire [19:0] address;
wire [31:0] dataIn;
reg [31:0] dataOut;

assign we = pp_out_lw_axi[30];
assign regNum = pp_out_lw_axi[29:20]; // 29 28 27 26 25 24 23 22 21 20 >> 10
assign address = pp_out_lw_axi[19:0];
assign dataIn = pp_out_axi;
assign pp_in_axi = dataOut;

reg signed [31:0] inputLayer [1:0];
reg signed [31:0] outputVector [1:0];

reg start = 1'b0;
reg reset = 1'b0;

integer count = 0;

always @ (posedge CLOCK1_50) begin
	if(we == 1) begin
		if(regNum == 0) begin
			inputLayer[address] <= dataIn;
		end
	end
	
	if(we == 0) begin
		if(regNum == 0)
			dataOut <= outputVector[address];
	end

	if(inputLayer[0] == 32'd1)
		input1 <= 17'b01111111111111111;
	else
		input1 <= 17'b00000000000000000;

	if(inputLayer[1] == 32'd1)
		input2 <= 17'b01111111111111111;
	else
		input2 <= 17'b00000000000000000;
		
	posTwoThirds <= 17'b00000011111111111;
	negTwoThirds <= 17'b10000011111111111;	

	bias1 = 17'b10000000000011111;
	bias2 = 17'b01111111111111111;
	bias3 = 17'b11111111111111111;

	//led[8:0] <= out[15:7]; 
	outputVector[0] <= y; //[24:16];
	LEDR[7:0] <= y;
end

wire [32:0] total1 = {33{1'b0}};
wire [32:0] total2 = {33{1'b0}};
wire [32:0] total3 = {33{1'b0}};
reg [16:0] input1;
reg [16:0] input2;
reg [16:0] posTwoThirds;
reg [16:0] negTwoThirds;

/********************** first **********************/
// equation 1: 2/3*x1 + 2/3*x2 - 5 = h1

wire [16:0] m1_to_a1;
matrix m1(
	.clk(clk),
	.in(input1),
	.slope(posTwoThirds),
	.out(m1_to_a1)
);

wire [32:0] total_a1;
add a1(
	.clk(clk),
	.in(m1_to_a1),
	.total(total1),
	.out(total_a1)
);

wire [16:0] m2_to_a2;
matrix m2(
	.clk(clk),
	.in(input2),
	.slope(posTwoThirds),
	.out(m2_to_a2)
);

wire [32:0] total_a2;
add a2(
	.clk(clk),
	.in(m2_to_a2),
	.total(total_a1),
	.out(total_a2)
);

wire [32:0] total_a3;
reg [16:0] bias1;
add a3(
	.clk(clk),
	.in(bias1),
	.total(total_a2),
	.out(total_a3)
);

wire [16:0] sq1_to_sig1;
squish sq1(
	.in(total_a3),
	.out(sq1_to_sig1)
);

wire [16:0] h1;
sigmoid sig1(
	.in(sq1_to_sig1),
	.out(h1)
);

/********************* second *********************/
//  equation 2: -2/3*x1 - 2/3*x2 + 16 = h2

wire [16:0] m3_to_a4;
matrix m3(
	.clk(clk),
	.in(input1),
	.slope(negTwoThirds),
	.out(m3_to_a4)
);

wire [32:0] total_a4;
add a4(
	.clk(clk),
	.in(m3_to_a4),
	.total(total2),
	.out(total_a4)
);

wire [16:0] m4_to_a5;
matrix m4(
	.clk(clk),
	.in(input2),
	.slope(negTwoThirds),
	.out(m4_to_a5)
);

wire [32:0] total_a5;
add a5(
	.clk(clk),
	.in(m4_to_a5),
	.total(total_a4),
	.out(total_a5)
);

wire [32:0] total_a6;
reg [16:0] bias2;
add a6(
	.clk(clk),
	.in(bias2),
	.total(total_a5),
	.out(total_a6)
);

wire [16:0] sq2_to_sig2;
squish sq2(
	.in(total_a6),
	.out(sq2_to_sig2)
);

wire [16:0] h2;
sigmoid sig2(
	.in(sq2_to_sig2),
	.out(h2)
);

/********************* third *********************/
// equation 3: 2/3*h1 + 2/3*h2 - 16 = y

wire [16:0] m5_to_a7;
matrix m5(
	.clk(clk),
	.in(h1),
	.slope(posTwoThirds),
	.out(m5_to_a7)
);

wire [32:0] total_a7;
add a7(
	.clk(clk),
	.in(m5_to_a7),
	.total(total3),
	.out(total_a7)
);

wire [16:0] m6_to_a8;
matrix m6(
	.clk(clk),
	.in(h2),
	.slope(posTwoThirds),
	.out(m6_to_a8)
);

wire [32:0] total_a8;
add a8(
	.clk(clk),
	.in(m6_to_a8),
	.total(total_a7),
	.out(total_a8)
);

wire [32:0] total_a9;
reg [16:0] bias3;
add a9(
	.clk(clk),
	.in(bias3),
	.total(total_a8),
	.out(total_a9)
);

wire [16:0] sq3_to_sig3;
squish sq3(
	.in(total_a9),
	.out(sq3_to_sig3)
);

wire [16:0] y;
sigmoid sig3(
	.in(sq3_to_sig3),
	.out(y)
);


// assign LEDR[0] = SW[0];
// assign LEDR[1] = SW[1];

//=======================================================
//  Structural coding
//=======================================================
// From Qsys

Computer_System The_System (
	////////////////////////////////////
	// FPGA Side
	////////////////////////////////////

	// Global signals
	.system_pll_ref_clk_clk					(CLOCK1_50),
	.system_pll_ref_reset_reset			(1'b0),
	
	////////////////////////////////////
	// PIO ports
	////////////////////////////////////
	.pp_out_axi_export               (pp_out_axi),               
	.pp_in_axi_export                (pp_in_axi),              
	.pp_out_lw_axi_export            (pp_out_lw_axi), 
	.pp_in_lw_axi_export             (pp_in_lw_axi),  

	////////////////////////////////////
	// HPS Side
	////////////////////////////////////
	// DDR3 SDRAM
	.memory_mem_a			(HPS_DDR3_ADDR),
	.memory_mem_ba			(HPS_DDR3_BA),
	.memory_mem_ck			(HPS_DDR3_CK_P),
	.memory_mem_ck_n		(HPS_DDR3_CK_N),
	.memory_mem_cke		(HPS_DDR3_CKE),
	.memory_mem_cs_n		(HPS_DDR3_CS_N),
	.memory_mem_ras_n		(HPS_DDR3_RAS_N),
	.memory_mem_cas_n		(HPS_DDR3_CAS_N),
	.memory_mem_we_n		(HPS_DDR3_WE_N),
	.memory_mem_reset_n	(HPS_DDR3_RESET_N),
	.memory_mem_dq			(HPS_DDR3_DQ),
	.memory_mem_dqs		(HPS_DDR3_DQS_P),
	.memory_mem_dqs_n		(HPS_DDR3_DQS_N),
	.memory_mem_odt		(HPS_DDR3_ODT),
	.memory_mem_dm			(HPS_DDR3_DM),
	.memory_oct_rzqin		(HPS_DDR3_RZQ),
		  
	// Ethernet
	.hps_io_hps_io_gpio_inst_GPIO35	(HPS_ENET_INT_N),
	.hps_io_hps_io_emac1_inst_TX_CLK	(HPS_ENET_GTX_CLK),
	.hps_io_hps_io_emac1_inst_TXD0	(HPS_ENET_TX_DATA[0]),
	.hps_io_hps_io_emac1_inst_TXD1	(HPS_ENET_TX_DATA[1]),
	.hps_io_hps_io_emac1_inst_TXD2	(HPS_ENET_TX_DATA[2]),
	.hps_io_hps_io_emac1_inst_TXD3	(HPS_ENET_TX_DATA[3]),
	.hps_io_hps_io_emac1_inst_RXD0	(HPS_ENET_RX_DATA[0]),
	.hps_io_hps_io_emac1_inst_MDIO	(HPS_ENET_MDIO),
	.hps_io_hps_io_emac1_inst_MDC		(HPS_ENET_MDC),
	.hps_io_hps_io_emac1_inst_RX_CTL	(HPS_ENET_RX_DV),
	.hps_io_hps_io_emac1_inst_TX_CTL	(HPS_ENET_TX_EN),
	.hps_io_hps_io_emac1_inst_RX_CLK	(HPS_ENET_RX_CLK),
	.hps_io_hps_io_emac1_inst_RXD1	(HPS_ENET_RX_DATA[1]),
	.hps_io_hps_io_emac1_inst_RXD2	(HPS_ENET_RX_DATA[2]),
	.hps_io_hps_io_emac1_inst_RXD3	(HPS_ENET_RX_DATA[3]),

	// Flash
	.hps_io_hps_io_qspi_inst_IO0	(HPS_FLASH_DATA[0]),
	.hps_io_hps_io_qspi_inst_IO1	(HPS_FLASH_DATA[1]),
	.hps_io_hps_io_qspi_inst_IO2	(HPS_FLASH_DATA[2]),
	.hps_io_hps_io_qspi_inst_IO3	(HPS_FLASH_DATA[3]),
	.hps_io_hps_io_qspi_inst_SS0	(HPS_FLASH_NCSO),
	.hps_io_hps_io_qspi_inst_CLK	(HPS_FLASH_DCLK),

	// Accelerometer
	.hps_io_hps_io_gpio_inst_GPIO61	(HPS_GSENSOR_INT),

	//.adc_sclk                        (ADC_SCLK),
	//.adc_cs_n                        (ADC_CS_N),
	//.adc_dout                        (ADC_DOUT),
	//.adc_din                         (ADC_DIN),

	// General Purpose I/O
	.hps_io_hps_io_gpio_inst_GPIO40	(HPS_GPIO[0]),
	.hps_io_hps_io_gpio_inst_GPIO41	(HPS_GPIO[1]),

	// I2C
	.hps_io_hps_io_gpio_inst_GPIO48	(HPS_I2C_CONTROL),
	.hps_io_hps_io_i2c0_inst_SDA		(HPS_I2C1_SDAT),
	.hps_io_hps_io_i2c0_inst_SCL		(HPS_I2C1_SCLK),
	.hps_io_hps_io_i2c1_inst_SDA		(HPS_I2C2_SDAT),
	.hps_io_hps_io_i2c1_inst_SCL		(HPS_I2C2_SCLK),

	// Pushbutton
	.hps_io_hps_io_gpio_inst_GPIO54	(HPS_KEY),

	// LED
	.hps_io_hps_io_gpio_inst_GPIO53	(HPS_LED),

	// SD Card
	.hps_io_hps_io_sdio_inst_CMD	(HPS_SD_CMD),
	.hps_io_hps_io_sdio_inst_D0	(HPS_SD_DATA[0]),
	.hps_io_hps_io_sdio_inst_D1	(HPS_SD_DATA[1]),
	.hps_io_hps_io_sdio_inst_CLK	(HPS_SD_CLK),
	.hps_io_hps_io_sdio_inst_D2	(HPS_SD_DATA[2]),
	.hps_io_hps_io_sdio_inst_D3	(HPS_SD_DATA[3]),

	// SPI
	.hps_io_hps_io_spim1_inst_CLK		(HPS_SPIM_CLK),
	.hps_io_hps_io_spim1_inst_MOSI	(HPS_SPIM_MOSI),
	.hps_io_hps_io_spim1_inst_MISO	(HPS_SPIM_MISO),
	.hps_io_hps_io_spim1_inst_SS0		(HPS_SPIM_SS),

	// UART
	.hps_io_hps_io_uart0_inst_RX	(HPS_UART_RX),
	.hps_io_hps_io_uart0_inst_TX	(HPS_UART_TX),

	// USB
	.hps_io_hps_io_gpio_inst_GPIO09	(HPS_CONV_USB_N),
	.hps_io_hps_io_usb1_inst_D0		(HPS_USB_DATA[0]),
	.hps_io_hps_io_usb1_inst_D1		(HPS_USB_DATA[1]),
	.hps_io_hps_io_usb1_inst_D2		(HPS_USB_DATA[2]),
	.hps_io_hps_io_usb1_inst_D3		(HPS_USB_DATA[3]),
	.hps_io_hps_io_usb1_inst_D4		(HPS_USB_DATA[4]),
	.hps_io_hps_io_usb1_inst_D5		(HPS_USB_DATA[5]),
	.hps_io_hps_io_usb1_inst_D6		(HPS_USB_DATA[6]),
	.hps_io_hps_io_usb1_inst_D7		(HPS_USB_DATA[7]),
	.hps_io_hps_io_usb1_inst_CLK		(HPS_USB_CLKOUT),
	.hps_io_hps_io_usb1_inst_STP		(HPS_USB_STP),
	.hps_io_hps_io_usb1_inst_DIR		(HPS_USB_DIR),
	.hps_io_hps_io_usb1_inst_NXT		(HPS_USB_NXT)
);
endmodule // end top level

//============================================================
// M10K module for testing
//============================================================
// See example 12-16 in 
// http://people.ece.cornell.edu/land/courses/ece5760/DE1_SOC/HDL_style_qts_qii51007.pdf
//============================================================

module M10K_256_32( 
    output reg [31:0] q,
    input [31:0] d,
    input [255:0] write_address, read_address,
    input we, clk
);
	 // force M10K ram style
	 // 256 words of 32 bits
    reg [31:0] mem [255:0]  /* synthesis ramstyle = "no_rw_check, M10K" */;
	 
    always @ (posedge clk) begin
        if (we) begin
            mem[write_address] <= d;
		  end
		  q <= mem[read_address]; // q doesn't get d in this clock cycle
    end
endmodule

//============================================================
// MLAB module for testing
//============================================================
// See example 12-16 in 
// http://people.ece.cornell.edu/land/courses/ece5760/DE1_SOC/HDL_style_qts_qii51007.pdf
//============================================================

//module MLAB_20_32(
//	output reg signed [31:0] q,
//	input  [31:0] data,
//	input [7:0] readaddr, writeaddr,
//	input wren, clock
//);
//	// force MLAB ram style
//	// 20 words of 32 bits
//	reg signed [31:0] mem [19:0] /* synthesis ramstyle = "no_rw_check, MLAB" */;
//	
//	always @ (posedge clock)
//	begin
//		if (wren) begin
//			mem[writeaddr] <= data;
//		end
//		q <= mem[readaddr];
//	end
//endmodule

//endmodule

/// end /////////////////////////////////////////////////////////////////////