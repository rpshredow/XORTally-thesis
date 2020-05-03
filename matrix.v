module matrix(
	input clk,
	input [16:0] in,
	input [16:0] slope,
	output reg [16:0] out
);

reg [15:0] matrix [15:0];
reg [15:0] temp;
reg finish = 1'b0;
reg start = 1'b1;
wire [15:0] input1;
integer i;
assign input1 = in[15:0];

initial begin
	for(i = 0; i < 16; i = i + 1) begin
		matrix[i] = {16{1'b0}};
	end
end

wire [15:0] slope_in;

assign slope_in = slope[15:0];

always @ (posedge clk) begin		
		case(slope_in)
		16'b0000000000000001: begin
			matrix[0] = 16'b1111111111111111;
			end
		16'b0000000000000011: begin
			matrix[1] = 16'b1111111100000000;
			matrix[0] = 16'b0000000011111111;
			end
		16'b0000000000000111: begin
			matrix[2] = 16'b0011111000000000;
			matrix[1] = 16'b0000000111111000;
			matrix[0] = 16'b0000000000000111;
			end
		16'b0000000000001111: begin
			matrix[3] = 16'b0111100000000000;
			matrix[2] = 16'b0000011110000000;
			matrix[1] = 16'b0000000001111000;
			matrix[0] = 16'b0000000000000111;
			end
		16'b0000000000011111: begin
			matrix[4] = 16'b0111000000000000;
			matrix[3] = 16'b0000111000000000;
			matrix[2] = 16'b0000000111100000;
			matrix[1] = 16'b0000000000011100;
			matrix[0] = 16'b0000000000000011;
			end
		16'b0000000000111111: begin
			matrix[5] = 16'b0110000000000000;
			matrix[4] = 16'b0001110000000000;
			matrix[3] = 16'b0000001110000000;
			matrix[2] = 16'b0000000001100000;
			matrix[1] = 16'b0000000000011100;
			matrix[0] = 16'b0000000000000011;
			end
		16'b0000000001111111: begin
			matrix[6] = 16'b0110000000000000;
			matrix[5] = 16'b0001100000000000;
			matrix[4] = 16'b0000011000000000;
			matrix[3] = 16'b0000000111000000;
			matrix[2] = 16'b0000000000110000;
			matrix[1] = 16'b0000000000001100;
			matrix[0] = 16'b0000000000000011;
			end
		16'b0000000011111111: begin // 8
			matrix[7] = 16'b1100000000000000;
			matrix[6] = 16'b0011000000000000;
			matrix[5] = 16'b0000110000000000;
			matrix[4] = 16'b0000001100000000;
			matrix[3] = 16'b0000000011000000;
			matrix[2] = 16'b0000000000110000;
			matrix[1] = 16'b0000000000001100;
			matrix[0] = 16'b0000000000000011;
			end
		16'b0000000111111111: begin // 9
			matrix[8] = 16'b1100000000000000;
			matrix[7] = 16'b0011000000000000;
			matrix[6] = 16'b0000110000000000;
			matrix[5] = 16'b0000001000000000;
			matrix[4] = 16'b0000000110000000;
			matrix[3] = 16'b0000000001100000;
			matrix[2] = 16'b0000000000011000;
			matrix[1] = 16'b0000000000000110;
			matrix[0] = 16'b0000000000000001;
			end
		16'b0000001111111111: begin
			matrix[9] = 16'b1100000000000000;
			matrix[8] = 16'b0010000000000000;
			matrix[7] = 16'b0001100000000000;
			matrix[6] = 16'b0000011000000000;
			matrix[5] = 16'b0000000100000000;
			matrix[4] = 16'b0000000011000000;
			matrix[3] = 16'b0000000000100000;
			matrix[2] = 16'b0000000000011000;
			matrix[1] = 16'b0000000000000110;
			matrix[0] = 16'b0000000000000001;
			end
		16'b0000011111111111: begin
			matrix[10] = 16'b1100000000000000;
			matrix[9] = 16'b0010000000000000;
			matrix[8] = 16'b0001100000000000;
			matrix[7] = 16'b0000010000000000;
			matrix[6] = 16'b0000001000000000;
			matrix[5] = 16'b0000000110000000;
			matrix[4] = 16'b0000000001000000;
			matrix[3] = 16'b0000000000110000;
			matrix[2] = 16'b0000000000001000;
			matrix[1] = 16'b0000000000000110;
			matrix[0] = 16'b0000000000000001;
			end
		16'b0000111111111111: begin
			matrix[11] = 16'b1000000000000000;
			matrix[10] = 16'b0110000000000000;
			matrix[9] = 16'b0001000000000000;
			matrix[8] = 16'b0000100000000000;
			matrix[7] = 16'b0000011000000000;
			matrix[6] = 16'b0000000100000000;
			matrix[5] = 16'b0000000010000000;
			matrix[4] = 16'b0000000001100000;
			matrix[3] = 16'b0000000000010000;
			matrix[2] = 16'b0000000000001000;
			matrix[1] = 16'b0000000000000110;
			matrix[0] = 16'b0000000000000001;
			end
		16'b0001111111111111: begin
			matrix[12] = 16'b1000000000000000;
			matrix[11] = 16'b0110000000000000;
			matrix[10] = 16'b0001000000000000;
			matrix[9] = 16'b0000100000000000;
			matrix[8] = 16'b0000010000000000;
			matrix[7] = 16'b0000001000000000;
			matrix[6] = 16'b0000000110000000;
			matrix[5] = 16'b0000000001000000;
			matrix[4] = 16'b0000000000100000;
			matrix[3] = 16'b0000000000010000;
			matrix[2] = 16'b0000000000001100;
			matrix[1] = 16'b0000000000000010;
			matrix[0] = 16'b0000000000000001;
			end
		16'b0011111111111111: begin
			matrix[13] = 16'b1000000000000000;
			matrix[12] = 16'b0100000000000000;
			matrix[11] = 16'b0010000000000000;
			matrix[10] = 16'b0001100000000000;
			matrix[9] = 16'b0000010000000000;
			matrix[8] = 16'b0000001000000000;
			matrix[7] = 16'b0000000100000000;
			matrix[6] = 16'b0000000010000000;
			matrix[5] = 16'b0000000001000000;
			matrix[4] = 16'b0000000000100000;
			matrix[3] = 16'b0000000000011000;
			matrix[2] = 16'b0000000000000100;
			matrix[1] = 16'b0000000000000010;
			matrix[0] = 16'b0000000000000001;
			end
		16'b0111111111111111: begin
			matrix[14] = 16'b1000000000000000;
			matrix[13] = 16'b0100000000000000;
			matrix[12] = 16'b0010000000000000;
			matrix[11] = 16'b0001000000000000;
			matrix[10] = 16'b0000100000000000;
			matrix[9] = 16'b0000010000000000;
			matrix[8] = 16'b0000001000000000;
			matrix[7] = 16'b0000000110000000;
			matrix[6] = 16'b0000000001000000;
			matrix[5] = 16'b0000000000100000;
			matrix[4] = 16'b0000000000010000;
			matrix[3] = 16'b0000000000001000;
			matrix[2] = 16'b0000000000000100;
			matrix[1] = 16'b0000000000000010;
			matrix[0] = 16'b0000000000000001;
			end
		16'b1111111111111111: begin
			matrix[15] = 16'b1000000000000000;
			matrix[14] = 16'b0100000000000000;
			matrix[13] = 16'b0010000000000000;
			matrix[12] = 16'b0001000000000000;
			matrix[11] = 16'b0000100000000000;
			matrix[10] = 16'b0000010000000000;
			matrix[9] = 16'b0000001000000000;
			matrix[8] = 16'b0000000100000000;
			matrix[7] = 16'b0000000010000000;
			matrix[6] = 16'b0000000001000000;
			matrix[5] = 16'b0000000000100000;
			matrix[4] = 16'b0000000000010000;
			matrix[3] = 16'b0000000000001000;
			matrix[2] = 16'b0000000000000100;
			matrix[1] = 16'b0000000000000010;
			matrix[0] = 16'b0000000000000001;
		end
	endcase
	
		temp[0] = |(input1 & matrix[0]);
		temp[1] = |(input1 & matrix[1]);
		temp[2] = |(input1 & matrix[2]);
		temp[3] = |(input1 & matrix[3]);
		temp[4] = |(input1 & matrix[4]);
		temp[5] = |(input1 & matrix[5]);
		temp[6] = |(input1 & matrix[6]);
		temp[7] = |(input1 & matrix[7]);
		temp[8] = |(input1 & matrix[8]);
		temp[9] = |(input1 & matrix[9]);
		temp[10] = |(input1 & matrix[10]);
		temp[11] = |(input1 & matrix[11]);
		temp[12] = |(input1 & matrix[12]);
		temp[13] = |(input1 & matrix[13]);
		temp[14] = |(input1 & matrix[14]);
		temp[15] = |(input1 & matrix[15]);
		
		out[15:0] = temp;
	   out[16] = in[16] ^ slope[16];
end

endmodule