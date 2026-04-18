module top (
	input wire clk,
	input wire rst_btn,
	input wire start_btn,

	inout wire i2c_scl,
	inout wire i2c_sda,

	output wire [15:0] distance_mm,
	output wire busy,
	output wire done,
	output wire error
);

	reg start_btn_d;
	wire start_pulse;
	wire rst;

	assign rst = rst_btn;
	assign start_pulse = start_btn & ~start_btn_d;

	always @(posedge clk or posedge rst) begin
		if (rst)
			start_btn_d <= 0;
		else
			start_btn_d <= start_btn;
	end

	vl53l0x_controller #(
		.MAX_BYTES(16),
		.GPIO1_ACTIVE_LEVEL(1'b1)
	) vl53 (
		.clk(clk),
		.rst(rst),
		.start(start_pulse),
		.scl(i2c_scl),
		.sda(i2c_sda),
		.distance_mm(distance_mm),
		.busy(busy),
		.done(done),
		.error(error)
	);

endmodule