module i2c_master #(
	parameter integer CLK_DIV = 250,
	parameter integer MAX_BYTES = 16
) (
	input wire clk,
	input wire rst,

	input wire start,
	input wire rw,
	input wire [6:0] addr,

	input wire reg_addr_valid,
	input wire [7:0] reg_addr,

	input wire [7:0] write_len,
	input wire [7:0] read_len,
	input wire [MAX_BYTES * 8 - 1:0] write_data,

	output reg [7:0] data_out,
	output reg [MAX_BYTES * 8 - 1:0] read_data,

	output reg busy,
	output reg done,
	output reg ack_error,

	inout wire sda,
	inout wire scl
);

	localparam MAX_BYTES_U8 = MAX_BYTES;

	localparam STAGE_ADDR_W = 0;
	localparam STAGE_REG = 1;
	localparam STAGE_ADDR_R = 2;
	localparam STAGE_WRITE_DATA = 3;
	localparam STAGE_READ_DATA = 4;

    localparam I2C_WRITE = 1'b0;
    localparam I2C_READ = 1'b1;

	localparam IDLE = 0;
	localparam WAIT_BUS_FREE = 1;
	localparam START_DRIVE_SDA_LOW = 2;
	localparam START_PULL_SCL_LOW = 3;
	localparam LOAD_WRITE_BYTE = 4;
	localparam WRITE_BIT_SETUP = 5;
	localparam WRITE_BIT_RAISE_SCL = 6;
	localparam WRITE_BIT_WAIT_SCL_HIGH = 7;
	localparam WRITE_BIT_HOLD = 8;
	localparam WRITE_ACK_RELEASE = 9;
	localparam WRITE_ACK_RAISE_SCL = 10;
	localparam WRITE_ACK_WAIT_SCL_HIGH = 11;
	localparam WRITE_ACK_SAMPLE = 12;
	localparam RESTART_RELEASE_SDA = 13;
	localparam RESTART_RAISE_SCL = 14;
	localparam RESTART_WAIT_SCL_HIGH = 15;
	localparam RESTART_DRIVE_SDA_LOW = 16;
	localparam RESTART_PULL_SCL_LOW = 17;
	localparam READ_BIT_RELEASE = 18;
	localparam READ_BIT_RAISE_SCL = 19;
	localparam READ_BIT_WAIT_SCL_HIGH = 20;
	localparam READ_BIT_SAMPLE = 21;
	localparam READ_STORE_BYTE = 22;
	localparam READ_ACK_SETUP = 23;
	localparam READ_ACK_RAISE_SCL = 24;
	localparam READ_ACK_WAIT_SCL_HIGH = 25;
	localparam READ_ACK_HOLD = 26;
	localparam STOP_DRIVE_SDA_LOW = 27;
	localparam STOP_RAISE_SCL = 28;
	localparam STOP_WAIT_SCL_HIGH = 29;
	localparam STOP_RELEASE_SDA = 30;
	localparam DONE_ST = 31;

	reg [15:0] clk_cnt;
	reg tick;

	reg start_d;
	reg start_req;
	wire start_pulse;

	reg sda_oe_low;
	reg scl_oe_low;

	wire sda_raw;
	wire scl_raw;

	reg sda_meta;
	reg sda_sync;
	reg scl_meta;
	reg scl_sync;

	reg [5:0] state;
	reg [2:0] stage;
	reg [3:0] bit_cnt;
	reg [7:0] shift_reg;
	reg [7:0] tx_idx;
	reg [7:0] rx_idx;

	reg rw_latched;
	reg [6:0] addr_latched;
	reg reg_addr_valid_latched;
	reg [7:0] reg_addr_latched;
	reg [7:0] write_len_latched;
	reg [7:0] read_len_latched;
	reg [MAX_BYTES * 8 - 1:0] write_data_latched;

	assign start_pulse = start & ~start_d;

	assign sda = sda_oe_low ? 1'b0 : 1'bz;
	assign scl = scl_oe_low ? 1'b0 : 1'bz;

	assign sda_raw = sda;
	assign scl_raw = scl;

	always @(posedge clk or posedge rst) begin
		if (rst) begin
			clk_cnt <= 0;
			tick <= 0;
		end else if (clk_cnt == CLK_DIV - 1) begin
			clk_cnt <= 0;
			tick <= 1;
		end else begin
			clk_cnt <= clk_cnt + 1;
			tick <= 0;
		end
	end

	always @(posedge clk or posedge rst) begin
		if (rst)
			start_d <= 0;
		else
			start_d <= start;
	end

	always @(posedge clk or posedge rst) begin
		if (rst) begin
			start_req <= 0;
		end else begin
			if (start_pulse)
				start_req <= 1;
			else if (tick && state == IDLE && start_req)
				start_req <= 0;
		end
	end

	always @(posedge clk or posedge rst) begin
		if (rst) begin
			sda_meta <= 1;
			sda_sync <= 1;
			scl_meta <= 1;
			scl_sync <= 1;
		end else begin
			sda_meta <= sda_raw;
			sda_sync <= sda_meta;
			scl_meta <= scl_raw;
			scl_sync <= scl_meta;
		end
	end

	always @(posedge clk or posedge rst) begin
		if (rst) begin
			state <= IDLE;
			stage <= STAGE_ADDR_W;
			bit_cnt <= 0;
			shift_reg <= 0;
			tx_idx <= 0;
			rx_idx <= 0;

			rw_latched <= 0;
			addr_latched <= 0;
			reg_addr_valid_latched <= 0;
			reg_addr_latched <= 0;
			write_len_latched <= 0;
			read_len_latched <= 0;
			write_data_latched <= 0;

			data_out <= 0;
			read_data <= 0;

			busy <= 0;
			done <= 0;
			ack_error <= 0;

			sda_oe_low <= 0;
			scl_oe_low <= 0;
		end else if (tick) begin
			case (state)

				IDLE: begin
					busy <= 0;
					done <= 0;
					ack_error <= 0;
					sda_oe_low <= 0;
					scl_oe_low <= 0;

					if (start_req) begin
						rw_latched <= rw;
						addr_latched <= addr;
						reg_addr_valid_latched <= reg_addr_valid;
						reg_addr_latched <= reg_addr;
						write_data_latched <= write_data;

						data_out <= 0;
						read_data <= 0;
						tx_idx <= 0;
						rx_idx <= 0;

						if (write_len > MAX_BYTES_U8)
							write_len_latched <= MAX_BYTES_U8;
						else
							write_len_latched <= write_len;

						if (read_len > MAX_BYTES_U8)
							read_len_latched <= MAX_BYTES_U8;
						else
							read_len_latched <= read_len;

						if (rw) begin
							if (reg_addr_valid)
								stage <= STAGE_ADDR_W;
							else
								stage <= STAGE_ADDR_R;
						end else begin
							stage <= STAGE_ADDR_W;
						end

						busy <= 1;
						state <= WAIT_BUS_FREE;
					end
				end

				WAIT_BUS_FREE: begin
					sda_oe_low <= 0;
					scl_oe_low <= 0;

					if (sda_sync && scl_sync)
						state <= START_DRIVE_SDA_LOW;
				end

				START_DRIVE_SDA_LOW: begin
					sda_oe_low <= 1;
					scl_oe_low <= 0;
					state <= START_PULL_SCL_LOW;
				end

				START_PULL_SCL_LOW: begin
					sda_oe_low <= 1;
					scl_oe_low <= 1;
					state <= LOAD_WRITE_BYTE;
				end

				LOAD_WRITE_BYTE: begin
					scl_oe_low <= 1;

					case (stage)
						STAGE_ADDR_W: begin
							shift_reg <= {addr_latched, I2C_WRITE};
							bit_cnt <= 7;
							state <= WRITE_BIT_SETUP;
						end

						STAGE_REG: begin
							shift_reg <= reg_addr_latched;
							bit_cnt <= 7;
							state <= WRITE_BIT_SETUP;
						end

						STAGE_ADDR_R: begin
							shift_reg <= {addr_latched, I2C_READ};
							bit_cnt <= 7;
							state <= WRITE_BIT_SETUP;
						end

						STAGE_WRITE_DATA: begin
							shift_reg <= write_data_latched[tx_idx * 8 +: 8];
							bit_cnt <= 7;
							state <= WRITE_BIT_SETUP;
						end

						default: begin
							state <= STOP_DRIVE_SDA_LOW;
						end
					endcase
				end

				WRITE_BIT_SETUP: begin
					scl_oe_low <= 1;

					if (shift_reg[bit_cnt] == 0)
						sda_oe_low <= 1;
					else
						sda_oe_low <= 0;

					state <= WRITE_BIT_RAISE_SCL;
				end

				WRITE_BIT_RAISE_SCL: begin
					scl_oe_low <= 0;
					state <= WRITE_BIT_WAIT_SCL_HIGH;
				end

				WRITE_BIT_WAIT_SCL_HIGH: begin
					if (scl_sync)
						state <= WRITE_BIT_HOLD;
				end

				WRITE_BIT_HOLD: begin
					scl_oe_low <= 1;

					if (bit_cnt == 0)
						state <= WRITE_ACK_RELEASE;
					else begin
						bit_cnt <= bit_cnt - 1;
						state <= WRITE_BIT_SETUP;
					end
				end

				WRITE_ACK_RELEASE: begin
					sda_oe_low <= 0;
					scl_oe_low <= 1;
					state <= WRITE_ACK_RAISE_SCL;
				end

				WRITE_ACK_RAISE_SCL: begin
					scl_oe_low <= 0;
					state <= WRITE_ACK_WAIT_SCL_HIGH;
				end

				WRITE_ACK_WAIT_SCL_HIGH: begin
					if (scl_sync)
						state <= WRITE_ACK_SAMPLE;
				end

				WRITE_ACK_SAMPLE: begin
					scl_oe_low <= 1;

					if (sda_sync != 0) begin
						ack_error <= 1;
						sda_oe_low <= 1;
						state <= STOP_DRIVE_SDA_LOW;
					end else begin
						case (stage)
							STAGE_ADDR_W: begin
								if (reg_addr_valid_latched) begin
									stage <= STAGE_REG;
									state <= LOAD_WRITE_BYTE;
								end else if (rw_latched) begin
									stage <= STAGE_ADDR_R;
									state <= RESTART_RELEASE_SDA;
								end else if (write_len_latched != 0) begin
									stage <= STAGE_WRITE_DATA;
									tx_idx <= 0;
									state <= LOAD_WRITE_BYTE;
								end else begin
									sda_oe_low <= 1;
									state <= STOP_DRIVE_SDA_LOW;
								end
							end

							STAGE_REG: begin
								if (rw_latched) begin
									stage <= STAGE_ADDR_R;
									state <= RESTART_RELEASE_SDA;
								end else if (write_len_latched != 0) begin
									stage <= STAGE_WRITE_DATA;
									tx_idx <= 0;
									state <= LOAD_WRITE_BYTE;
								end else begin
									sda_oe_low <= 1;
									state <= STOP_DRIVE_SDA_LOW;
								end
							end

							STAGE_ADDR_R: begin
								if (read_len_latched != 0) begin
									stage <= STAGE_READ_DATA;
									bit_cnt <= 7;
									state <= READ_BIT_RELEASE;
								end else begin
									sda_oe_low <= 1;
									state <= STOP_DRIVE_SDA_LOW;
								end
							end

							STAGE_WRITE_DATA: begin
								if (tx_idx + 1 < write_len_latched) begin
									tx_idx <= tx_idx + 1;
									state <= LOAD_WRITE_BYTE;
								end else begin
									sda_oe_low <= 1;
									state <= STOP_DRIVE_SDA_LOW;
								end
							end

							default: begin
								sda_oe_low <= 1;
								state <= STOP_DRIVE_SDA_LOW;
							end
						endcase
					end
				end

				RESTART_RELEASE_SDA: begin
					scl_oe_low <= 1;
					sda_oe_low <= 0;
					state <= RESTART_RAISE_SCL;
				end

				RESTART_RAISE_SCL: begin
					scl_oe_low <= 0;
					state <= RESTART_WAIT_SCL_HIGH;
				end

				RESTART_WAIT_SCL_HIGH: begin
					if (scl_sync)
						state <= RESTART_DRIVE_SDA_LOW;
				end

				RESTART_DRIVE_SDA_LOW: begin
					sda_oe_low <= 1;
					scl_oe_low <= 0;
					state <= RESTART_PULL_SCL_LOW;
				end

				RESTART_PULL_SCL_LOW: begin
					scl_oe_low <= 1;
					state <= LOAD_WRITE_BYTE;
				end

				READ_BIT_RELEASE: begin
					scl_oe_low <= 1;
					sda_oe_low <= 0;
					state <= READ_BIT_RAISE_SCL;
				end

				READ_BIT_RAISE_SCL: begin
					scl_oe_low <= 0;
					state <= READ_BIT_WAIT_SCL_HIGH;
				end

				READ_BIT_WAIT_SCL_HIGH: begin
					if (scl_sync)
						state <= READ_BIT_SAMPLE;
				end

				READ_BIT_SAMPLE: begin
					shift_reg[bit_cnt] <= sda_sync;
					scl_oe_low <= 1;

					if (bit_cnt == 0)
						state <= READ_STORE_BYTE;
					else begin
						bit_cnt <= bit_cnt - 1;
						state <= READ_BIT_RELEASE;
					end
				end

				READ_STORE_BYTE: begin
					read_data[rx_idx * 8 +: 8] <= shift_reg;
					data_out <= shift_reg;
					state <= READ_ACK_SETUP;
				end

				READ_ACK_SETUP: begin
					scl_oe_low <= 1;

					if (rx_idx + 1 < read_len_latched)
						sda_oe_low <= 1;
					else
						sda_oe_low <= 0;

					state <= READ_ACK_RAISE_SCL;
				end

				READ_ACK_RAISE_SCL: begin
					scl_oe_low <= 0;
					state <= READ_ACK_WAIT_SCL_HIGH;
				end

				READ_ACK_WAIT_SCL_HIGH: begin
					if (scl_sync)
						state <= READ_ACK_HOLD;
				end

				READ_ACK_HOLD: begin
					scl_oe_low <= 1;

					if (rx_idx + 1 < read_len_latched) begin
						rx_idx <= rx_idx + 1;
						bit_cnt <= 7;
						state <= READ_BIT_RELEASE;
					end else begin
						sda_oe_low <= 1;
						state <= STOP_DRIVE_SDA_LOW;
					end
				end

				STOP_DRIVE_SDA_LOW: begin
					scl_oe_low <= 1;
					sda_oe_low <= 1;
					state <= STOP_RAISE_SCL;
				end

				STOP_RAISE_SCL: begin
					scl_oe_low <= 0;
					state <= STOP_WAIT_SCL_HIGH;
				end

				STOP_WAIT_SCL_HIGH: begin
					if (scl_sync)
						state <= STOP_RELEASE_SDA;
				end

				STOP_RELEASE_SDA: begin
					sda_oe_low <= 0;
					scl_oe_low <= 0;
					state <= DONE_ST;
				end

				DONE_ST: begin
					busy <= 0;
					done <= 1;
					state <= IDLE;
				end

				default: begin
					state <= IDLE;
					busy <= 0;
					done <= 0;
					ack_error <= 0;
					sda_oe_low <= 0;
					scl_oe_low <= 0;
				end
			endcase
		end
	end

endmodule
