module vl53l0x_controller #(
    parameter integer MAX_BYTES = 16,
    parameter integer POLL_DELAY_CYCLES = 50_000,    // ~1 ms at 50 MHz
    parameter integer TIMEOUT_CYCLES    = 5_000_000  // ~100 ms at 50 MHz
) (
    input  wire clk,
    input  wire rst,
    input  wire start,

    inout  wire scl,
    inout  wire sda,

    output reg [15:0] distance_mm,
    output reg busy,
    output reg done,
    output reg error
);

    localparam [6:0] VL53L0X_ADDR = 7'h29;

    localparam [7:0] REG_SYSRANGE_START         = 8'h00;
    localparam [7:0] REG_SYSTEM_INTERRUPT_CLEAR = 8'h0B;
    localparam [7:0] REG_RESULT_INTERRUPT_STATUS= 8'h13;
    localparam [7:0] REG_RESULT_RANGE_MM        = 8'h1E;

    localparam [7:0] CMD_START_SINGLE    = 8'h01;
    localparam [7:0] CMD_CLEAR_INTERRUPT = 8'h01;

    localparam I2C_WRITE = 1'b0;
    localparam I2C_READ  = 1'b1;

    localparam [3:0]
        IDLE                      = 4'd0,
        START_MEASUREMENT         = 4'd1,
        WAIT_START_DONE           = 4'd2,
        START_POLL_STATUS         = 4'd3,
        WAIT_POLL_STATUS_DONE     = 4'd4,
        POLL_DELAY                = 4'd5,
        START_READ_RANGE          = 4'd6,
        WAIT_READ_DONE            = 4'd7,
        START_CLEAR_INTERRUPT     = 4'd8,
        WAIT_CLEAR_INTERRUPT_DONE = 4'd9,
        DONE_ST                   = 4'd10,
        ERROR_ST                  = 4'd11;

    reg [3:0] state;

    reg start_d;
    wire start_pulse;

    reg        i2c_start;
    reg        i2c_rw;
    reg        i2c_reg_addr_valid;
    reg [7:0]  i2c_reg_addr;
    reg [7:0]  i2c_write_len;
    reg [7:0]  i2c_read_len;
    reg [MAX_BYTES * 8 - 1:0] i2c_write_data;

    wire [7:0] i2c_data_out;
    wire [MAX_BYTES * 8 - 1:0] i2c_read_data;
    wire i2c_busy;
    wire i2c_done;
    wire i2c_ack_error;

    reg [31:0] poll_delay_cnt;
    reg [31:0] timeout_cnt;
    reg [7:0]  status_reg;

    assign start_pulse = start & ~start_d;

    i2c_master #(
        .MAX_BYTES(MAX_BYTES)
    ) i2c (
        .clk(clk),
        .rst(rst),
        .start(i2c_start),
        .rw(i2c_rw),
        .addr(VL53L0X_ADDR),
        .reg_addr_valid(i2c_reg_addr_valid),
        .reg_addr(i2c_reg_addr),
        .write_len(i2c_write_len),
        .read_len(i2c_read_len),
        .write_data(i2c_write_data),
        .data_out(i2c_data_out),
        .read_data(i2c_read_data),
        .busy(i2c_busy),
        .done(i2c_done),
        .ack_error(i2c_ack_error),
        .sda(sda),
        .scl(scl)
    );

    always @(posedge clk or posedge rst) begin
        if (rst)
            start_d <= 1'b0;
        else
            start_d <= start;
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE;

            i2c_start <= 1'b0;
            i2c_rw <= I2C_WRITE;
            i2c_reg_addr_valid <= 1'b0;
            i2c_reg_addr <= 8'd0;
            i2c_write_len <= 8'd0;
            i2c_read_len <= 8'd0;
            i2c_write_data <= {MAX_BYTES * 8{1'b0}};

            poll_delay_cnt <= 32'd0;
            timeout_cnt <= 32'd0;
            status_reg <= 8'd0;

            distance_mm <= 16'd0;
            busy <= 1'b0;
            done <= 1'b0;
            error <= 1'b0;
        end else begin
            i2c_start <= 1'b0;
            done <= 1'b0;

            case (state)
                IDLE: begin
                    busy <= 1'b0;
                    error <= 1'b0;
                    poll_delay_cnt <= 32'd0;
                    timeout_cnt <= 32'd0;
                    status_reg <= 8'd0;

                    if (start_pulse) begin
                        busy <= 1'b1;
                        state <= START_MEASUREMENT;
                    end
                end

                START_MEASUREMENT: begin
                    if (!i2c_busy) begin
                        i2c_rw <= I2C_WRITE;
                        i2c_reg_addr_valid <= 1'b1;
                        i2c_reg_addr <= REG_SYSRANGE_START;
                        i2c_write_len <= 8'd1;
                        i2c_read_len <= 8'd0;
                        i2c_write_data <= {{(MAX_BYTES * 8 - 8){1'b0}}, CMD_START_SINGLE};
                        i2c_start <= 1'b1;
                        timeout_cnt <= 32'd0;
                        state <= WAIT_START_DONE;
                    end
                end

                WAIT_START_DONE: begin
                    if (i2c_done) begin
                        if (i2c_ack_error)
                            state <= ERROR_ST;
                        else
                            state <= START_POLL_STATUS;
                    end
                end

                START_POLL_STATUS: begin
                    if (!i2c_busy) begin
                        i2c_rw <= I2C_READ;
                        i2c_reg_addr_valid <= 1'b1;
                        i2c_reg_addr <= REG_RESULT_INTERRUPT_STATUS;
                        i2c_write_len <= 8'd0;
                        i2c_read_len <= 8'd1;
                        i2c_write_data <= {MAX_BYTES * 8{1'b0}};
                        i2c_start <= 1'b1;
                        state <= WAIT_POLL_STATUS_DONE;
                    end
                end

                WAIT_POLL_STATUS_DONE: begin
                    if (i2c_done) begin
                        if (i2c_ack_error) begin
                            state <= ERROR_ST;
                        end else begin
                            status_reg <= i2c_read_data[7:0];

                            // RESULT_INTERRUPT_STATUS[2:0] != 0 => measurement ready
                            if ((i2c_read_data[2:0]) != 3'b000) begin
                                state <= START_READ_RANGE;
                            end else if (timeout_cnt >= TIMEOUT_CYCLES) begin
                                state <= ERROR_ST;
                            end else begin
                                poll_delay_cnt <= 32'd0;
                                state <= POLL_DELAY;
                            end
                        end
                    end
                end

                POLL_DELAY: begin
                    if (poll_delay_cnt >= POLL_DELAY_CYCLES) begin
                        state <= START_POLL_STATUS;
                    end else begin
                        poll_delay_cnt <= poll_delay_cnt + 1'd1;
                        timeout_cnt <= timeout_cnt + 1'd1;
                    end
                end

                START_READ_RANGE: begin
                    if (!i2c_busy) begin
                        i2c_rw <= I2C_READ;
                        i2c_reg_addr_valid <= 1'b1;
                        i2c_reg_addr <= REG_RESULT_RANGE_MM;
                        i2c_write_len <= 8'd0;
                        i2c_read_len <= 8'd2;
                        i2c_write_data <= {MAX_BYTES * 8{1'b0}};
                        i2c_start <= 1'b1;
                        state <= WAIT_READ_DONE;
                    end
                end

                WAIT_READ_DONE: begin
                    if (i2c_done) begin
                        if (i2c_ack_error) begin
                            state <= ERROR_ST;
                        end else begin
                            // first byte read from 0x1E = MSB, second from 0x1F = LSB
                            distance_mm <= {i2c_read_data[7:0], i2c_read_data[15:8]};
                            state <= START_CLEAR_INTERRUPT;
                        end
                    end
                end

                START_CLEAR_INTERRUPT: begin
                    if (!i2c_busy) begin
                        i2c_rw <= I2C_WRITE;
                        i2c_reg_addr_valid <= 1'b1;
                        i2c_reg_addr <= REG_SYSTEM_INTERRUPT_CLEAR;
                        i2c_write_len <= 8'd1;
                        i2c_read_len <= 8'd0;
                        i2c_write_data <= {{(MAX_BYTES * 8 - 8){1'b0}}, CMD_CLEAR_INTERRUPT};
                        i2c_start <= 1'b1;
                        state <= WAIT_CLEAR_INTERRUPT_DONE;
                    end
                end

                WAIT_CLEAR_INTERRUPT_DONE: begin
                    if (i2c_done) begin
                        if (i2c_ack_error)
                            state <= ERROR_ST;
                        else
                            state <= DONE_ST;
                    end
                end

                DONE_ST: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    state <= IDLE;
                end

                ERROR_ST: begin
                    busy <= 1'b0;
                    error <= 1'b1;
                    done <= 1'b1;
                    state <= IDLE;
                end

                default: begin
                    state <= IDLE;
                    busy <= 1'b0;
                    done <= 1'b0;
                    error <= 1'b0;
                end
            endcase
        end
    end

endmodule