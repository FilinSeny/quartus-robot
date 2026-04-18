module vl53l0x_controller (
    input wire clk,
    input wire rst,
    input wire start,

    output wire scl,
    inout wire sda,

    output reg [7:0] distance,
    output reg done
);

    localparam ADDR = 7'h29; // В теории для VL53L0X всегда такой
    localparam SENSOR_REG = 8'h1E; // пример регистра
    
    localparam IDLE = 0;
    localparam WRITE_REG_ADDR = 1;
    localparam READ_REG_DATA = 2;

    reg i2c_start;
    reg i2c_rw;
    reg [7:0] i2c_data_in;
    wire [7:0] i2c_data_out;
    wire i2c_done;

    i2c_master i2c (
        .clk(clk),
        .rst(rst),
        .start(i2c_start),
        .rw(i2c_rw),
        .addr(ADDR),
        .data_in(i2c_data_in),
        .data_out(i2c_data_out),
        .done(i2c_done),
        .scl(scl),
        .sda(sda)
    );

    reg [2:0] state;

    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            done <= 0;
        end else begin
            case (state)

            // Запись адреса регистра
            IDLE: if (start) begin
                i2c_start <= 1;
                i2c_rw <= 0;
                i2c_data_in <= SENSOR_REG;
                state <= WRITE_REG_ADDR;
            end

            WRITE_REG_ADDR: if (i2c_done) begin
                i2c_start <= 1;
                i2c_rw <= 1;
                state <= READ_REG_DATA;
            end

            READ_REG_DATA: if (i2c_done) begin
                distance <= i2c_data_out;
                done <= 1;
                state <= IDLE;
            end

            endcase
        end
    end

endmodule

