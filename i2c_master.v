module i2c_master (
    input wire clk,
    input wire rst,

    input wire start,
    input wire rw, // 0 = write, 1 = read
    input wire [6:0] addr,
    input wire [7:0] data_in,
    output reg [7:0] data_out,

    output reg busy,
    output reg done,
    output reg ack_error, // Если датчик не ответил, нет устройства, не верный адрес, ошибка тайминга

    inout wire sda,
    output reg scl
);
    // Clock divider (100 kHz)
    parameter CLK_DIV = 250; // для 50MHz → ~100kHz

    reg [15:0] clk_cnt;
    reg tick;

    always @(posedge clk) begin
        if (rst) begin
            clk_cnt <= 0;
            tick <= 0;
        end else if (clk_cnt == CLK_DIV) begin
            clk_cnt <= 0;
            tick <= 1;
        end else begin
            clk_cnt <= clk_cnt + 1;
            tick <= 0;
        end
    end

    // SDA (open-drain)
    reg sda_drive;
    reg sda_out;

    assign sda = (sda_drive) ? 1'b0 : 1'bz;
    wire sda_in = sda;

    localparam IDLE = 0;
    localparam START = 1;
    localparam ADDR = 2;
    localparam ACK_AFTER_ADDRES = 3;
    localparam DATA = 4;
    localparam ACK_AFTER_DATA = 5;
    localparam READ = 6;
    localparam NACK = 7;
    localparam STOP = 8;
    localparam DONE = 9;

    reg [3:0] state;
    reg [3:0] bit_cnt;
    reg [7:0] shift_reg;

    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            scl <= 1;
            sda_drive <= 0;
            busy <= 0;
            done <= 0;
            ack_error <= 0;
        end else if (tick) begin

            case (state)

            IDLE: begin
                done <= 0;
                if (start) begin
                    busy <= 1;
                    ack_error <= 0;
                    state <= START;
                end
            end

            // START: SDA ↓ при SCL=1
            START: begin
                sda_drive <= 1;
                scl <= 1;
                state <= ADDR;
                shift_reg <= {addr, rw};
                bit_cnt <= 7;
            end

            // Передача адреса + R/W
            ADDR: begin
                scl <= 0;
                sda_drive <= shift_reg[bit_cnt];

                scl <= 1;

                if (bit_cnt == 0)
                    state <= ACK_AFTER_ADDRES;
                else
                    bit_cnt <= bit_cnt - 1;
            end

            // ACK после адреса
            ACK_AFTER_ADDRES: begin
                scl <= 0;
                sda_drive <= 0; // отпустить SDA

                scl <= 1;
                if (sda_in != 0)
                    ack_error <= 1;

                if (rw)
                    state <= READ;
                else begin
                    state <= DATA;
                    shift_reg <= data_in;
                    bit_cnt <= 7;
                end
            end

            // Запись данных
            DATA: begin
                scl <= 0;
                sda_drive <= shift_reg[bit_cnt];

                scl <= 1;

                if (bit_cnt == 0)
                    state <= ACK_AFTER_DATA;
                else
                    bit_cnt <= bit_cnt - 1;
            end

            // ACK после данных
            ACK_AFTER_DATA: begin
                scl <= 0;
                sda_drive <= 0;

                scl <= 1;
                if (sda_in != 0)
                    ack_error <= 1;

                state <= STOP;
            end

            // Чтение
            READ: begin
                scl <= 0;
                sda_drive <= 0;

                scl <= 1;
                data_out[bit_cnt] <= sda_in;

                if (bit_cnt == 0)
                    state <= NACK;
                else
                    bit_cnt <= bit_cnt - 1;
            end

            // NACK после чтения
            NACK: begin
                scl <= 0;
                sda_drive <= 0; // NACK
                scl <= 1;
                state <= STOP;
            end

            // STOP: SDA ↑ при SCL=1
            STOP: begin
                scl <= 1;
                sda_drive <= 0;
                state <= DONE;
            end

            DONE: begin
                busy <= 0;
                done <= 1;
                state <= IDLE;
            end

            endcase
        end
    end

endmodule

