`timescale 1ns / 1ps
`default_nettype none

module top #(
    parameter int unsigned CLK_HZ          = 50_000_000,
    parameter int unsigned I2C_HZ          = 100_000,
    parameter int unsigned TOF_SENSOR_KIND = 0
) (
    input  wire       CLK,
    input  wire [3:0] KEY_SW,

    output logic [3:0] LED,
    output wire  [7:0] SEG,
    output wire  [3:0] DIG,

    inout  wire       i2c_scl,
    inout  wire       i2c_sda
);

    localparam int unsigned W_DIGIT = 4;

    wire clk   = CLK;
    wire rst_n = KEY_SW[3];
    wire rst   = ~rst_n;

    logic [3:0] key_sw_d;

    logic tof_init_start;
    logic tof_sample_start;

    logic [15:0] tof_distance_mm;
    logic        tof_distance_valid;
    logic        tof_busy;
    logic        tof_done;
    logic        tof_error;
    logic        tof_nack;

    logic        tof_txn_req_valid;
    logic        tof_txn_req_ready;
    logic        tof_txn_req_is_read;
    logic [6:0]  tof_txn_req_dev_addr;
    logic [15:0] tof_txn_req_reg_addr;
    logic        tof_txn_req_reg_addr_16b;
    logic [7:0]  tof_txn_req_wr_data;
    logic [1:0]  tof_txn_req_rd_len;

    logic        tof_txn_rsp_done;
    logic        tof_txn_rsp_error;
    logic        tof_txn_rsp_nack;
    logic [15:0] tof_txn_rsp_rd_data;

    logic        txn_core_start;
    logic        txn_core_is_read;
    logic [6:0]  txn_core_dev_addr;
    logic [15:0] txn_core_reg_addr;
    logic        txn_core_reg_addr_16b;
    logic [7:0]  txn_core_wr_data;
    logic [1:0]  txn_core_rd_len;
    logic [15:0] txn_core_rd_data;
    logic        txn_core_busy;
    logic        txn_core_done;
    logic        txn_core_error;
    logic        txn_core_nack;

    logic        scl_drive_low;
    logic        sda_drive_low;

    logic [15:0] disp_value;

    typedef enum logic [0:0] {
        BR_IDLE = 1'b0,
        BR_WAIT = 1'b1
    } bridge_state_t;

    bridge_state_t bridge_state;

    assign i2c_scl = scl_drive_low ? 1'b0 : 1'bz;
    assign i2c_sda = sda_drive_low ? 1'b0 : 1'bz;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_sw_d <= 4'b0000;
        end else begin
            key_sw_d <= KEY_SW;
        end
    end

    assign tof_init_start   = KEY_SW[0] & ~key_sw_d[0];
    assign tof_sample_start = KEY_SW[1] & ~key_sw_d[1];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bridge_state          <= BR_IDLE;

            txn_core_start        <= 1'b0;
            txn_core_is_read      <= 1'b0;
            txn_core_dev_addr     <= 7'h00;
            txn_core_reg_addr     <= 16'h0000;
            txn_core_reg_addr_16b <= 1'b0;
            txn_core_wr_data      <= 8'h00;
            txn_core_rd_len       <= 2'd0;

            tof_txn_req_ready     <= 1'b0;
            tof_txn_rsp_done      <= 1'b0;
            tof_txn_rsp_error     <= 1'b0;
            tof_txn_rsp_nack      <= 1'b0;
            tof_txn_rsp_rd_data   <= 16'h0000;
        end else begin
            txn_core_start      <= 1'b0;
            tof_txn_req_ready   <= 1'b0;
            tof_txn_rsp_done    <= 1'b0;

            case (bridge_state)
                BR_IDLE: begin
                    if (tof_txn_req_valid && !txn_core_busy) begin
                        txn_core_is_read      <= tof_txn_req_is_read;
                        txn_core_dev_addr     <= tof_txn_req_dev_addr;
                        txn_core_reg_addr     <= tof_txn_req_reg_addr;
                        txn_core_reg_addr_16b <= tof_txn_req_reg_addr_16b;
                        txn_core_wr_data      <= tof_txn_req_wr_data;
                        txn_core_rd_len       <= tof_txn_req_rd_len;

                        txn_core_start        <= 1'b1;
                        tof_txn_req_ready     <= 1'b1;

                        bridge_state          <= BR_WAIT;
                    end
                end

                BR_WAIT: begin
                    if (txn_core_done) begin
                        tof_txn_rsp_done    <= 1'b1;
                        tof_txn_rsp_error   <= txn_core_error;
                        tof_txn_rsp_nack    <= txn_core_nack;
                        tof_txn_rsp_rd_data <= txn_core_rd_data;

                        bridge_state        <= BR_IDLE;
                    end
                end

                default: begin
                    bridge_state <= BR_IDLE;
                end
            endcase
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            disp_value <= 16'h0000;
        end else if (tof_distance_valid) begin
            disp_value <= tof_distance_mm;
        end
    end

    always_comb begin
        LED[0] = tof_busy;
        LED[1] = tof_done;
        LED[2] = tof_error | tof_nack;
        LED[3] = tof_distance_valid;
    end

    i2c_master #(
        .CLK_HZ(CLK_HZ),
        .I2C_HZ(I2C_HZ)
    ) u_i2c_master (
        .clk           (clk),
        .rst_n         (rst_n),
        .start         (txn_core_start),
        .is_read       (txn_core_is_read),
        .dev_addr      (txn_core_dev_addr),
        .reg_addr      (txn_core_reg_addr),
        .reg_addr_16b  (txn_core_reg_addr_16b),
        .wr_data       (txn_core_wr_data),
        .rd_len        (txn_core_rd_len),
        .rd_data       (txn_core_rd_data),
        .busy          (txn_core_busy),
        .done          (txn_core_done),
        .error         (txn_core_error),
        .nack          (txn_core_nack),
        .scl_drive_low (scl_drive_low),
        .sda_drive_low (sda_drive_low),
        .scl_in        (i2c_scl),
        .sda_in        (i2c_sda)
    );

    tof_ctrl #(
        .SENSOR_KIND(TOF_SENSOR_KIND),
        .CLK_HZ(CLK_HZ)
    ) u_tof_ctrl (
        .clk                  (clk),
        .rst_n                (rst_n),
        .init_start           (tof_init_start),
        .sample_start         (tof_sample_start),
        .distance_mm          (tof_distance_mm),
        .distance_valid       (tof_distance_valid),
        .busy                 (tof_busy),
        .done                 (tof_done),
        .error                (tof_error),
        .nack                 (tof_nack),
        .txn_req_valid        (tof_txn_req_valid),
        .txn_req_ready        (tof_txn_req_ready),
        .txn_req_is_read      (tof_txn_req_is_read),
        .txn_req_dev_addr     (tof_txn_req_dev_addr),
        .txn_req_reg_addr     (tof_txn_req_reg_addr),
        .txn_req_reg_addr_16b (tof_txn_req_reg_addr_16b),
        .txn_req_wr_data      (tof_txn_req_wr_data),
        .txn_req_rd_len       (tof_txn_req_rd_len),
        .txn_rsp_done         (tof_txn_rsp_done),
        .txn_rsp_error        (tof_txn_rsp_error),
        .txn_rsp_nack         (tof_txn_rsp_nack),
        .txn_rsp_rd_data      (tof_txn_rsp_rd_data)
    );

    seven_segment_display #(
        .w_digit   (W_DIGIT),
        .clk_mhz   (CLK_HZ / 1_000_000),
        .update_hz (120)
    ) u_seven_segment_display (
        .clk      (clk),
        .rst      (rst),
        .number   (disp_value),
        .dots     ({W_DIGIT{1'b0}}),
        .abcdefgh (SEG),
        .digit    (DIG)
    );

endmodule

`default_nettype wire
