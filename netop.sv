`timescale 1ns / 1ps
`default_nettype none

module top (
    input  wire       CLK,
    input  wire [3:0] KEY_SW,

    output logic [3:0] LED,
    output logic [7:0] SEG,
    output logic [3:0] DIG,

    inout  wire       i2c_scl,
    inout  wire       i2c_sda
);

    // ---------------------------------------------------------------------
    // Reset
    // KEY_SW[3] = reset button
    // Assumption: released = 1, pressed = 0
    // So rst_n = KEY_SW[3]
    // ---------------------------------------------------------------------
    logic rst_n;
    assign rst_n = KEY_SW[3];

    // ---------------------------------------------------------------------
    // Edge detect for buttons
    // KEY_SW[0] -> init_start
    // KEY_SW[1] -> sample_start
    // ---------------------------------------------------------------------
    logic [3:0] key_d;
    logic init_start;
    logic sample_start;

    always_ff @(posedge CLK or negedge rst_n) begin
        if (!rst_n) begin
            key_d <= 4'b0000;
        end else begin
            key_d <= KEY_SW;
        end
    end

    assign init_start   =  KEY_SW[0] & ~key_d[0];
    assign sample_start =  KEY_SW[1] & ~key_d[1];

    // ---------------------------------------------------------------------
    // TOF controller interface
    // ---------------------------------------------------------------------
    logic [15:0] distance_mm;
    logic        distance_valid;
    logic        busy;
    logic        done;
    logic        error;
    logic        nack;

    logic        txn_req_valid;
    logic        txn_req_ready;
    logic        txn_req_is_read;
    logic [6:0]  txn_req_dev_addr;
    logic [15:0] txn_req_reg_addr;
    logic        txn_req_reg_addr_16b;
    logic [7:0]  txn_req_wr_data;
    logic [1:0]  txn_req_rd_len;

    logic        txn_rsp_done;
    logic        txn_rsp_error;
    logic        txn_rsp_nack;
    logic [15:0] txn_rsp_rd_data;

    // ---------------------------------------------------------------------
    // ToF wrapper
    // SENSOR_KIND:
    //   0 = AUTO
    //   1 = VL53L0X
    //   2 = VL53L1X
    // ---------------------------------------------------------------------
    tof_ctrl #(
        .SENSOR_KIND(0),
        .CLK_HZ(50_000_000)
    ) u_tof_ctrl (
        .clk                  (CLK),
        .rst_n                (rst_n),

        .init_start           (init_start),
        .sample_start         (sample_start),

        .distance_mm          (distance_mm),
        .distance_valid       (distance_valid),
        .busy                 (busy),
        .done                 (done),
        .error                (error),
        .nack                 (nack),

        .txn_req_valid        (txn_req_valid),
        .txn_req_ready        (txn_req_ready),
        .txn_req_is_read      (txn_req_is_read),
        .txn_req_dev_addr     (txn_req_dev_addr),
        .txn_req_reg_addr     (txn_req_reg_addr),
        .txn_req_reg_addr_16b (txn_req_reg_addr_16b),
        .txn_req_wr_data      (txn_req_wr_data),
        .txn_req_rd_len       (txn_req_rd_len),

        .txn_rsp_done         (txn_rsp_done),
        .txn_rsp_error        (txn_rsp_error),
        .txn_rsp_nack         (txn_rsp_nack),
        .txn_rsp_rd_data      (txn_rsp_rd_data)
    );

    // ---------------------------------------------------------------------
    // I2C transaction master
    // Replace this module name/ports if your real I2C master is different.
    // ---------------------------------------------------------------------
    i2c_reg_master #(
        .CLK_HZ(50_000_000),
        .I2C_HZ(100_000)
    ) u_i2c_reg_master (
        .clk              (CLK),
        .rst_n            (rst_n),

        .req_valid        (txn_req_valid),
        .req_ready        (txn_req_ready),
        .req_is_read      (txn_req_is_read),
        .req_dev_addr     (txn_req_dev_addr),
        .req_reg_addr     (txn_req_reg_addr),
        .req_reg_addr_16b (txn_req_reg_addr_16b),
        .req_wr_data      (txn_req_wr_data),
        .req_rd_len       (txn_req_rd_len),

        .rsp_done         (txn_rsp_done),
        .rsp_error        (txn_rsp_error),
        .rsp_nack         (txn_rsp_nack),
        .rsp_rd_data      (txn_rsp_rd_data),

        .i2c_scl          (i2c_scl),
        .i2c_sda          (i2c_sda)
    );

    // ---------------------------------------------------------------------
    // LEDs
    // ---------------------------------------------------------------------
    always_comb begin
        LED[0] = busy;
        LED[1] = done;
        LED[2] = error | nack;
        LED[3] = distance_valid;
    end

    // ---------------------------------------------------------------------
    // 7-segment display
    // Shows distance_mm as HEX
    //
    // Assumption:
    //   SEG active low
    //   DIG active low
    // ---------------------------------------------------------------------
    logic [15:0] disp_value;
    logic [15:0] scan_div;
    logic [1:0]  scan_sel;
    logic [3:0]  digit_nibble;
    logic [7:0]  seg_raw;
    logic [3:0]  dig_raw;

    always_ff @(posedge CLK or negedge rst_n) begin
        if (!rst_n) begin
            disp_value <= 16'h0000;
        end else if (distance_valid) begin
            disp_value <= distance_mm;
        end
    end

    always_ff @(posedge CLK or negedge rst_n) begin
        if (!rst_n) begin
            scan_div <= 16'd0;
            scan_sel <= 2'd0;
        end else begin
            scan_div <= scan_div + 16'd1;
            scan_sel <= scan_div[15:14];
        end
    end

    always_comb begin
        case (scan_sel)
            2'd0: begin
                digit_nibble = disp_value[3:0];
                dig_raw      = 4'b1110;
            end
            2'd1: begin
                digit_nibble = disp_value[7:4];
                dig_raw      = 4'b1101;
            end
            2'd2: begin
                digit_nibble = disp_value[11:8];
                dig_raw      = 4'b1011;
            end
            default: begin
                digit_nibble = disp_value[15:12];
                dig_raw      = 4'b0111;
            end
        endcase
    end

    always_comb begin
        case (digit_nibble)
            4'h0: seg_raw = 8'b1100_0000;
            4'h1: seg_raw = 8'b1111_1001;
            4'h2: seg_raw = 8'b1010_0100;
            4'h3: seg_raw = 8'b1011_0000;
            4'h4: seg_raw = 8'b1001_1001;
            4'h5: seg_raw = 8'b1001_0010;
            4'h6: seg_raw = 8'b1000_0010;
            4'h7: seg_raw = 8'b1111_1000;
            4'h8: seg_raw = 8'b1000_0000;
            4'h9: seg_raw = 8'b1001_0000;
            4'hA: seg_raw = 8'b1000_1000;
            4'hB: seg_raw = 8'b1000_0011;
            4'hC: seg_raw = 8'b1100_0110;
            4'hD: seg_raw = 8'b1010_0001;
            4'hE: seg_raw = 8'b1000_0110;
            4'hF: seg_raw = 8'b1000_1110;
            default: seg_raw = 8'b1111_1111;
        endcase
    end

    assign SEG = seg_raw;
    assign DIG = dig_raw;

endmodule

`default_nettype wire
