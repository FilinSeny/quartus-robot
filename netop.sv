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
    output logic [7:0] SEG,
    output logic [3:0] DIG,

    inout  wire       i2c_scl,
    inout  wire       i2c_sda
);

    // ---------------------------------------------------------------------
    // Clock / Reset
    // KEY_SW[3] = reset, active low on button press
    // ---------------------------------------------------------------------
    wire clk   = CLK;
    wire rst_n = KEY_SW[3];

    // ---------------------------------------------------------------------
    // Button edge detect
    // KEY_SW[0] -> init
    // KEY_SW[1] -> sample
    // ---------------------------------------------------------------------
    logic [3:0] key_sw_d;

    logic tof_init_start;
    logic tof_sample_start;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_sw_d <= 4'b0000;
        end else begin
            key_sw_d <= KEY_SW;
        end
    end

    assign tof_init_start   =  KEY_SW[0] & ~key_sw_d[0];
    assign tof_sample_start =  KEY_SW[1] & ~key_sw_d[1];

    // ---------------------------------------------------------------------
    // TOF controller signals
    // ---------------------------------------------------------------------
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

    // ---------------------------------------------------------------------
    // I2C master core signals
    // ---------------------------------------------------------------------
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

    // ---------------------------------------------------------------------
    // Open-drain I2C pins
    // ---------------------------------------------------------------------
    assign i2c_scl = scl_drive_low ? 1'b0 : 1'bz;
    assign i2c_sda = sda_drive_low ? 1'b0 : 1'bz;

    // ---------------------------------------------------------------------
    // Bridge: tof_ctrl handshake -> pulse start for i2c_master
    //
    // tof_ctrl gives:
    //   txn_req_valid / txn_req_ready
    //
    // i2c_master expects:
    //   start pulse + busy/done/error/nack
    //
    // So we:
    //   1) wait for request from tof_ctrl
    //   2) if i2c_master is idle, latch request and pulse txn_core_start
    //   3) return ready to tof_ctrl for one cycle on accept
    //   4) pass done/error/nack/rd_data back when transaction finishes
    // ---------------------------------------------------------------------
    typedef enum logic [0:0] {
        BR_IDLE = 1'b0,
        BR_WAIT = 1'b1
    } bridge_state_t;

    bridge_state_t bridge_state;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bridge_state           <= BR_IDLE;

            txn_core_start         <= 1'b0;
            txn_core_is_read       <= 1'b0;
            txn_core_dev_addr      <= 7'h00;
            txn_core_reg_addr      <= 16'h0000;
            txn_core_reg_addr_16b  <= 1'b0;
            txn_core_wr_data       <= 8'h00;
            txn_core_rd_len        <= 2'd0;

            tof_txn_req_ready      <= 1'b0;
            tof_txn_rsp_done       <= 1'b0;
            tof_txn_rsp_error      <= 1'b0;
            tof_txn_rsp_nack       <= 1'b0;
            tof_txn_rsp_rd_data    <= 16'h0000;
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

    // ---------------------------------------------------------------------
    // I2C master
    // ---------------------------------------------------------------------
    i2c_master #(
        .CLK_HZ(CLK_HZ),
        .I2C_HZ(I2C_HZ)
    ) u_i2c_master (
        .clk             (clk),
        .rst_n           (rst_n),
        .start           (txn_core_start),
        .is_read         (txn_core_is_read),
        .dev_addr        (txn_core_dev_addr),
        .reg_addr        (txn_core_reg_addr),
        .reg_addr_16b    (txn_core_reg_addr_16b),
        .wr_data         (txn_core_wr_data),
        .rd_len          (txn_core_rd_len),
        .rd_data         (txn_core_rd_data),
        .busy            (txn_core_busy),
        .done            (txn_core_done),
        .error           (txn_core_error),
        .nack            (txn_core_nack),
        .scl_drive_low   (scl_drive_low),
        .sda_drive_low   (sda_drive_low),
        .scl_in          (i2c_scl),
        .sda_in          (i2c_sda)
    );

    // ---------------------------------------------------------------------
    // TOF controller
    // ---------------------------------------------------------------------
    tof_ctrl #(
        .SENSOR_KIND(TOF_SENSOR_KIND),
        .CLK_HZ(CLK_HZ)
    ) u_tof_ctrl (
        .clk                 (clk),
        .rst_n               (rst_n),
        .init_start          (tof_init_start),
        .sample_start        (tof_sample_start),
        .distance_mm         (tof_distance_mm),
        .distance_valid      (tof_distance_valid),
        .busy                (tof_busy),
        .done                (tof_done),
        .error               (tof_error),
        .nack                (tof_nack),
        .txn_req_valid       (tof_txn_req_valid),
        .txn_req_ready       (tof_txn_req_ready),
        .txn_req_is_read     (tof_txn_req_is_read),
        .txn_req_dev_addr    (tof_txn_req_dev_addr),
        .txn_req_reg_addr    (tof_txn_req_reg_addr),
        .txn_req_reg_addr_16b(tof_txn_req_reg_addr_16b),
        .txn_req_wr_data     (tof_txn_req_wr_data),
        .txn_req_rd_len      (tof_txn_req_rd_len),
        .txn_rsp_done        (tof_txn_rsp_done),
        .txn_rsp_error       (tof_txn_rsp_error),
        .txn_rsp_nack        (tof_txn_rsp_nack),
        .txn_rsp_rd_data     (tof_txn_rsp_rd_data)
    );

    // ---------------------------------------------------------------------
    // LEDs
    // ---------------------------------------------------------------------
    always_comb begin
        LED[0] = tof_busy;
        LED[1] = tof_done;
        LED[2] = tof_error | tof_nack;
        LED[3] = tof_distance_valid;
    end

    // ---------------------------------------------------------------------
    // 7-segment display: show distance in HEX
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

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            disp_value <= 16'h0000;
        end else if (tof_distance_valid) begin
            disp_value <= tof_distance_mm;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
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
