// -----------------------------------------------------------------------------
// Module: mpu6050_reader
// Project: Real-Time Flight Stabilization System (RT-FSS)
// Author: Yuvraj Singh
//
// Description:
// Reads a complete 6-axis frame from the MPU6050 using the I2C interface.
// The module is triggered by a 1 kHz sample tick and sequentially reads:
//
//   AX, AY, AZ, GX, GY, GZ
//
// After all six values are captured, data_valid pulses for one clock cycle.
// A sticky error flag is provided for transaction failures. Debug counters are
// included to help track missed sample opportunities and overruns.
// -----------------------------------------------------------------------------

module mpu6050_reader #(
    parameter integer POWERUP_DELAY_TICKS = 100
)(
    input  wire               clk,
    input  wire               rst,
    input  wire               sample_tick,

    output wire               scl,
    inout  wire               sda,

    output reg                init_done,
    output reg                data_valid,
    output reg                error_flag,

    output reg [15:0]         missed_ticks,
    output reg [15:0]         overrun_count,

    output reg signed [15:0]  accel_x,
    output reg signed [15:0]  accel_y,
    output reg signed [15:0]  accel_z,
    output reg signed [15:0]  gyro_x,
    output reg signed [15:0]  gyro_y,
    output reg signed [15:0]  gyro_z
);

    // Internal debug signals for Vivado ILA
    (* mark_debug = "true" *) reg  [2:0] state;
    (* mark_debug = "true" *) reg  [2:0] read_index;
    (* mark_debug = "true" *) reg        sample_request_pending;
    (* mark_debug = "true" *) reg [15:0] powerup_cnt;
    (* mark_debug = "true" *) wire       i2c_init_done;
    (* mark_debug = "true" *) wire       i2c_busy_dbg  = i2c_busy;
    (* mark_debug = "true" *) wire       i2c_done_dbg  = i2c_done;
    (* mark_debug = "true" *) wire       i2c_error_dbg = i2c_error;

    // MPU6050 register addresses
    localparam [7:0] REG_ACCEL_XOUT_H = 8'h3B;
    localparam [7:0] REG_ACCEL_YOUT_H = 8'h3D;
    localparam [7:0] REG_ACCEL_ZOUT_H = 8'h3F;
    localparam [7:0] REG_GYRO_XOUT_H  = 8'h43;
    localparam [7:0] REG_GYRO_YOUT_H  = 8'h45;
    localparam [7:0] REG_GYRO_ZOUT_H  = 8'h47;

    localparam integer NUM_READS = 6;

    // I2C interface
    reg         i2c_start;
    reg  [7:0]  i2c_reg_addr;
    wire        i2c_busy;
    wire        i2c_done;
    wire        i2c_error;
    wire [15:0] i2c_data_out;

    i2c_mpu6050 u_i2c (
        .clk       (clk),
        .rst       (rst),
        .start     (i2c_start),
        .reg_addr  (i2c_reg_addr),
        .busy      (i2c_busy),
        .done      (i2c_done),
        .error     (i2c_error),
        .scl       (scl),
        .sda       (sda),
        .data_out  (i2c_data_out),
        .init_done (i2c_init_done)
    );

    // Reader states
    localparam [2:0]
        ST_RESET      = 3'd0,
        ST_POWERUP    = 3'd1,
        ST_IDLE       = 3'd2,
        ST_START_READ = 3'd3,
        ST_WAIT_READ  = 3'd4,
        ST_STORE      = 3'd5,
        ST_ERROR      = 3'd6;

    reg [2:0] next_state;

    // Register address selection for each axis read
    function [7:0] reg_addr_for_index(input [2:0] idx);
        begin
            case (idx)
                3'd0: reg_addr_for_index = REG_ACCEL_XOUT_H;
                3'd1: reg_addr_for_index = REG_ACCEL_YOUT_H;
                3'd2: reg_addr_for_index = REG_ACCEL_ZOUT_H;
                3'd3: reg_addr_for_index = REG_GYRO_XOUT_H;
                3'd4: reg_addr_for_index = REG_GYRO_YOUT_H;
                3'd5: reg_addr_for_index = REG_GYRO_ZOUT_H;
                default: reg_addr_for_index = REG_ACCEL_XOUT_H;
            endcase
        end
    endfunction

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state                  <= ST_RESET;
            init_done              <= 1'b0;
            powerup_cnt            <= 16'd0;
            sample_request_pending <= 1'b0;

            i2c_start              <= 1'b0;
            i2c_reg_addr           <= 8'd0;

            read_index             <= 3'd0;
            data_valid             <= 1'b0;
            error_flag             <= 1'b0;

            missed_ticks           <= 16'd0;
            overrun_count          <= 16'd0;

            accel_x                <= 16'sd0;
            accel_y                <= 16'sd0;
            accel_z                <= 16'sd0;
            gyro_x                 <= 16'sd0;
            gyro_y                 <= 16'sd0;
            gyro_z                 <= 16'sd0;
        end else begin
            state      <= next_state;
            i2c_start  <= 1'b0;
            data_valid <= 1'b0;

            // Count sample ticks that arrive while I2C is still busy
            if (sample_tick && init_done && i2c_busy) begin
                if (missed_ticks != 16'hFFFF)
                    missed_ticks <= missed_ticks + 16'd1;
            end

            // Queue a sample request at every valid sample tick
            if (init_done && sample_tick) begin
                if (sample_request_pending && !i2c_busy) begin
                    if (overrun_count != 16'hFFFF)
                        overrun_count <= overrun_count + 16'd1;
                end

                sample_request_pending <= 1'b1;
            end

            // Initial power-up wait before starting reads
            if (state == ST_POWERUP && sample_tick && !init_done) begin
                if (powerup_cnt < POWERUP_DELAY_TICKS - 1)
                    powerup_cnt <= powerup_cnt + 16'd1;
                else
                    init_done <= 1'b1;
            end

            // Sticky transaction error flag
            if (i2c_error)
                error_flag <= 1'b1;

            // Store received sensor data
            if (state == ST_STORE) begin
                case (read_index)
                    3'd0: accel_x <= i2c_data_out;
                    3'd1: accel_y <= i2c_data_out;
                    3'd2: accel_z <= i2c_data_out;
                    3'd3: gyro_x  <= i2c_data_out;
                    3'd4: gyro_y  <= i2c_data_out;
                    3'd5: gyro_z  <= i2c_data_out;
                    default: begin end
                endcase
            end

            // Pulse when a full 6-axis frame has been captured
            if (state == ST_STORE && read_index == NUM_READS - 1)
                data_valid <= 1'b1;

            // I2C transaction control
            case (state)
                ST_IDLE: begin
                    if (init_done && sample_request_pending && !i2c_busy) begin
                        read_index             <= 3'd0;
                        i2c_reg_addr           <= reg_addr_for_index(3'd0);
                        sample_request_pending <= 1'b0;
                    end
                end

                ST_START_READ: begin
                    if (!i2c_busy) begin
                        i2c_reg_addr <= reg_addr_for_index(read_index);
                        i2c_start    <= 1'b1;
                    end
                end

                ST_STORE: begin
                    if (read_index < NUM_READS - 1)
                        read_index <= read_index + 3'd1;
                end

                default: begin end
            endcase
        end
    end

    always @* begin
        next_state = state;

        case (state)
            ST_RESET:
                next_state = ST_POWERUP;

            ST_POWERUP:
                if (init_done)
                    next_state = ST_IDLE;

            ST_IDLE:
                if (init_done && sample_request_pending && !i2c_busy)
                    next_state = ST_START_READ;

            ST_START_READ:
                next_state = ST_WAIT_READ;

            ST_WAIT_READ:
                if (i2c_done)
                    next_state = i2c_error ? ST_ERROR : ST_STORE;

            ST_STORE:
                if (read_index == NUM_READS - 1)
                    next_state = ST_IDLE;
                else
                    next_state = ST_START_READ;

            ST_ERROR:
                next_state = ST_ERROR;

            default:
                next_state = ST_RESET;
        endcase
    end

endmodule
