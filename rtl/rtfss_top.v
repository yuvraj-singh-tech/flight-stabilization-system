`timescale 1ns / 1ps

// -----------------------------------------------------------------------------
// Module: rtfss_top
// Project: Real-Time Flight Stabilization System (RT-FSS)
// Author: Yuvraj Singh
//
// Target Board : Xilinx SP701 (Spartan-7, ~33.33 MHz system clock)
// Sensor       : MPU6050 (I2C interface)
// Actuator     : Hobby Servo (SG90 / MG90S)
//
// Data Processing Pipeline:
//   reset_sync      -> reset conditioning
//   tick_gen        -> generates 1 kHz control tick
//   mpu6050_reader  -> reads IMU frame via I2C
//   angle_filter    -> complementary filter for pitch estimation
//   pid_controller  -> pitch stabilization control
//   pwm_servo       -> servo PWM generation (1–2 ms pulses)
// -----------------------------------------------------------------------------

module rtfss_top (
    input  wire clk_33m,      // 33.33 MHz system clock
    input  wire rst_btn,      // push-button reset (active-high)

    inout  wire i2c_sda,      // I2C data line to MPU6050
    output wire i2c_scl,      // I2C clock line to MPU6050

    output wire servo_pwm,    // PWM output to servo

    output wire init_done,    // IMU reader initialized
    output wire error_flag    // latched I2C error indicator
);

    // -------------------------------------------------------------------------
    // Reset synchronizer
    // -------------------------------------------------------------------------
    wire rst_sync;

    reset_sync u_reset_sync (
        .clk      (clk_33m),
        .rst_raw  (rst_btn),
        .rst_sync (rst_sync)
    );

    // -------------------------------------------------------------------------
    // Tick generator (1 kHz control tick)
    // -------------------------------------------------------------------------
    wire tick_1khz;
    wire tick_50hz;

    tick_gen u_tick_gen (
        .clk       (clk_33m),
        .rst       (rst_sync),
        .tick_1khz (tick_1khz),
        .tick_50hz (tick_50hz)
    );

    // -------------------------------------------------------------------------
    // MPU6050 sensor reader
    // -------------------------------------------------------------------------
    wire              imu_data_valid;

    wire signed [15:0] accel_x;
    wire signed [15:0] accel_y;
    wire signed [15:0] accel_z;

    wire signed [15:0] gyro_x;
    wire signed [15:0] gyro_y;
    wire signed [15:0] gyro_z;

    mpu6050_reader #(
        .POWERUP_DELAY_TICKS(100)
    ) u_mpu6050_reader (

        .clk         (clk_33m),
        .rst         (rst_sync),

        .sample_tick (tick_1khz),

        .scl         (i2c_scl),
        .sda         (i2c_sda),

        .init_done   (init_done),
        .data_valid  (imu_data_valid),
        .error_flag  (error_flag),

        .accel_x     (accel_x),
        .accel_y     (accel_y),
        .accel_z     (accel_z),

        .gyro_x      (gyro_x),
        .gyro_y      (gyro_y),
        .gyro_z      (gyro_z)
    );

    // -------------------------------------------------------------------------
    // Complementary filter for pitch estimation
    // -------------------------------------------------------------------------
    wire signed [15:0] pitch_angle;
    wire               angle_valid;

    angle_filter u_angle_filter (

        .clk         (clk_33m),
        .rst         (rst_sync),

        .data_valid  (imu_data_valid),

        .accel_y     (accel_y),
        .accel_z     (accel_z),
        .gyro_x      (gyro_x),

        .pitch_angle (pitch_angle),
        .angle_valid (angle_valid)
    );

    // -------------------------------------------------------------------------
    // PID controller
    // -------------------------------------------------------------------------
    wire [7:0] servo_position;

    pid_controller u_pid_controller (

        .clk          (clk_33m),
        .rst          (rst_sync),

        .angle_valid  (angle_valid),
        .pitch_angle  (pitch_angle),

        .target_angle (16'sd0),   // maintain level (0 degrees)

        .position_out (servo_position),

        .error_out    (),
        .p_term_dbg   (),
        .i_term_dbg   (),
        .d_term_dbg   (),
        .control_dbg  ()
    );

    // -------------------------------------------------------------------------
    // Servo PWM generator
    // -------------------------------------------------------------------------
    pwm_servo u_pwm_servo (

        .clk      (clk_33m),
        .rst      (rst_sync),

        .position (servo_position),
        .dout     (servo_pwm)
    );

endmodule
