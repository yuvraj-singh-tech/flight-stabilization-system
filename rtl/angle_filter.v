`timescale 1ns / 1ps
// ============================================================================
// Module : angle_filter
// Target : SP701 (XC7S100, SYSCLK ~33.33 MHz)
// Role   : Fuse MPU6050 accel + gyro into a filtered pitch angle
//
// Inputs from mpu6050_reader:
//   - data_valid : 1-clk pulse when a new 6-word IMU frame is ready
//   - accel_y/z  : 16-bit signed raw accel (AFS_SEL = ±2g → 16384 LSB/g)
//   - gyro_x     : 16-bit signed raw gyro  (FS_SEL = ±250 dps → 131 LSB/(°/s))
//
// Output to PID:
//   - pitch_angle : signed 16-bit, degrees * 100 (e.g. 2500 = 25.00°)
//   - angle_valid : 1-clk pulse aligned with new pitch_angle
//
// Notes:
//   - Assumes board is roughly level (Z ≈ +1g) and small pitch angles.
//   - Uses a simple complementary filter (no heavy atan2/CORDIC).
//   - All scaling is parameterized so you can tune later if needed.
// ============================================================================

module angle_filter #(
    // ------------------------------------------------------------------------
    // Complementary filter coefficient:
    //   alpha ≈ 0.98 (gyro dominance)
    //   (1 - alpha) ≈ 0.02 (accel correction)
    // ------------------------------------------------------------------------
    parameter integer ALPHA_NUM = 98,     // numerator (0..ALPHA_DEN)
    parameter integer ALPHA_DEN = 100,

    // ------------------------------------------------------------------------
    // Gyro integration gain:
    // We want: Δangle_deg ≈ gyro_x(LSB) * (°/s per LSB) * Δt(s)
    // Default FS_SEL=0 => 131 LSB/(°/s)
    // If we approximate Δt ≈ 1 ms = 0.001 s:
    //   Δangle_deg ≈ gyro_x / 131000
    // For angle in deg*100: multiply by 100 => gyro_x / 1310
    //
    // Implement as: gyro_delta = gyro_x * GYRO_GAIN_NUM / GYRO_GAIN_DEN
    // You can tweak this later during tuning.
    // ------------------------------------------------------------------------
    parameter integer GYRO_GAIN_NUM = 100,    // numerator for gyro gain
    parameter integer GYRO_GAIN_DEN = 1310,   // denominator for gyro gain

    // ------------------------------------------------------------------------
    // Accel angle gain (small-angle approximation):
    // For ±2g: 16384 LSB/g. For small pitch, θ ≈ accel_y / 1g (in rad),
    // convert to deg*100 ⇒ approx factor ~0.35.
    // Implement as: angle_acc = accel_y * ACCEL_GAIN_NUM / ACCEL_GAIN_DEN
    // ------------------------------------------------------------------------
    parameter integer ACCEL_GAIN_NUM = 5730,   // ≈ 57.3 * 100
    parameter integer ACCEL_GAIN_DEN = 16384   // LSB per g at ±2g
)(
    input  wire                  clk,
    input  wire                  rst,

    // From mpu6050_reader
    input  wire                  data_valid,   // 1-clk pulse per IMU frame
    input  wire signed [15:0]    accel_y,
    input  wire signed [15:0]    accel_z,      // currently not used, but kept for future refinement
    input  wire signed [15:0]    gyro_x,       // pitch rate axis

    // Filtered output
    output reg  signed [15:0]    pitch_angle,  // deg * 100
    output reg                   angle_valid   // 1-clk pulse
);

    // ------------------------------------------------------------------------
    // Internal state: angle estimate in deg*100 (wider to keep precision)
    // ------------------------------------------------------------------------
    reg  signed [31:0] angle_est;       // main internal angle state

    // ------------------------------------------------------------------------
    // Combinational helpers
    // ------------------------------------------------------------------------

    // 1) Gyro integration term: Δangle from gyro for this sample
    wire signed [31:0] gyro_delta =
        ( (gyro_x * GYRO_GAIN_NUM) / GYRO_GAIN_DEN );

    // 2) Integrated gyro angle (before accel correction)
    wire signed [31:0] angle_gyro_next = angle_est + gyro_delta;

    // 3) Accel-based angle estimate (small-angle approx)
    //    Here we essentially scale accel_y to an "angle-like" quantity.
    wire signed [31:0] angle_acc_next =
        ( (accel_y * ACCEL_GAIN_NUM) / ACCEL_GAIN_DEN );

    // 4) Complementary filter mix:
    //    angle_fused = alpha * angle_gyro_next + (1-alpha) * angle_acc_next
    wire signed [47:0] mix_num =
          ( (angle_gyro_next * ALPHA_NUM) )
        + ( (angle_acc_next * (ALPHA_DEN - ALPHA_NUM)) );

    wire signed [31:0] angle_fused_next = mix_num / ALPHA_DEN;

    // ------------------------------------------------------------------------
    // Sequential logic: update on each data_valid frame
    // ------------------------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            angle_est   <= 32'sd0;
            pitch_angle <= 16'sd0;
            angle_valid <= 1'b0;
        end else begin
            angle_valid <= 1'b0;   // default

            if (data_valid) begin
                // Update internal angle estimate
                angle_est <= angle_fused_next;

                // Export lower 16 bits as external pitch angle
                // (for typical ranges ±90° this is safe in deg*100 units)
                pitch_angle <= angle_fused_next[15:0];

                // Strobe valid for one cycle
                angle_valid <= 1'b1;
            end
        end
    end

endmodule
