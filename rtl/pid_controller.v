// -----------------------------------------------------------------------------
// Module: pid_controller
// Project: Real-Time Flight Stabilization System (RT-FSS)
// Author: Yuvraj Singh
//
// Description:
// Fixed-point PID controller for pitch stabilization.
//
// The controller updates on each valid angle sample and generates an 8-bit
// servo position command. Proportional, integral, and derivative terms are
// calculated using Q8.8 fixed-point gains. Integral clamping is included to
// limit wind-up during sustained error conditions.
// -----------------------------------------------------------------------------

module pid_controller (
    input  wire               clk,
    input  wire               rst,

    input  wire               angle_valid,
    input  wire signed [15:0] pitch_angle,
    input  wire signed [15:0] target_angle,

    output reg  [7:0]         position_out,

    output reg  signed [15:0] error_out,
    output reg  signed [15:0] p_term_dbg,
    output reg  signed [15:0] i_term_dbg,
    output reg  signed [15:0] d_term_dbg,
    output reg  signed [15:0] control_dbg
);

    // PID gains in Q8.8 format
    parameter signed [15:0] KP_NUM = 16'sd64;
    parameter signed [15:0] KI_NUM = 16'sd8;
    parameter signed [15:0] KD_NUM = 16'sd32;

    // Integral clamp limits
    parameter signed [31:0] I_MAX = 32'sd500000;
    parameter signed [31:0] I_MIN = -32'sd500000;

    // Neutral servo position
    parameter [7:0] SERVO_CENTER = 8'd128;

    reg signed [15:0] error_q;
    reg signed [15:0] prev_error_q;

    reg signed [31:0] i_accum_q;

    reg signed [31:0] p_term_full;
    reg signed [31:0] i_term_full;
    reg signed [31:0] d_term_full;
    reg signed [31:0] control_full;

    always @(posedge clk) begin
        if (rst) begin
            error_q      <= 16'sd0;
            prev_error_q <= 16'sd0;
            i_accum_q    <= 32'sd0;

            error_out    <= 16'sd0;
            p_term_dbg   <= 16'sd0;
            i_term_dbg   <= 16'sd0;
            d_term_dbg   <= 16'sd0;
            control_dbg  <= 16'sd0;

            position_out <= SERVO_CENTER;
        end else if (angle_valid) begin
            // Current control error
            error_q   <= target_angle - pitch_angle;
            error_out <= target_angle - pitch_angle;

            // Proportional term
            p_term_full <= (target_angle - pitch_angle) * KP_NUM;

            // Integral term with clamping
            begin : integral_block
                reg signed [31:0] i_accum_next;

                i_accum_next = i_accum_q + ((target_angle - pitch_angle) * KI_NUM);

                if (i_accum_next > I_MAX)
                    i_accum_q <= I_MAX;
                else if (i_accum_next < I_MIN)
                    i_accum_q <= I_MIN;
                else
                    i_accum_q <= i_accum_next;
            end

            // Derivative term
            d_term_full  <= ((target_angle - pitch_angle) - prev_error_q) * KD_NUM;
            prev_error_q <= (target_angle - pitch_angle);

            // Combine PID terms and generate servo command
            begin : compose_block
                reg signed [15:0] p_term_s;
                reg signed [15:0] i_term_s;
                reg signed [15:0] d_term_s;
                reg signed [31:0] ctrl_temp;
                reg [7:0] pos_temp;

                p_term_s = p_term_full[31:8];
                i_term_s = i_accum_q[31:8];
                d_term_s = d_term_full[31:8];

                ctrl_temp = p_term_s + i_term_s + d_term_s;

                p_term_dbg  <= p_term_s;
                i_term_dbg  <= i_term_s;
                d_term_dbg  <= d_term_s;
                control_dbg <= ctrl_temp[15:0];

                ctrl_temp = ctrl_temp + SERVO_CENTER;

                if (ctrl_temp < 0)
                    pos_temp = 8'd0;
                else if (ctrl_temp > 255)
                    pos_temp = 8'd255;
                else
                    pos_temp = ctrl_temp[7:0];

                position_out <= pos_temp;
            end
        end
    end

endmodule
