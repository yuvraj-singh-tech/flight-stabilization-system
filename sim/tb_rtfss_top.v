`timescale 1ns / 1ps

// -----------------------------------------------------------------------------
// Testbench: tb_rtfss_top
// Project  : Real-Time Flight Stabilization System (RT-FSS)
// Author   : Yuvraj Singh
//
// Description:
// System-level simulation for the RT-FSS control chain:
//
//   angle_filter -> pid_controller -> pwm_servo
// -----------------------------------------------------------------------------

module tb_rtfss_top;

    // -------------------------------------------------------------------------
    // Clock and reset
    // -------------------------------------------------------------------------
    reg clk;
    reg rst;

    // Approx. 33.33 MHz clock (30 ns period)
    initial begin
        clk = 1'b0;
        forever #15 clk = ~clk;
    end

    initial begin
        rst = 1'b1;
        #200;
        rst = 1'b0;
    end

    // -------------------------------------------------------------------------
    // IMU stimulus signals
    // -------------------------------------------------------------------------
    reg               data_valid;
    reg signed [15:0] accel_y;
    reg signed [15:0] accel_z;
    reg signed [15:0] gyro_x;

    wire signed [15:0] pitch_angle;
    wire               angle_valid;

    // -------------------------------------------------------------------------
    // PID outputs
    // -------------------------------------------------------------------------
    reg  signed [15:0] target_angle;
    wire [7:0]         position_out;
    wire signed [15:0] error_out;
    wire signed [15:0] p_term_dbg;
    wire signed [15:0] i_term_dbg;
    wire signed [15:0] d_term_dbg;
    wire signed [15:0] control_dbg;

    // -------------------------------------------------------------------------
    // PWM output
    // -------------------------------------------------------------------------
    wire servo_pwm;

    // -------------------------------------------------------------------------
    // DUT chain
    // -------------------------------------------------------------------------
    angle_filter u_angle_filter (
        .clk         (clk),
        .rst         (rst),
        .data_valid  (data_valid),
        .accel_y     (accel_y),
        .accel_z     (accel_z),
        .gyro_x      (gyro_x),
        .pitch_angle (pitch_angle),
        .angle_valid (angle_valid)
    );

    pid_controller u_pid_controller (
        .clk          (clk),
        .rst          (rst),
        .angle_valid  (angle_valid),
        .pitch_angle  (pitch_angle),
        .target_angle (target_angle),
        .position_out (position_out),
        .error_out    (error_out),
        .p_term_dbg   (p_term_dbg),
        .i_term_dbg   (i_term_dbg),
        .d_term_dbg   (d_term_dbg),
        .control_dbg  (control_dbg)
    );

    pwm_servo u_pwm_servo (
        .clk      (clk),
        .rst      (rst),
        .position (position_out),
        .dout     (servo_pwm)
    );

    // -------------------------------------------------------------------------
    // 1 ms frame helper
    // -------------------------------------------------------------------------
    localparam integer CYCLES_PER_FRAME = 33333;

    task automatic apply_frame_1ms(
        input signed [15:0] ay,
        input signed [15:0] az,
        input signed [15:0] gx,
        input integer       num_frames,
        input [127:0]       label
    );
        integer f, i;
        begin
            for (f = 0; f < num_frames; f = f + 1) begin
                accel_y = ay;
                accel_z = az;
                gyro_x  = gx;

                @(posedge clk);
                data_valid <= 1'b1;

                @(posedge clk);
                data_valid <= 1'b0;

                for (i = 0; i < CYCLES_PER_FRAME - 2; i = i + 1) begin
                    @(posedge clk);
                end
            end

            if (num_frames > 0)
                $display("=== Finished phase: %s at time %0t ns ===", label, $time);
        end
    endtask

    // -------------------------------------------------------------------------
    // Debug display
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst && angle_valid) begin
            $display("T=%0t ns | pitch=%0d, target=%0d, error=%0d, pos=%0d, P=%0d, I=%0d, D=%0d, CTRL=%0d",
                     $time, pitch_angle, target_angle, error_out,
                     position_out, p_term_dbg, i_term_dbg, d_term_dbg, control_dbg);
        end
    end

    reg last_servo_pwm;
    always @(posedge clk) begin
        if (rst) begin
            last_servo_pwm <= 1'b0;
        end else begin
            last_servo_pwm <= servo_pwm;
            if (servo_pwm && !last_servo_pwm)
                $display("T=%0t ns | PWM rising edge (position_out=%0d)", $time, position_out);
        end
    end

    // -------------------------------------------------------------------------
    // Main stimulus
    // -------------------------------------------------------------------------
    initial begin
        data_valid   = 1'b0;
        accel_y      = 16'sd0;
        accel_z      = 16'sd16384;
        gyro_x       = 16'sd0;
        target_angle = 16'sd0;

        @(negedge rst);
        $display("Reset deasserted at %0t ns", $time);

        apply_frame_1ms(16'sd0,    16'sd16384,  16'sd0,    50, "Level steady");
        apply_frame_1ms(16'sd3000, 16'sd16000,  16'sd0,    80, "Nose-up steady");
        apply_frame_1ms(16'sd2500, 16'sd16000, -16'sd150,  80, "Returning to level");
        apply_frame_1ms(16'sd0,    16'sd16384,  16'sd20,   60, "Small residual motion");
        apply_frame_1ms(16'sd0,    16'sd16384,  16'sd0,    40, "Final level steady");

        $display("Simulation complete at %0t ns", $time);
        #100000;
        $finish;
    end

endmodule
