// -----------------------------------------------------------------------------
// Module: pwm_servo
// Project: Real-Time Flight Stabilization System (RT-FSS)
// Author: Yuvraj Singh
//
// Description:
// Generates a standard 50 Hz servo PWM signal from an 8-bit position command.
// The position input (0..255) is mapped to a pulse width between ~1 ms and
// ~2 ms inside a 20 ms frame. The pulse width is updated once per frame to
// ensure stable servo behavior.
// -----------------------------------------------------------------------------

module pwm_servo(
    input  wire       clk,
    input  wire       rst,
    input  wire [7:0] position,   // 0..255 position command
    output reg        dout
);

    // Clock frequency of SP701 board (~33.33 MHz)
    parameter integer CLK_FREQ = 33333333;

    // 50 Hz frame period (20 ms)
    parameter integer PERIOD = 666666;

    // Servo pulse limits
    parameter integer TON_MIN = 33333;   // ~1.0 ms
    parameter integer TON_MAX = 66666;   // ~2.0 ms

    // Pulse span used for position mapping
    localparam integer PULSE_RANGE = TON_MAX - TON_MIN;

    integer count = 0;
    integer ton   = 0;

    // Indicates start of a new PWM frame
    reg ncyc = 1'b0;

    // -------------------------------------------------------------------------
    // PWM frame generator
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            count <= 0;
            ton   <= (TON_MIN + TON_MAX) / 2;
            dout  <= 1'b0;
            ncyc  <= 1'b0;
        end else begin
            if (count <= ton) begin
                dout  <= 1'b1;
                count <= count + 1;
                ncyc  <= 1'b0;
            end
            else if (count < PERIOD) begin
                dout  <= 1'b0;
                count <= count + 1;
                ncyc  <= 1'b0;
            end
            else begin
                dout  <= 1'b0;
                count <= 0;
                ncyc  <= 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Update pulse width once per frame
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            ton <= (TON_MIN + TON_MAX) / 2;
        end
        else if (ncyc) begin
            ton <= TON_MIN + (PULSE_RANGE * position) / 255;
        end
    end

endmodule
