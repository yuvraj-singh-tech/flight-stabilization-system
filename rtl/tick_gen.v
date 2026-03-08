// -----------------------------------------------------------------------------
// Module: tick_gen
// Project: Real-Time Flight Stabilization System (RT-FSS)
// Author: Yuvraj Singh
//
// Description:
// Generates periodic timing pulses from the system clock.
// A 1 kHz tick (1 ms) is used as the main control loop timing.
// A 50 Hz tick (20 ms) is derived from the 1 kHz tick for slower tasks.
// -----------------------------------------------------------------------------

module tick_gen (
    input  wire clk,        // System clock (33.333 MHz)
    input  wire rst,        // Synchronous reset from reset_sync
    output reg  tick_1khz,  // 1 ms timing pulse (1 clock cycle)
    output reg  tick_50hz   // 20 ms timing pulse (1 clock cycle)
);

    // Number of clock cycles for 1 ms period
    localparam integer CYCLES_1KHZ = 33333;

    // 50 Hz tick derived from the 1 kHz base tick
    localparam integer DIV_50HZ_FROM_1KHZ = 20;

    reg [19:0] ctr_1khz = 20'd0;
    reg [4:0]  ctr_50hz = 5'd0;

    always @(posedge clk) begin
        if (rst) begin
            ctr_1khz  <= 20'd0;
            ctr_50hz  <= 5'd0;
            tick_1khz <= 1'b0;
            tick_50hz <= 1'b0;
        end else begin
            // Generate 1 kHz timing pulse
            if (ctr_1khz == CYCLES_1KHZ - 1) begin
                ctr_1khz  <= 20'd0;
                tick_1khz <= 1'b1;
            end else begin
                ctr_1khz  <= ctr_1khz + 1'b1;
                tick_1khz <= 1'b0;
            end

            // Generate 50 Hz pulse using the 1 kHz tick
            if (tick_1khz) begin
                if (ctr_50hz == DIV_50HZ_FROM_1KHZ - 1) begin
                    ctr_50hz  <= 5'd0;
                    tick_50hz <= 1'b1;
                end else begin
                    ctr_50hz  <= ctr_50hz + 1'b1;
                    tick_50hz <= 1'b0;
                end
            end else begin
                tick_50hz <= 1'b0;
            end
        end
    end

endmodule
