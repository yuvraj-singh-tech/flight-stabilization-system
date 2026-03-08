// -----------------------------------------------------------------------------
// Module: reset_sync
// Project: Real-Time Flight Stabilization System (RT-FSS)
// Author: Yuvraj Singh
//
// Description:
// Synchronizes an asynchronous reset signal to the system clock domain.
// Reset assertion is asynchronous while release is synchronized to the clock
// to avoid metastability in downstream logic.
// -----------------------------------------------------------------------------

module reset_sync (
    input  wire clk,        // System clock (33.333 MHz)
    input  wire rst_raw,    // External asynchronous reset
    output wire rst_sync    // Clean synchronized reset
);

    // Two-flip-flop reset synchronizer
    // Ensures reset deassertion occurs safely in the clock domain
    (* ASYNC_REG = "TRUE" *) reg [1:0] sync_ff = 2'b11;

    always @(posedge clk or posedge rst_raw) begin
        if (rst_raw)
            sync_ff <= 2'b11;          // Immediate reset assertion
        else
            sync_ff <= {1'b0, sync_ff[1]};  // Release reset over two cycles
    end

    assign rst_sync = sync_ff[0];

endmodule
