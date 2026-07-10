// =============================================================================
// sync_2ff.v  (unchanged -- no issues raised against this module)
// -----------------------------------------------------------------------------
// Generic 2-flop synchronizer for any single-bit signal crossing from an
// asynchronous domain into clocked logic. Required on every raw async
// digital input per README Sec.4/Sec.7 note 2: MQ-2 D0, Flame D0, HC-SR04
// Echo, DHT11 data line.
// =============================================================================
module sync_2ff #(
    parameter RESET_VAL = 1'b0
)(
    input  wire clk,
    input  wire rst,
    input  wire async_in,
    output reg  sync_out
);
    reg meta;
    always @(posedge clk) begin
        if (rst) begin
            meta     <= RESET_VAL;
            sync_out <= RESET_VAL;
        end else begin
            meta     <= async_in;
            sync_out <= meta;
        end
    end
endmodule