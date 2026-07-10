// =============================================================================
// sample_hold_sync.v
// -----------------------------------------------------------------------------
// Sample-rate reconciliation via zero-order hold. Channels arrive at very
// different native rates (gas~10Hz, flame~50Hz, ultrasonic~33Hz, dht11~1Hz).
// Each channel's last filtered value is latched; a coherent 4-D vector is
// formed aligned to CH3 (DHT11, the slowest channel).
//
// SAFETY-CRITICAL REQUIREMENT (README Sec.7 note 2, non-negotiable):
//   vector_valid = all_seen && ch3_valid
//   all_seen     = seen_0 & seen_1 & seen_2 & seen_3
// Each seen_n is a STICKY flag, set on that channel's first valid sample
// post-reset and never cleared. Without this guard the MLP/feature extractor
// could consume uninitialized register values during CALIBRATE / MQ-2's
// ~30s preheat window, producing a meaningless (and potentially unsafe)
// inference. Do NOT gate on ch3_valid alone.
// =============================================================================
// =============================================================================
// sample_hold_sync.v  (unchanged -- no issues raised against this module;
// the all_seen sticky-flag guard was already correctly implemented, per
// Kimi review "What's Solid" list.)
// =============================================================================
module sample_hold_sync #(
    parameter DATA_WIDTH = 16
)(
    input  wire                     clk,
    input  wire                     rst,
    input  wire [DATA_WIDTH-1:0]    ch0_data, input wire ch0_valid,  // gas   (MA filtered)
    input  wire [DATA_WIDTH-1:0]    ch1_data, input wire ch1_valid,  // flame (MA filtered)
    input  wire [DATA_WIDTH-1:0]    ch2_data, input wire ch2_valid,  // ultra (MA filtered)
    input  wire [DATA_WIDTH-1:0]    ch3_data, input wire ch3_valid,  // dht11 (MA filtered)
    output reg  [DATA_WIDTH-1:0]    hold_gas,
    output reg  [DATA_WIDTH-1:0]    hold_flame,
    output reg  [DATA_WIDTH-1:0]    hold_ultra,
    output reg  [DATA_WIDTH-1:0]    hold_temp,
    output reg                      vector_valid,   // 1-cycle pulse, aligned to CH3
    output wire                     all_seen        // exported for debug / STATUS reg
);
    reg seen_0, seen_1, seen_2, seen_3;
    assign all_seen = seen_0 & seen_1 & seen_2 & seen_3;
    always @(posedge clk) begin
        if (rst) begin
            hold_gas     <= {DATA_WIDTH{1'b0}};
            hold_flame   <= {DATA_WIDTH{1'b0}};
            hold_ultra   <= {DATA_WIDTH{1'b0}};
            hold_temp    <= {DATA_WIDTH{1'b0}};
            seen_0       <= 1'b0;
            seen_1       <= 1'b0;
            seen_2       <= 1'b0;
            seen_3       <= 1'b0;
            vector_valid <= 1'b0;
        end else begin
            vector_valid <= 1'b0;
            if (ch0_valid) begin hold_gas   <= ch0_data; seen_0 <= 1'b1; end
            if (ch1_valid) begin hold_flame <= ch1_data; seen_1 <= 1'b1; end
            if (ch2_valid) begin hold_ultra <= ch2_data; seen_2 <= 1'b1; end
            if (ch3_valid) begin hold_temp  <= ch3_data; seen_3 <= 1'b1; end
            if (all_seen && ch3_valid)
                vector_valid <= 1'b1;
        end
    end
endmodule