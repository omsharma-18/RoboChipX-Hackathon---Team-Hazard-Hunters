// =============================================================================
// threshold_detector.v
// -----------------------------------------------------------------------------
// Hard-rule comparators -- the deterministic, auditable, life-safety layer
// that the ML output can NEVER override (README Sec.7 note 6). Thresholds
// are runtime-configurable via AXI4-Lite (axi4_lite_reg.v), Q1.15 fixed point
// to match the register map (README Sec.5).
//
//   Gas   > THRESH_GAS   -> immediate CRITICAL, ML cannot override
//   Flame detected at all -> immediate CRITICAL, ML cannot override
//   Ultrasonic delta      -> fall-detection candidate (WARNING-level input to fusion_engine)
//   Temp  > THRESH_TEMP   -> WARNING-level input to fusion_engine
// =============================================================================
// =============================================================================
// threshold_detector.v  (unchanged -- no issues raised against this module)
// =============================================================================
module threshold_detector #(
    parameter DATA_WIDTH = 16
)(
    input  wire                          clk,
    input  wire                          rst,
    input  wire signed [DATA_WIDTH-1:0]  gas_mean,
    input  wire signed [DATA_WIDTH-1:0]  flame_mean,
    input  wire signed [DATA_WIDTH-1:0]  ultra_mean,
    input  wire signed [DATA_WIDTH-1:0]  temp_mean,
    input  wire                          vector_valid,
    input  wire signed [DATA_WIDTH-1:0]  thresh_gas,
    input  wire signed [DATA_WIDTH-1:0]  thresh_flame,
    input  wire signed [DATA_WIDTH-1:0]  thresh_ultra,
    input  wire signed [DATA_WIDTH-1:0]  thresh_temp,
    output reg   gas_critical,
    output reg   flame_critical,
    output reg   ultra_exceeded,
    output reg   temp_exceeded,
    output reg   hard_rule_critical
);
    always @(posedge clk) begin
        if (rst) begin
            gas_critical       <= 1'b0;
            flame_critical     <= 1'b0;
            ultra_exceeded     <= 1'b0;
            temp_exceeded      <= 1'b0;
            hard_rule_critical <= 1'b0;
        end else if (vector_valid) begin
            gas_critical   <= (gas_mean   > thresh_gas);
            flame_critical <= (flame_mean > thresh_flame);
            ultra_exceeded <= (ultra_mean > thresh_ultra) || (ultra_mean < -thresh_ultra);
            temp_exceeded  <= (temp_mean  > thresh_temp);
            hard_rule_critical <= (gas_mean > thresh_gas) || (flame_mean > thresh_flame);
        end
    end
endmodule