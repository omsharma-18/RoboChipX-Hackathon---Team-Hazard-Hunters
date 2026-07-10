// =============================================================================
// fusion_engine.v
// -----------------------------------------------------------------------------
// Cross-sensor domain-knowledge rules (README Sec.3 "CROSS-SENSOR FUSION
// ENGINE" block). These are hand-coded because they are cheaper to state
// directly than to hope an MLP with only 72 weights rediscovers them:
//
//   Gas + Flame              -> EXPLOSION_RISK -> CRITICAL
//   Gas + No Flame           -> GAS_LEAK        -> WARNING
//   Depth changed too fast   -> FALL_DETECTED   -> CRITICAL
//
// Consumes the temperature-compensated ultrasonic reading (from
// ultrasonic_temp_comp.v) and the hard-rule flags from threshold_detector.v.
// =============================================================================
// =============================================================================
// fusion_engine.v  (unchanged -- no issues raised against this module)
// =============================================================================
module fusion_engine #(
    parameter DATA_WIDTH = 16
)(
    input  wire                     clk,
    input  wire                     rst,
    input  wire                     vector_valid,
    input  wire                     gas_critical,
    input  wire                     flame_critical,
    input  wire                     ultra_exceeded,
    input  wire signed [DATA_WIDTH:0] ultra_roc,
    output reg                      explosion_risk,
    output reg                      gas_leak,
    output reg                      fall_detected,
    output reg  [3:0]               fusion_anomaly_type,
    output reg                      fusion_critical,
    output reg                      fusion_warning
);
    localparam [3:0]
        AT_NONE            = 4'h0,
        AT_EXPLOSION_RISK  = 4'h6,
        AT_GAS_LEAK        = 4'h7,
        AT_FALL_DETECTED   = 4'h8;
    wire fall_candidate = (ultra_roc >  17'sd200) || (ultra_roc < -17'sd200);
    always @(posedge clk) begin
        if (rst) begin
            explosion_risk       <= 1'b0;
            gas_leak             <= 1'b0;
            fall_detected        <= 1'b0;
            fusion_anomaly_type  <= AT_NONE;
            fusion_critical      <= 1'b0;
            fusion_warning       <= 1'b0;
        end else if (vector_valid) begin
            explosion_risk <= gas_critical && flame_critical;
            gas_leak       <= gas_critical && !flame_critical;
            fall_detected  <= fall_candidate;
            fusion_critical <= (gas_critical && flame_critical) || fall_candidate;
            fusion_warning  <= (gas_critical && !flame_critical);
            if (gas_critical && flame_critical)
                fusion_anomaly_type <= AT_EXPLOSION_RISK;
            else if (fall_candidate)
                fusion_anomaly_type <= AT_FALL_DETECTED;
            else if (gas_critical && !flame_critical)
                fusion_anomaly_type <= AT_GAS_LEAK;
            else
                fusion_anomaly_type <= AT_NONE;
        end
    end
endmodule