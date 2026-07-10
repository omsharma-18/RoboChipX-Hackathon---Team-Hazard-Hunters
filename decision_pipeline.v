// =============================================================================
// decision_pipeline.v
// -----------------------------------------------------------------------------
// Wraps feature_extractor -> threshold_detector -> tiny_mlp -> fusion_engine
// -> hybrid_voter into a single pipeline stage (README Sec.6, modules 9-14).
//
// SAFETY-CRITICAL REQUIREMENT (README Sec.7 note 5, non-negotiable):
//   When fault_flag is asserted, this module forces hard-rules-only mode --
//   a sensor that has failed its checksum or timed out must never be allowed
//   to feed the ML layer, since a compromised input could cause the MLP to
//   output a false "safe". mlp_enabled is gated here and passed to
//   hybrid_voter.v, which independently also never lets ML override a
//   hard-rule CRITICAL.
// =============================================================================
// =============================================================================
// decision_pipeline.v
// -----------------------------------------------------------------------------
// Wraps feature_extractor -> threshold_detector -> tiny_mlp -> fusion_engine
// -> hybrid_voter into a single pipeline stage (README Sec.6, modules 9-14).
//
// SAFETY-CRITICAL REQUIREMENT (README Sec.7 note 5, non-negotiable):
//   When fault_flag is asserted, this module forces hard-rules-only mode --
//   a sensor that has failed its checksum or timed out must never be allowed
//   to feed the ML layer, since a compromised input could cause the MLP to
//   output a false "safe". mlp_enabled is gated here and passed to
//   hybrid_voter.v, which independently also never lets ML override a
//   hard-rule CRITICAL.
//
// FIX (Kimi review Major Issue #7): confidence was previously always
// assign confidence = mlp_confidence, even during FAULT when mlp_confidence
// is meaningless (MLP is disabled and its last-computed score is stale).
// The dashboard would show a misleading confidence number during FAULT.
// Now gated to 0 whenever fault_flag is set, matching the alert_level/
// anomaly_type gating that was already correct just below.
// =============================================================================
module decision_pipeline #(
    parameter DATA_WIDTH = 16
)(
    input  wire                          clk,
    input  wire                          rst,

    input  wire                          vector_valid,       // from sample_hold_sync.v
    input  wire signed [DATA_WIDTH-1:0]  hold_gas,
    input  wire signed [DATA_WIDTH-1:0]  hold_flame,
    input  wire signed [DATA_WIDTH-1:0]  hold_ultra_comp,    // temp-compensated (ultrasonic_temp_comp.v)
    input  wire signed [DATA_WIDTH-1:0]  hold_temp,

    input  wire                          fault_flag,         // from fault_detector.v

    // AXI4-Lite configured thresholds (Q1.15)
    input  wire signed [DATA_WIDTH-1:0]  thresh_gas,
    input  wire signed [DATA_WIDTH-1:0]  thresh_flame,
    input  wire signed [DATA_WIDTH-1:0]  thresh_ultra,
    input  wire signed [DATA_WIDTH-1:0]  thresh_temp,

    // MLP weight load passthrough (AXI4-Lite -> tiny_mlp.v)
    input  wire                          weight_wr_en,
    input  wire [6:0]                    weight_wr_addr,
    input  wire signed [DATA_WIDTH-1:0]  weight_wr_data,

    output wire [1:0]                    alert_level,        // 00=NORMAL 01=WARNING 10=CRITICAL
    output wire [3:0]                    anomaly_type,
    output wire [7:0]                    confidence
);

    // ---------------- feature extraction ----------------
    wire signed [DATA_WIDTH-1:0] gas_mean, flame_mean, ultra_mean, temp_mean;
    wire [DATA_WIDTH-1:0] gas_var, flame_var, ultra_var, temp_var;
    wire signed [DATA_WIDTH-1:0] gas_max, gas_min, flame_max, flame_min,
                                  ultra_max, ultra_min, temp_max, temp_min;
    wire [DATA_WIDTH-1:0] gas_p2p, flame_p2p, ultra_p2p, temp_p2p;
    wire signed [DATA_WIDTH:0] gas_roc, flame_roc, ultra_roc, temp_roc;

    feature_extractor #(.DATA_WIDTH(DATA_WIDTH)) u_features (
        .clk(clk), .rst(rst), .vector_valid(vector_valid),
        .gas_in(hold_gas), .flame_in(hold_flame), .ultra_in(hold_ultra_comp), .temp_in(hold_temp),
        .gas_mean(gas_mean), .gas_var(gas_var), .gas_max(gas_max), .gas_min(gas_min), .gas_p2p(gas_p2p), .gas_roc(gas_roc),
        .flame_mean(flame_mean), .flame_var(flame_var), .flame_max(flame_max), .flame_min(flame_min), .flame_p2p(flame_p2p), .flame_roc(flame_roc),
        .ultra_mean(ultra_mean), .ultra_var(ultra_var), .ultra_max(ultra_max), .ultra_min(ultra_min), .ultra_p2p(ultra_p2p), .ultra_roc(ultra_roc),
        .temp_mean(temp_mean), .temp_var(temp_var), .temp_max(temp_max), .temp_min(temp_min), .temp_p2p(temp_p2p), .temp_roc(temp_roc)
    );

    // ---------------- hard-rule threshold layer ----------------
    wire gas_critical, flame_critical, ultra_exceeded, temp_exceeded, hard_rule_critical_raw;

    threshold_detector #(.DATA_WIDTH(DATA_WIDTH)) u_thresh (
        .clk(clk), .rst(rst),
        .gas_mean(gas_mean), .flame_mean(flame_mean), .ultra_mean(ultra_mean), .temp_mean(temp_mean),
        .vector_valid(vector_valid),
        .thresh_gas(thresh_gas), .thresh_flame(thresh_flame), .thresh_ultra(thresh_ultra), .thresh_temp(thresh_temp),
        .gas_critical(gas_critical), .flame_critical(flame_critical),
        .ultra_exceeded(ultra_exceeded), .temp_exceeded(temp_exceeded),
        .hard_rule_critical(hard_rule_critical_raw)
    );

    // ---------------- Tiny MLP (disabled whenever fault_flag is set) ----------------
    wire mlp_valid, mlp_class;
    wire [7:0] mlp_confidence;
    wire mlp_enabled = ~fault_flag;   // <-- the non-negotiable gate (Sec.7 note 5)

    tiny_mlp #(.DATA_WIDTH(DATA_WIDTH)) u_mlp (
        .clk(clk), .rst(rst),
        .start(vector_valid && mlp_enabled),
        .gas_norm(gas_mean), .flame_norm(flame_mean), .ultra_norm(ultra_mean), .temp_norm(temp_mean),
        .weight_wr_en(weight_wr_en), .weight_wr_addr(weight_wr_addr), .weight_wr_data(weight_wr_data),
        .mlp_valid(mlp_valid), .mlp_class(mlp_class), .mlp_confidence(mlp_confidence)
    );

    // ---------------- cross-sensor fusion ----------------
    wire explosion_risk, gas_leak, fall_detected, fusion_critical, fusion_warning;
    wire [3:0] fusion_anomaly_type;

    fusion_engine #(.DATA_WIDTH(DATA_WIDTH)) u_fusion (
        .clk(clk), .rst(rst), .vector_valid(vector_valid),
        .gas_critical(gas_critical), .flame_critical(flame_critical), .ultra_exceeded(ultra_exceeded),
        .ultra_roc(ultra_roc),
        .explosion_risk(explosion_risk), .gas_leak(gas_leak), .fall_detected(fall_detected),
        .fusion_anomaly_type(fusion_anomaly_type),
        .fusion_critical(fusion_critical), .fusion_warning(fusion_warning)
    );

    wire hard_rule_critical = hard_rule_critical_raw || fusion_critical;

    // ---------------- hybrid vote ----------------
    wire [1:0] alert_level_internal;

    hybrid_voter u_voter (
        .clk(clk), .rst(rst),
        .vector_valid(vector_valid),
        .hard_rule_critical(hard_rule_critical),
        .mlp_valid(mlp_valid), .mlp_class(mlp_class), .mlp_enabled(mlp_enabled),
        .alert_level(alert_level_internal)
    );

    // FAULT is the highest-priority state of all -- overrides voter output entirely.
    assign alert_level  = fault_flag ? 2'b11 : alert_level_internal;
    assign anomaly_type = fault_flag ? 4'hF : fusion_anomaly_type;

    // FIX (Kimi review Bug #7): gate confidence to 0 during FAULT instead of
    // always passing through mlp_confidence (which is stale/meaningless once
    // mlp_enabled goes low -- the MLP simply stops being started, so its
    // output registers hold whatever they last computed before the fault).
    assign confidence   = fault_flag ? 8'd0 : mlp_confidence;

endmodule