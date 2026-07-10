// =============================================================================
// hybrid_voter.v
// -----------------------------------------------------------------------------
// Combines hard-rule and Tiny MLP verdicts into the final alert_level.
// Priority is FIXED and must never be made "smarter" (README Sec.7 note 6):
//
//   IF hard_rule = CRITICAL           -> CRITICAL  (never overridden by ML)
//   IF hard_rule = NORMAL, ML=UNSAFE  -> WARNING
//   IF hard_rule = NORMAL, ML=SAFE    -> NORMAL
//   IF hard_rule and ML disagree      -> WARNING   (conservative default)
//
// FAULT is handled upstream/downstream (fault_flag forces FAULT regardless
// of this module's output -- see decision_pipeline.v).
// =============================================================================
// =============================================================================
// hybrid_voter.v  (unchanged -- no issues raised against this module)
// =============================================================================
module hybrid_voter (
    input  wire        clk,
    input  wire         rst,
    input  wire         vector_valid,
    input  wire         hard_rule_critical,
    input  wire         mlp_valid,
    input  wire         mlp_class,
    input  wire         mlp_enabled,
    output reg  [1:0]   alert_level
);
    localparam [1:0]
        NORMAL   = 2'b00,
        WARNING  = 2'b01,
        CRITICAL = 2'b10;
    wire ml_says_unsafe = mlp_enabled && mlp_valid && mlp_class;
    wire ml_says_safe   = mlp_enabled && mlp_valid && !mlp_class;
    always @(posedge clk) begin
        if (rst) begin
            alert_level <= NORMAL;
        end else if (vector_valid || mlp_valid) begin
            if (hard_rule_critical) begin
                alert_level <= CRITICAL;
            end else if (!mlp_enabled || !mlp_valid) begin
                alert_level <= NORMAL;
            end else if (ml_says_unsafe) begin
                alert_level <= WARNING;
            end else if (ml_says_safe) begin
                alert_level <= NORMAL;
            end else begin
                alert_level <= WARNING;
            end
        end
    end
endmodule