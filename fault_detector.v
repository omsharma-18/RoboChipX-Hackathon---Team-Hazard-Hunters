// =============================================================================
// fault_detector.v
// -----------------------------------------------------------------------------
// Combines per-sensor health signals into a single fault_flag and an encoded
// fault_type. Any fault forces alert_level=FAULT and hard-rules-only mode
// downstream (decision_pipeline.v enforces the actual MLP disable, per
// README Sec.7 note 5 -- this module only reports, it does not gate).
//
// DIGITAL REWRITE (README revision): gas_stuck_flag / flame_stuck_flag were
// removed. MQ-2 and Flame IR now interface via their onboard comparator's
// D0 pin (binary detected/not-detected), and a comparator output gives no
// way to distinguish "genuinely safe, holding at 0" from "sensor dead,
// stuck at 0" -- unlike the old analog path where a frozen ADC code was a
// real anomaly signal. This is an acknowledged, documented limitation of
// D0-only interfacing (README Sec.7 note 3), not an oversight. Only
// ultrasonic (echo timeout) and DHT11 (checksum/timeout) produce genuine,
// detectable fault conditions in this design.
// =============================================================================
// =============================================================================
// fault_detector.v  (unchanged -- no issues raised against this module)
// -----------------------------------------------------------------------------
// Combines per-sensor health signals into a single fault_flag and encoded
// fault_type. Only ultrasonic (echo timeout) and DHT11 (checksum/timeout)
// produce genuine, detectable fault conditions in D0-only mode -- MQ-2/Flame
// stuck-at detection is an acknowledged, documented limitation (README
// Sec.7 note 3), not an oversight.
// =============================================================================
module fault_detector (
    input  wire       clk,
    input  wire       rst,
    input  wire       echo_timeout_flag,     // ultrasonic_controller.v
    input  wire       dht_checksum_fail_flag,// dht11_controller.v
    input  wire       dht_timeout_flag,      // dht11_controller.v
    output reg        fault_flag,
    output reg [3:0]  fault_type    // encoded, priority order below
);
    localparam [3:0]
        FT_NONE          = 4'h0,
        FT_ULTRA_TIMEOUT = 4'h1,
        FT_DHT_CHECKSUM  = 4'h2,
        FT_DHT_TIMEOUT   = 4'h3;
    always @(posedge clk) begin
        if (rst) begin
            fault_flag <= 1'b0;
            fault_type <= FT_NONE;
        end else begin
            if (echo_timeout_flag) begin
                fault_flag <= 1'b1;
                fault_type <= FT_ULTRA_TIMEOUT;
            end else if (dht_checksum_fail_flag) begin
                fault_flag <= 1'b1;
                fault_type <= FT_DHT_CHECKSUM;
            end else if (dht_timeout_flag) begin
                fault_flag <= 1'b1;
                fault_type <= FT_DHT_TIMEOUT;
            end else begin
                fault_flag <= 1'b0;
                fault_type <= FT_NONE;
            end
        end
    end
endmodule