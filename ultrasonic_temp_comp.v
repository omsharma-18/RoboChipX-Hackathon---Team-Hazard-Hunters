// =============================================================================
// ultrasonic_temp_comp.v
// -----------------------------------------------------------------------------
// Applies the classic speed-of-sound temperature correction:
//     v(T) = 331.4 + 0.6*T   (m/s, T in deg C)
// ultrasonic_controller.v computes a baseline distance assuming a fixed
// v=343 m/s (20 deg C). This module rescales that baseline distance using
// the DHT11's actual temperature reading -- "free accuracy" per README Sec.3.
//
// FIX (Kimi review Major Issue #6): distance_comp_mm was computed via
// scale_num / 32'sd3430, a runtime division of a wide value by a non-power-
// of-2 constant. Replaced with a fixed-point reciprocal multiply + shift:
// 305 / 2^20 = 0.00029087, vs true 1/3430 = 0.00029155 -> ~0.22% error,
// well under the sensor's own accuracy budget.
//
// FIX (pipeline, this revision): the board's actual clock is 125MHz (8ns
// period, per PYNQ-Z2 XDC), not the 100MHz the earlier fix assumed. This
// module chained THREE multiplies purely combinationally into one register
// (v_actual's 6*temp_c, then scale_num = distance_raw_mm*v_actual, then
// scale_wide = scale_num*305) -- worse than ultrasonic_controller.v's
// two-multiply chain, and that one was already borderline at 10ns. Split
// across two clock edges: register scale_num (the first two, cheap,
// multiplies) when distance_valid pulses, then compute the second
// (wider) multiply + shift from that registered value on the next cycle.
// Costs one extra clock (8ns) on a signal that only updates roughly once
// per second -- free in practice.
// =============================================================================
module ultrasonic_temp_comp #(
    parameter DIST_WIDTH = 16
)(
    input  wire                        clk,
    input  wire                        rst,
    input  wire [DIST_WIDTH-1:0]       distance_raw_mm,   // computed assuming v=343 m/s
    input  wire                        distance_valid,
    input  wire signed [15:0]          temp_c,            // DHT11 temperature, integer deg C
    output reg  [DIST_WIDTH-1:0]       distance_comp_mm,
    output reg                         distance_comp_valid
);

    // ---- Pipeline stage 1 (combinational): v_actual + scale_num ----
    // v_actual is a small add + a 3-bit-constant (6) multiply -- collapses
    // to a couple of LUT levels, cheap enough to leave feeding straight
    // into the same-cycle scale_num multiply without its own register.
    // v_actual(mm/s *10 fixed point) = 3314 + 6*T (avoids fractional 0.6 literal)
    wire signed [31:0] v_actual      = 32'sd3314 + (32'sd6 * temp_c);
    wire signed [31:0] scale_num_comb = $signed({1'b0, distance_raw_mm}) * v_actual;

    // Registered between stage 1 and stage 2.
    reg signed [31:0] scale_num_reg;
    reg                stage1_valid;

    always @(posedge clk) begin
        if (rst) begin
            scale_num_reg <= 32'sd0;
            stage1_valid  <= 1'b0;
        end else begin
            stage1_valid <= distance_valid;
            if (distance_valid)
                scale_num_reg <= scale_num_comb;
        end
    end

    // ---- Pipeline stage 2 (combinational): scale_num_reg -> distance_comp ----
    // Second multiply, off the *registered* stage-1 result, so this is a
    // single multiply + shift delay, not chained with the stage-1 math.
    // distance_comp = distance_raw * v_actual / v_baseline(=3430, i.e. 343.0 m/s *10)
    // reciprocal-multiply: 305 / 2^20 ~= 1/3430, widened to 48 bits before
    // the *305 to avoid overflow, then >>20.
    wire signed [47:0] scale_wide         = scale_num_reg * 48'sd305;
    wire signed [47:0] distance_comp_wide = scale_wide >>> 20;

    always @(posedge clk) begin
        if (rst) begin
            distance_comp_mm    <= {DIST_WIDTH{1'b0}};
            distance_comp_valid <= 1'b0;
        end else begin
            distance_comp_valid <= 1'b0;
            if (stage1_valid) begin
                distance_comp_mm    <= distance_comp_wide[DIST_WIDTH-1:0];
                distance_comp_valid <= 1'b1;
            end
        end
    end

endmodule