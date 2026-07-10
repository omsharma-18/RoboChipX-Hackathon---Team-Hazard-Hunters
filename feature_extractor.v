// =============================================================================
// feature_extractor.v
// -----------------------------------------------------------------------------
// Per-channel (time-domain only -- no FFT, sensors are far too slow to need
// spectral analysis per README Sec.3) feature extraction:
//   - running mean          (Welford's online algorithm: 2 running registers)
//   - running variance      (Welford's online algorithm)
//   - max / min / peak-to-peak
//   - rate-of-change        (delta from previous sample)
// Updates once per vector_valid pulse from sample_hold_sync.v (~1Hz, aligned
// to DHT11). Instantiated once per channel via the internal channel_stats
// helper below.
// =============================================================================

// =============================================================================
// feature_extractor.v  (unchanged -- see note below on Kimi review Bug #4)
// -----------------------------------------------------------------------------
// NOTE on Kimi review Major Issue #4 ("p2p_out lags by one sample"): on
// inspection this is already NOT a bug in the pasted RTL. p2p_out is not
// simply "reads max_out/min_out the same cycle they update" (which would
// indeed lag by one sample under non-blocking semantics) -- it explicitly
// re-derives this cycle's true max/min with
//     (sample_in > max_out ? sample_in : max_out) - (sample_in < min_out ? sample_in : min_out)
// which folds the *current* sample_in into the comparison rather than just
// reading the old registered max_out/min_out. That expression evaluates to
// exactly the same new-max/new-min pair that gets latched into max_out/
// min_out on this same clock edge, so p2p_out and max_out/min_out update in
// lockstep -- no one-sample lag. No code change made here; documenting the
// review finding as already resolved in this version of the file rather than
// silently dropping it.
// -----------------------------------------------------------------------------
// Per-channel (time-domain only) feature extraction: running mean/variance
// (Welford's online algorithm), max/min/peak-to-peak, rate-of-change.
// Updates once per vector_valid pulse from sample_hold_sync.v (~1Hz).
// =============================================================================

module channel_stats #(
    parameter DATA_WIDTH = 16
)(
    input  wire                          clk,
    input  wire                          rst,
    input  wire                          sample_valid,
    input  wire signed [DATA_WIDTH-1:0]  sample_in,

    output reg  signed [DATA_WIDTH-1:0]  mean_out,
    output reg  [DATA_WIDTH-1:0]         variance_out,
    output reg  signed [DATA_WIDTH-1:0]  max_out,
    output reg  signed [DATA_WIDTH-1:0]  min_out,
    output reg  [DATA_WIDTH-1:0]         p2p_out,
    output reg  signed [DATA_WIDTH:0]    roc_out       // rate-of-change, 1 extra bit for sign headroom
);

    reg [31:0]                     n_count;      // sample count (Welford)
    reg signed [DATA_WIDTH+15:0]   m2;            // sum of squared deltas (Welford)
    reg signed [DATA_WIDTH-1:0]    prev_sample;
    reg                            has_prev;

    wire signed [DATA_WIDTH:0]      delta      = sample_in - mean_out;
    wire signed [DATA_WIDTH+15:0]   delta_scaled = delta; // widened for mult
    wire signed [32:0]               n_count_p1     = {1'b0, n_count} + 33'sd1;
    wire signed [DATA_WIDTH:0]      new_mean_delta = delta / n_count_p1[DATA_WIDTH:0]; // delta/n
    wire signed [DATA_WIDTH:0]      delta2     = sample_in - (mean_out + new_mean_delta[DATA_WIDTH-1:0]);

    always @(posedge clk) begin
        if (rst) begin
            mean_out     <= {DATA_WIDTH{1'b0}};
            variance_out <= {DATA_WIDTH{1'b0}};
            max_out      <= {1'b1, {(DATA_WIDTH-1){1'b0}}}; // most negative
            min_out      <= {1'b0, {(DATA_WIDTH-1){1'b1}}}; // most positive
            p2p_out      <= {DATA_WIDTH{1'b0}};
            roc_out      <= {(DATA_WIDTH+1){1'b0}};
            n_count      <= 32'd0;
            m2           <= 0;
            prev_sample  <= {DATA_WIDTH{1'b0}};
            has_prev     <= 1'b0;
        end else if (sample_valid) begin
            // ---- Welford running mean/variance ----
            n_count  <= n_count + 32'd1;
            mean_out <= mean_out + new_mean_delta[DATA_WIDTH-1:0];
            m2       <= m2 + (delta * delta2);
            if (n_count > 32'd0)
                variance_out <= (m2 + (delta * delta2)) / n_count;

            // ---- max / min / peak-to-peak ----
            if (sample_in > max_out) max_out <= sample_in;
            if (sample_in < min_out) min_out <= sample_in;
            p2p_out <= (sample_in > max_out ? sample_in : max_out) -
                       (sample_in < min_out ? sample_in : min_out);

            // ---- rate of change ----
            if (has_prev)
                roc_out <= sample_in - prev_sample;
            else
                roc_out <= {(DATA_WIDTH+1){1'b0}};

            prev_sample <= sample_in;
            has_prev    <= 1'b1;
        end
    end

endmodule


module feature_extractor #(
    parameter DATA_WIDTH = 16
)(
    input  wire                          clk,
    input  wire                          rst,
    input  wire                          vector_valid,     // from sample_hold_sync.v

    input  wire signed [DATA_WIDTH-1:0]  gas_in,
    input  wire signed [DATA_WIDTH-1:0]  flame_in,
    input  wire signed [DATA_WIDTH-1:0]  ultra_in,
    input  wire signed [DATA_WIDTH-1:0]  temp_in,

    output wire signed [DATA_WIDTH-1:0]  gas_mean,   output wire [DATA_WIDTH-1:0] gas_var,
    output wire signed [DATA_WIDTH-1:0]  gas_max,    output wire signed [DATA_WIDTH-1:0] gas_min,
    output wire [DATA_WIDTH-1:0]         gas_p2p,    output wire signed [DATA_WIDTH:0]   gas_roc,

    output wire signed [DATA_WIDTH-1:0]  flame_mean, output wire [DATA_WIDTH-1:0] flame_var,
    output wire signed [DATA_WIDTH-1:0]  flame_max,  output wire signed [DATA_WIDTH-1:0] flame_min,
    output wire [DATA_WIDTH-1:0]         flame_p2p,  output wire signed [DATA_WIDTH:0]   flame_roc,

    output wire signed [DATA_WIDTH-1:0]  ultra_mean, output wire [DATA_WIDTH-1:0] ultra_var,
    output wire signed [DATA_WIDTH-1:0]  ultra_max,  output wire signed [DATA_WIDTH-1:0] ultra_min,
    output wire [DATA_WIDTH-1:0]         ultra_p2p,  output wire signed [DATA_WIDTH:0]   ultra_roc,

    output wire signed [DATA_WIDTH-1:0]  temp_mean,  output wire [DATA_WIDTH-1:0] temp_var,
    output wire signed [DATA_WIDTH-1:0]  temp_max,   output wire signed [DATA_WIDTH-1:0] temp_min,
    output wire [DATA_WIDTH-1:0]         temp_p2p,   output wire signed [DATA_WIDTH:0]   temp_roc
);

    channel_stats #(.DATA_WIDTH(DATA_WIDTH)) u_gas (
        .clk(clk), .rst(rst), .sample_valid(vector_valid), .sample_in(gas_in),
        .mean_out(gas_mean), .variance_out(gas_var), .max_out(gas_max),
        .min_out(gas_min), .p2p_out(gas_p2p), .roc_out(gas_roc)
    );

    channel_stats #(.DATA_WIDTH(DATA_WIDTH)) u_flame (
        .clk(clk), .rst(rst), .sample_valid(vector_valid), .sample_in(flame_in),
        .mean_out(flame_mean), .variance_out(flame_var), .max_out(flame_max),
        .min_out(flame_min), .p2p_out(flame_p2p), .roc_out(flame_roc)
    );

    channel_stats #(.DATA_WIDTH(DATA_WIDTH)) u_ultra (
        .clk(clk), .rst(rst), .sample_valid(vector_valid), .sample_in(ultra_in),
        .mean_out(ultra_mean), .variance_out(ultra_var), .max_out(ultra_max),
        .min_out(ultra_min), .p2p_out(ultra_p2p), .roc_out(ultra_roc)
    );

    channel_stats #(.DATA_WIDTH(DATA_WIDTH)) u_temp (
        .clk(clk), .rst(rst), .sample_valid(vector_valid), .sample_in(temp_in),
        .mean_out(temp_mean), .variance_out(temp_var), .max_out(temp_max),
        .min_out(temp_min), .p2p_out(temp_p2p), .roc_out(temp_roc)
    );

endmodule