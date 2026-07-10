// =============================================================================
// mq2_interface.v
// -----------------------------------------------------------------------------
// DIGITAL REWRITE (README revision: fully digital sensor acquisition, no
// XADC/analog front-end -- see README Sec.2/Sec.3). Reads the MQ-2 module's
// onboard LM393 comparator output (D0 pin) directly: a clean binary
// "gas detected past the onboard-pot threshold" signal, already synchronized
// by sync_2ff.v at the top level before it reaches this module.
//
// Output encoding is deliberate, not arbitrary: gas_raw is driven to
// full-scale ({DATA_WIDTH{1'b1}}) when D0 is asserted, or all-zero
// otherwise -- NOT just a single LSB bit. This matters: downstream,
// true_moving_avg.v averages this over a configurable window (default 64
// samples, README Sec.3) to produce a genuine confidence FRACTION
// (0 = never asserted in the window, full-scale = asserted every sample in
// the window, in between = proportional duty cycle). If we only ever wrote
// a single '1' in the LSB, the running_sum >> window_log2 shift would floor
// to zero for any window fill less than 100%, destroying the debounce
// filter's resolution entirely (README Sec.7 note 4 explains why the
// windowed_filter core is reused this way for binary channels).
//
// No stuck-at fault detection here, by design (README Sec.7 note 3): a
// comparator-only output gives no way to distinguish "genuinely safe,
// holding at 0" from "sensor dead, stuck at 0" -- this is an acknowledged,
// documented limitation of D0-only interfacing, not an oversight.
// =============================================================================
module mq2_interface #(
    parameter OUT_WIDTH        = 16,
    parameter CLK_FREQ_HZ      = 100_000_000,
    parameter SAMPLE_PERIOD_US = 100_000    // ~10Hz native sample rate (README Sec.3)
)(
    input  wire                     clk,
    input  wire                     rst,
    input  wire                     en,          // gated by system_fsm.v acquire_en

    input  wire                     gas_d0_sync, // already passed through sync_2ff.v upstream

    output reg  [OUT_WIDTH-1:0]     gas_raw,     // full-scale or zero (see header)
    output reg                      gas_valid    // pulses at SAMPLE_PERIOD_US rate
);

    localparam [31:0] SAMPLE_PERIOD_CYC = SAMPLE_PERIOD_US * (CLK_FREQ_HZ / 1_000_000);

    reg [31:0] sample_cnt;

    always @(posedge clk) begin
        if (rst) begin
            gas_raw    <= {OUT_WIDTH{1'b0}};
            gas_valid  <= 1'b0;
            sample_cnt <= 32'd0;
        end else begin
            gas_valid <= 1'b0;

            if (en) begin
                if (sample_cnt >= SAMPLE_PERIOD_CYC - 1) begin
                    sample_cnt <= 32'd0;
                    gas_raw    <= gas_d0_sync ? {OUT_WIDTH{1'b1}} : {OUT_WIDTH{1'b0}};
                    gas_valid  <= 1'b1;
                end else begin
                    sample_cnt <= sample_cnt + 32'd1;
                end
            end else begin
                sample_cnt <= 32'd0;
            end
        end
    end

endmodule