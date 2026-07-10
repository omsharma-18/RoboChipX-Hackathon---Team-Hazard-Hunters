// =============================================================================
// flame_interface.v  (unchanged -- no issues raised against this module)
// -----------------------------------------------------------------------------
// Reads the flame IR module's onboard LM393 comparator D0 output, already
// synchronized upstream by sync_2ff.v. Same full-scale/zero encoding
// rationale as mq2_interface.v.
// =============================================================================
module flame_interface #(
    parameter OUT_WIDTH        = 16,
    parameter CLK_FREQ_HZ      = 100_000_000,
    parameter SAMPLE_PERIOD_US = 20_000    // ~50Hz native sample rate (README Sec.3)
)(
    input  wire                     clk,
    input  wire                     rst,
    input  wire                     en,            // gated by system_fsm.v acquire_en
    input  wire                     flame_d0_sync, // already passed through sync_2ff.v upstream
    output reg  [OUT_WIDTH-1:0]     flame_raw,     // full-scale or zero
    output reg                      flame_valid    // pulses at SAMPLE_PERIOD_US rate
);
    localparam [31:0] SAMPLE_PERIOD_CYC = SAMPLE_PERIOD_US * (CLK_FREQ_HZ / 1_000_000);
    reg [31:0] sample_cnt;
    always @(posedge clk) begin
        if (rst) begin
            flame_raw  <= {OUT_WIDTH{1'b0}};
            flame_valid <= 1'b0;
            sample_cnt <= 32'd0;
        end else begin
            flame_valid <= 1'b0;
            if (en) begin
                if (sample_cnt >= SAMPLE_PERIOD_CYC - 1) begin
                    sample_cnt <= 32'd0;
                    flame_raw   <= flame_d0_sync ? {OUT_WIDTH{1'b1}} : {OUT_WIDTH{1'b0}};
                    flame_valid <= 1'b1;
                end else begin
                    sample_cnt <= sample_cnt + 32'd1;
                end
            end else begin
                sample_cnt <= 32'd0;
            end
        end
    end
endmodule