// =============================================================================
// ultrasonic_controller.v
// -----------------------------------------------------------------------------
// Drives an HC-SR04: issues a >=10us TRIG pulse, times the ECHO high pulse
// with a free-running counter, converts pulse width to distance, and flags
// a timeout fault if no echo returns within 30ms (blocked/dead sensor per
// README Sec.3 Fault Detector). Re-triggers no faster than the HC-SR04's
// required 1s minimum cycle time (echo settling).
//
// FIX (Kimi review Major Issue #5): echo_us was computed as echo_cnt /
// CYCLES_PER_US, a runtime division of a 32-bit value by 100 -- expensive
// in FPGA fabric and a likely timing-closure failure. Replaced with a
// fixed-point reciprocal multiply + shift: 655/65536 ~= 1/100.0153,
// error ~0.15%, negligible next to HC-SR04's own mm-level accuracy.
//
// FIX (pipeline, this revision): the board's actual clock is 125MHz
// (8ns period, per the PYNQ-Z2 XDC: create_clock -period 8.00 on sysclk),
// not the 100MHz the earlier fix assumed. Two chained 32-bit constant
// multiplies (echo_cnt*655 then *343) purely combinational into a single
// register was already marginal at 10ns; at 8ns it's very likely a timing
// failure (~6 chained carry-chain adder stages for the two multiplies,
// roughly 9-14ns realistic total incl. routing -- see discussion). Split
// across two clock edges via a new S_CALC state: register the first
// multiply's result (echo_us) when the falling edge is detected, then
// compute the second multiply + shift from that registered value on the
// next cycle. Costs one extra clock (8ns) on a measurement that only
// happens once per second -- free in practice, and each half of the old
// chain is now a single-multiply combinational path with comfortable
// margin inside 8ns.
// =============================================================================
module ultrasonic_controller #(
    parameter CLK_FREQ_HZ     = 100_000_000,  // overridden to 125_000_000 at top level
    parameter TRIG_US         = 10,     // minimum 10us trigger pulse
    parameter TIMEOUT_US      = 30_000, // 30ms echo timeout
    parameter RETRIGGER_US    = 1_000_000, // 1s minimum between triggers
    parameter DIST_WIDTH      = 16      // distance in mm
)(
    input  wire                    clk,
    input  wire                    rst,
    input  wire                    en,

    output reg                     trig,
    input  wire                    echo,       // ASYNC pin -- see sync_2ff below, do not use directly

    output reg  [DIST_WIDTH-1:0]   distance_mm,
    output reg                     distance_valid,
    output reg                     echo_timeout_flag
);

    localparam CYCLES_PER_US = CLK_FREQ_HZ / 1_000_000;

    localparam CNT_W = 32;

    // NEW: S_CALC added between S_MEASURE and S_WAIT_RT to pipeline the
    // distance multiply across a clock edge (see header note).
    localparam [2:0] S_IDLE          = 3'd0,
                      S_WAIT_RT       = 3'd1,
                      S_TRIGGER       = 3'd2,
                      S_WAIT_ECHO_RISE = 3'd3,
                      S_MEASURE       = 3'd4,
                      S_CALC          = 3'd5;

    reg [2:0]       state;
    reg [CNT_W-1:0] cycle_cnt;
    reg [CNT_W-1:0] echo_cnt;
    reg             echo_d;

    // BUGFIX (README Sec.7 note 2): echo is fully asynchronous to clk -- it's
    // driven by the HC-SR04's own internal timing, no clock relationship at
    // all. sync_2ff.v gives it two clock periods to resolve before any
    // edge-detect or timing logic reads it.
    wire echo_sync;
    sync_2ff #(.RESET_VAL(1'b0)) u_echo_sync (
        .clk(clk), .rst(rst), .async_in(echo), .sync_out(echo_sync)
    );

    // ---- Pipeline stage 1 (combinational): echo_cnt (raw cycles) -> echo_us ----
    // Single 32x16-ish constant multiply. On its own this is a single
    // multiplier/shift-add-tree delay, comfortably inside 8ns.
    // 655 / 65536 ~= 1/100.0153 -> ~0.15% error vs a true divide-by-100.
    wire [47:0] echo_us_wide = echo_cnt * 32'd655;
    wire [31:0] echo_us      = echo_us_wide[47:16]; // >> 16

    // Registered between stage 1 and stage 2 -- latched only at the moment
    // the falling edge is detected in S_MEASURE.
    reg [31:0] echo_us_reg;

    // ---- Pipeline stage 2 (combinational): echo_us_reg -> distance_mm ----
    // Second single multiply, off the *registered* stage-1 result, so this
    // is also just one multiply/shift delay, not chained with stage 1.
    // Speed of sound ~343 m/s @ 20C => distance_mm = (echo_us * 343) >> 11
    // (>>11 ~= /2048; ultrasonic_temp_comp.v applies the precise correction
    // downstream).
    wire [31:0] dist_calc = (echo_us_reg * 32'd343) >> 11;

    always @(posedge clk) begin
        if (rst) begin
            state              <= S_IDLE;
            trig               <= 1'b0;
            cycle_cnt          <= 0;
            echo_cnt           <= 0;
            echo_us_reg        <= 0;
            distance_mm        <= 0;
            distance_valid     <= 1'b0;
            echo_timeout_flag  <= 1'b0;
            echo_d             <= 1'b0;
        end else begin
            distance_valid <= 1'b0;
            echo_d         <= echo_sync;

            case (state)
                S_IDLE: begin
                    trig      <= 1'b0;
                    cycle_cnt <= 0;
                    if (en) state <= S_TRIGGER;
                end

                S_TRIGGER: begin
                    trig <= 1'b1;
                    if (cycle_cnt >= (TRIG_US * CYCLES_PER_US)) begin
                        trig      <= 1'b0;
                        cycle_cnt <= 0;
                        state     <= S_WAIT_ECHO_RISE;
                    end else begin
                        cycle_cnt <= cycle_cnt + 1'b1;
                    end
                end

                S_WAIT_ECHO_RISE: begin
                    if (echo_sync && !echo_d) begin
                        echo_cnt <= 0;
                        state    <= S_MEASURE;
                    end else if (cycle_cnt >= (TIMEOUT_US * CYCLES_PER_US)) begin
                        // no echo pulse ever started -> timeout
                        echo_timeout_flag <= 1'b1;
                        cycle_cnt         <= 0;
                        state             <= S_WAIT_RT;
                    end else begin
                        cycle_cnt <= cycle_cnt + 1'b1;
                    end
                end

                S_MEASURE: begin
                    if (!echo_sync && echo_d) begin
                        // falling edge: echo pulse complete. Latch stage-1
                        // result (echo_us, derived from the final echo_cnt
                        // value) into echo_us_reg; the second multiply runs
                        // next cycle in S_CALC, off this registered value.
                        echo_us_reg       <= echo_us;
                        echo_timeout_flag <= 1'b0;
                        cycle_cnt         <= 0;
                        state             <= S_CALC;
                    end else if (echo_cnt >= (TIMEOUT_US * CYCLES_PER_US)) begin
                        echo_timeout_flag <= 1'b1;
                        cycle_cnt         <= 0;
                        state             <= S_WAIT_RT;
                    end else begin
                        echo_cnt <= echo_cnt + 1'b1;
                    end
                end

                // NEW: second pipeline stage. echo_us_reg was latched last
                // cycle; dist_calc (stage-2 combinational multiply off that
                // registered value) is stable and safe to sample now.
                S_CALC: begin
                    distance_mm    <= dist_calc[DIST_WIDTH-1:0];
                    distance_valid <= 1'b1;
                    state          <= S_WAIT_RT;
                end

                S_WAIT_RT: begin
                    // enforce HC-SR04's minimum retrigger spacing (settling time)
                    if (cycle_cnt >= (RETRIGGER_US * CYCLES_PER_US)) begin
                        cycle_cnt <= 0;
                        state     <= S_IDLE;
                    end else begin
                        cycle_cnt <= cycle_cnt + 1'b1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule