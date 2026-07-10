// =============================================================================
// system_fsm.v
// -----------------------------------------------------------------------------
// Top-level sequencing FSM: INIT -> CALIBRATE -> ACQUIRE -> PROCESS -> ALERT/FAULT.
// Gates everything in the sensor/decision pipeline on sensor warm-up/readiness.
//
// README Sec.7 note 7 (non-negotiable): WARMUP_CYCLES must be a real localparam
// derived from the target clock (100 MHz) and MQ-2's real preheat time
// (~30s), via a prescaled 1ms-tick counter rather than a raw 32-bit cycle
// counter -- 30s * 100MHz ~= 3*10^9 cycles, which would need a wide counter
// if done as a single free-running register; prescaling to 1ms ticks keeps
// the counter small (15 bits covers 30,000 ticks) while still being exact.
// =============================================================================
// =============================================================================
// system_fsm.v
// -----------------------------------------------------------------------------
// Top-level sequencing FSM: INIT -> CALIBRATE -> ACQUIRE -> PROCESS -> ALERT/FAULT.
// Gates everything in the sensor/decision pipeline on sensor warm-up/readiness.
//
// README Sec.7 note 7 (non-negotiable): WARMUP_CYCLES must be a real localparam
// derived from the target clock (100 MHz) and MQ-2's real preheat time
// (~30s), via a prescaled 1ms-tick counter rather than a raw 32-bit cycle
// counter -- 30s * 100MHz ~= 3*10^9 cycles, which would need a wide counter
// if done as a single free-running register; prescaling to 1ms ticks keeps
// the counter small (15 bits covers 30,000 ticks) while still being exact.
//
// FIX (Kimi review Critical Bug #1): ST_FAULT is only ever entered by the
// case statement below on a cycle where the outer `if (fault_flag)` branch
// was NOT taken -- i.e. fault_flag is already low. So the unconditional
// `fsm_state <= ST_ACQUIRE` here only ever fires once the fault has cleared,
// and does not by itself cause a FAULT/ACQUIRE oscillation. That said, this
// only holds as long as fault_flag is glitch-free and fully registered
// upstream (true in fault_detector.v today). Making the exit condition
// explicit (`if (!fault_flag)`) costs nothing and removes the dependency on
// that assumption -- if a future fault source ever produces a combinational
// or chattering fault_flag, this guard keeps the FSM correctly latched
// instead of silently relying on evaluation order.
// =============================================================================
module system_fsm #(
    parameter CLK_FREQ_HZ    = 100_000_000,
    parameter WARMUP_TIME_MS = 30_000        // MQ-2 preheat, ~30s
)(
    input  wire         clk,
    input  wire         rst,

    input  wire         global_enable,       // from axi4_lite_reg.v CONTROL register
    input  wire         soft_reset,          // from axi4_lite_reg.v CONTROL register
    input  wire         fault_flag,          // from fault_detector.v
    input  wire         all_seen,            // from sample_hold_sync.v
    input  wire  [1:0]  alert_level_in,       // from decision_pipeline.v / hybrid_voter

    output reg   [2:0]  fsm_state,           // exported to STATUS register
    output reg           acquire_en,          // gates sensor front-ends
    output reg           process_en,          // gates decision_pipeline
    output reg  [1:0]   alert_level_out      // latched/gated final alert (FAULT wins upstream already)
);

    localparam MS_TICK_CYCLES = CLK_FREQ_HZ / 1000;         // cycles per 1ms tick
    // WARMUP_CYCLES expressed in 1ms ticks -- exact, and avoids an
    // unnecessarily wide raw-cycle counter (README Sec.7 note 7).
    localparam WARMUP_CYCLES  = WARMUP_TIME_MS;             // in units of 1ms ticks

    localparam [2:0]
        ST_INIT      = 3'd0,
        ST_CALIBRATE = 3'd1,
        ST_ACQUIRE   = 3'd2,
        ST_PROCESS   = 3'd3,
        ST_ALERT     = 3'd4,
        ST_FAULT     = 3'd5;

    reg [31:0] ms_prescale_cnt;
    reg        ms_tick;
    reg [15:0] warmup_cnt;   // 16 bits comfortably covers 30,000 ms ticks

    // 1ms tick generator (prescaled from the 100MHz core clock)
    always @(posedge clk) begin
        if (rst) begin
            ms_prescale_cnt <= 32'd0;
            ms_tick         <= 1'b0;
        end else if (ms_prescale_cnt >= MS_TICK_CYCLES - 1) begin
            ms_prescale_cnt <= 32'd0;
            ms_tick         <= 1'b1;
        end else begin
            ms_prescale_cnt <= ms_prescale_cnt + 32'd1;
            ms_tick         <= 1'b0;
        end
    end

    always @(posedge clk) begin
        if (rst || soft_reset) begin
            fsm_state       <= ST_INIT;
            warmup_cnt      <= 16'd0;
            acquire_en      <= 1'b0;
            process_en      <= 1'b0;
            alert_level_out <= 2'b00;
        end else begin
            // FAULT can preempt any state immediately, from any state.
            if (fault_flag) begin
                fsm_state       <= ST_FAULT;
                process_en      <= 1'b0;
                alert_level_out <= 2'b11; // FAULT
            end else begin
                case (fsm_state)
                    ST_INIT: begin
                        acquire_en <= 1'b0;
                        process_en <= 1'b0;
                        warmup_cnt <= 16'd0;
                        if (global_enable) fsm_state <= ST_CALIBRATE;
                    end

                    ST_CALIBRATE: begin
                        // MQ-2 preheat / sensor warm-up window; acquisition can run
                        // during this window but PS/PL may adjust threshold
                        // registers based on baselines observed here.
                        acquire_en <= 1'b1;
                        process_en <= 1'b0;
                        if (ms_tick) begin
                            if (warmup_cnt >= WARMUP_CYCLES - 1)
                                fsm_state <= ST_ACQUIRE;
                            else
                                warmup_cnt <= warmup_cnt + 16'd1;
                        end
                    end

                    ST_ACQUIRE: begin
                        acquire_en <= 1'b1;
                        process_en <= 1'b0;
                        // Do not start feeding the decision pipeline until every
                        // channel has produced at least one valid sample
                        // (all_seen sticky flag from sample_hold_sync.v).
                        if (all_seen) fsm_state <= ST_PROCESS;
                    end

                    ST_PROCESS: begin
                        acquire_en <= 1'b1;
                        process_en <= 1'b1;
                        fsm_state  <= ST_ALERT;
                    end

                    ST_ALERT: begin
                        acquire_en      <= 1'b1;
                        process_en      <= 1'b1;
                        alert_level_out <= alert_level_in;
                        fsm_state       <= ST_PROCESS; // continuous re-evaluation
                    end

                    ST_FAULT: begin
                        // stays latched in FAULT via the fault_flag branch above;
                        // this case only ever runs on a cycle where fault_flag is
                        // already low (see file header), so gating explicitly on
                        // !fault_flag here is redundant-but-safe defensive coding
                        // rather than a behavior change (Kimi review Bug #1).
                        acquire_en <= 1'b1;
                        process_en <= 1'b0;
                        if (!fault_flag)
                            fsm_state <= ST_ACQUIRE;
                    end

                    default: fsm_state <= ST_INIT;
                endcase
            end
        end
    end

endmodule