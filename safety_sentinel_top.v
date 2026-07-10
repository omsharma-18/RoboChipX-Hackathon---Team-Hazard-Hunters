// =============================================================================
// safety_sentinel_top.v
// -----------------------------------------------------------------------------
// Top-level interconnect for the Edge AI Safety System (Problem 05: Edge
// Analytics IP, SDG 9 & SDG 11). Wraps every module in README Sec.6's build
// order into one Vivado-packageable IP core.
//
// DIGITAL REWRITE (README revision): fully digital sensor acquisition, no
// XADC/analog front-end anywhere. Physical I/O mapping now:
//   MQ-2   -> onboard LM393 comparator D0 pin -> PL GPIO in (PMODA)
//   Flame  -> onboard LM393 comparator D0 pin -> PL GPIO in (PMODA)
//   HC-SR04-> trig (PL GPIO out), echo (PL GPIO in)          (PMODB)
//   DHT11  -> single GPIO pin, bidirectional single-bus       (PMODB)
//
// CHANGE LOG vs previous revision (per Kimi RTL review + follow-up discussion):
//   * REMOVED: uart_tx_line / uart_tx instantiation / all UART framing logic.
//     PYNQ-Z2's onboard USB-UART is hardwired to PS UART0 (MIO), not
//     reachable from PL fabric without EMIO or a PMOD dongle. README Sec.4/5
//     already specify the dashboard as a PYNQ Jupyter notebook polling
//     AXI4-Lite registers directly (overlay.mmio.read(offset)) -- the PL
//     UART was redundant for that use case and is deleted, not rerouted.
//   * ADDED: wiring for the 0x200-0x214 AXI4-Lite dashboard readback block
//     (Kimi review Critical Bug #3) -- alert_level/anomaly_type/confidence,
//     timestamp, and all 4 filtered channel values are now exposed to the
//     PS via axi4_lite_reg.v instead of only existing on internal wires.
//   * ADDED: a real ~2Hz blink generator driving led_rgb's flash bit during
//     FAULT (Kimi review Bug #8) -- previously a static bit with no toggle
//     source behind it.
//   * FIXED (clock frequency): the board's actual PL clock is 125MHz, per
//     the PYNQ-Z2 XDC (create_clock -period 8.00 on sysclk), not the 100MHz
//     every submodule's default parameter assumed. CLK_FREQ_HZ is now
//     125_000_000 at the top and explicitly passed down to every submodule
//     whose internal timing (_US constants -> cycle counts) depends on it --
//     previously only system_fsm received the override; mq2_interface,
//     flame_interface, ultrasonic_controller, and dht11_controller were all
//     silently running their internal math against their 100MHz defaults,
//     making every timing constant in the design ~20% fast (most notably,
//     the HC-SR04 10us trigger pulse and DHT11's 18ms start-low requirement
//     were both landing under spec).
//
// Every raw async digital input (gas_d0, flame_d0, hcsr04_echo, and DHT11's
// data line internally) gets its own sync_2ff.v instance before touching
// any other clocked logic (README Sec.7 note 2). Voltage-safety note: if
// your MQ-2/Flame breakout boards run their comparator at 5V, D0 needs the
// same resistor-divider treatment as the echo pin before reaching these
// ports -- see README Sec.4.
//
// PL is fully autonomous for the safety decision (README Sec.4 PS
// responsibility #5) -- the AXI4-Lite interface is tuning-only, never
// triggering, and a PS hang/reboot never stops the alert path.
// =============================================================================
module safety_sentinel_top #(
    parameter DATA_WIDTH = 16
)(
    input  wire         clk,           // 125 MHz core clock (PYNQ-Z2 sysclk, per board XDC)
    input  wire         rst,           // synchronous, active-high

    // ---------------- MQ-2 / Flame (digital D0, async) ----------------
    input  wire         gas_d0,        // MQ-2 onboard comparator output, raw async
    input  wire         flame_d0,      // Flame IR onboard comparator output, raw async

    // ---------------- HC-SR04 ultrasonic ----------------
    output wire         hcsr04_trig,
    input  wire         hcsr04_echo,   // raw async, synchronized inside ultrasonic_controller.v

    // ---------------- DHT11 ----------------
    inout  wire         dht11_dat,

    // ---------------- AXI4-Lite slave (PS <-> PL config plane) ----------------
    input  wire [9:0]   s_axi_awaddr,
    input  wire         s_axi_awvalid,
    output wire         s_axi_awready,
    input  wire [31:0]  s_axi_wdata,
    input  wire [3:0]   s_axi_wstrb,
    input  wire         s_axi_wvalid,
    output wire         s_axi_wready,
    output wire [1:0]   s_axi_bresp,
    output wire         s_axi_bvalid,
    input  wire         s_axi_bready,
    input  wire [9:0]   s_axi_araddr,
    input  wire         s_axi_arvalid,
    output wire         s_axi_arready,
    output wire [31:0]  s_axi_rdata,
    output wire [1:0]   s_axi_rresp,
    output wire         s_axi_rvalid,
    input  wire         s_axi_rready,

    // ---------------- Output Analytics System (README Sec.3, Mandatory #4) ----------------
    output wire [1:0]   alert_level,      // 00=NORMAL 01=WARNING 10=CRITICAL 11=FAULT
    output wire [3:0]   anomaly_type,
    output wire [7:0]   confidence,
    output wire [31:0]  timestamp_out,

    output wire         relay_trigger,    // cuts gas valve / ventilation / alarm circuit
    output wire [2:0]   buzzer_pattern,   // off/slow/fast/urgent/evacuate
    output wire [2:0]   led_rgb,          // green/yellow/red/flashing
    output wire         lora_tx_ready     // optional gateway sync, non-critical path
    // NOTE: uart_tx_line REMOVED. PYNQ-Z2's onboard USB-UART is PS UART0
    // (MIO), not reachable from PL. Dashboard telemetry now goes entirely
    // through the AXI4-Lite 0x200-0x214 block below; poll it from Python:
    //   status = overlay.safety_sentinel.mmio.read(0x200)
    //   alert  = status & 0x3
    //   anomaly = (status >> 8) & 0xF   ; confidence = (status >> 16) & 0xFF
);

    // =========================================================================
    // 1us tick generator (drives sensor_aggregator.v timestamping and gates
    // MQ-2/flame/ultrasonic front-end pacing where needed)
    // =========================================================================
    // FIXED: board's actual PL clock is 125MHz (PYNQ-Z2 XDC: create_clock
    // -period 8.00 on sysclk), not 100MHz. This single localparam feeds
    // every submodule instantiation below via explicit .CLK_FREQ_HZ()
    // overrides -- see change log at top of file.
    localparam CLK_FREQ_HZ = 125_000_000;
    localparam US_DIV      = CLK_FREQ_HZ / 1_000_000;
    reg [15:0] us_prescale;
    reg        us_tick;
    always @(posedge clk) begin
        if (rst) begin
            us_prescale <= 16'd0;
            us_tick     <= 1'b0;
        end else if (us_prescale >= US_DIV-1) begin
            us_prescale <= 16'd0;
            us_tick     <= 1'b1;
        end else begin
            us_prescale <= us_prescale + 16'd1;
            us_tick     <= 1'b0;
        end
    end

    // =========================================================================
    // AXI4-Lite register bank
    // =========================================================================
    wire ctrl_soft_reset, ctrl_global_enable;
    wire signed [15:0] thresh_gas, thresh_flame, thresh_ultra, thresh_temp;
    wire [2:0] win_log2_0, win_log2_1, win_log2_2, win_log2_3;
    wire weight_wr_en;
    wire [6:0] weight_wr_addr;
    wire signed [15:0] weight_wr_data;
    wire [31:0] status_reg;

    // NEW: values fed into the 0x200-0x214 dashboard readback block. Declared
    // here so they're in scope before the axi4_lite_reg instantiation; driven
    // by wires defined further down (Verilog module-scope wires are fine to
    // forward-reference across always/assign blocks within the same module).
    wire [1:0]  alert_level_fsm;
    wire [3:0]  anomaly_type_pipeline;
    wire [7:0]  confidence_pipeline;
    wire [31:0] timestamp_now;
    wire signed [DATA_WIDTH-1:0] hold_gas, hold_flame, hold_ultra, hold_temp;

    axi4_lite_reg u_axi_reg (
        .s_axi_aclk(clk), .s_axi_aresetn(~rst),
        .s_axi_awaddr(s_axi_awaddr), .s_axi_awvalid(s_axi_awvalid), .s_axi_awready(s_axi_awready),
        .s_axi_wdata(s_axi_wdata), .s_axi_wstrb(s_axi_wstrb), .s_axi_wvalid(s_axi_wvalid), .s_axi_wready(s_axi_wready),
        .s_axi_bresp(s_axi_bresp), .s_axi_bvalid(s_axi_bvalid), .s_axi_bready(s_axi_bready),
        .s_axi_araddr(s_axi_araddr), .s_axi_arvalid(s_axi_arvalid), .s_axi_arready(s_axi_arready),
        .s_axi_rdata(s_axi_rdata), .s_axi_rresp(s_axi_rresp), .s_axi_rvalid(s_axi_rvalid), .s_axi_rready(s_axi_rready),
        .ctrl_soft_reset(ctrl_soft_reset), .ctrl_global_enable(ctrl_global_enable), .status_in(status_reg),
        .thresh_gas(thresh_gas), .thresh_flame(thresh_flame), .thresh_ultra(thresh_ultra), .thresh_temp(thresh_temp),
        .win_log2_0(win_log2_0), .win_log2_1(win_log2_1), .win_log2_2(win_log2_2), .win_log2_3(win_log2_3),
        .weight_wr_en(weight_wr_en), .weight_wr_addr(weight_wr_addr), .weight_wr_data(weight_wr_data),
        // NEW: dashboard readback (README Sec.5 0x200-0x214 / Kimi review Bug #3)
        .alert_level_in(alert_level_fsm),
        .anomaly_type_in(anomaly_type_pipeline),
        .confidence_in(confidence_pipeline),
        .timestamp_in(timestamp_now),
        .ch_raw_0_in(hold_gas),
        .ch_raw_1_in(hold_flame),
        .ch_raw_2_in(hold_ultra),
        .ch_raw_3_in(hold_temp)
    );

    // =========================================================================
    // Sensor acquisition block (Mandatory Feature #1)
    // =========================================================================
    wire [DATA_WIDTH-1:0] gas_raw;    wire gas_valid;
    wire [DATA_WIDTH-1:0] flame_raw;  wire flame_valid;
    wire [DATA_WIDTH-1:0] ultra_raw;  wire ultra_valid,  ultra_timeout;
    wire [7:0]            dht_hum, dht_temp; wire dht_valid, dht_csum_fail, dht_timeout;

    wire acquire_en;

    // Synchronize both raw async D0 inputs before any other logic touches
    // them (README Sec.7 note 2) -- same requirement as hcsr04_echo, which
    // is synchronized internally inside ultrasonic_controller.v.
    wire gas_d0_sync, flame_d0_sync;
    sync_2ff #(.RESET_VAL(1'b0)) u_gas_d0_sync (
        .clk(clk), .rst(rst), .async_in(gas_d0), .sync_out(gas_d0_sync)
    );
    sync_2ff #(.RESET_VAL(1'b0)) u_flame_d0_sync (
        .clk(clk), .rst(rst), .async_in(flame_d0), .sync_out(flame_d0_sync)
    );

    // FIXED: .CLK_FREQ_HZ(CLK_FREQ_HZ) added -- previously this instantiation
    // left CLK_FREQ_HZ at mq2_interface's own 100MHz default, so its
    // SAMPLE_PERIOD_CYC math didn't match the actual 125MHz clock.
    mq2_interface #(.OUT_WIDTH(DATA_WIDTH), .CLK_FREQ_HZ(CLK_FREQ_HZ)) u_mq2 (
        .clk(clk), .rst(rst), .en(acquire_en),
        .gas_d0_sync(gas_d0_sync),
        .gas_raw(gas_raw), .gas_valid(gas_valid)
    );

    // FIXED: .CLK_FREQ_HZ(CLK_FREQ_HZ) added -- same issue as u_mq2 above.
    flame_interface #(.OUT_WIDTH(DATA_WIDTH), .CLK_FREQ_HZ(CLK_FREQ_HZ)) u_flame (
        .clk(clk), .rst(rst), .en(acquire_en),
        .flame_d0_sync(flame_d0_sync),
        .flame_raw(flame_raw), .flame_valid(flame_valid)
    );

    // FIXED: .CLK_FREQ_HZ(CLK_FREQ_HZ) added -- previously this instantiation
    // left CLK_FREQ_HZ at ultrasonic_controller's own 100MHz default, so
    // TRIG_US/TIMEOUT_US/RETRIGGER_US were all being converted to cycle
    // counts using the wrong CYCLES_PER_US (most critically, the 10us
    // trigger pulse would only have measured ~8us of real time at 125MHz
    // against a 100MHz-derived cycle count -- under HC-SR04's spec).
    ultrasonic_controller #(.DIST_WIDTH(DATA_WIDTH), .CLK_FREQ_HZ(CLK_FREQ_HZ)) u_ultra (
        .clk(clk), .rst(rst), .en(acquire_en),
        .trig(hcsr04_trig), .echo(hcsr04_echo),
        .distance_mm(ultra_raw), .distance_valid(ultra_valid), .echo_timeout_flag(ultra_timeout)
    );

    // FIXED: .CLK_FREQ_HZ(CLK_FREQ_HZ) added -- this instantiation previously
    // had NO parameter override at all, so every DHT11 protocol timing
    // (START_LOW_US, RESP_WAIT_US, BIT_TIMEOUT_US, READ_PERIOD_US) was being
    // derived from the 100MHz default. Most critically, the 20ms start-low
    // pulse would only have measured ~16ms of real time at 125MHz -- under
    // DHT11's required >=18ms minimum.
    dht11_controller #(.CLK_FREQ_HZ(CLK_FREQ_HZ)) u_dht (
        .clk(clk), .rst(rst), .en(acquire_en),
        .dht_dat(dht11_dat),
        .humidity_raw(dht_hum), .temp_raw(dht_temp),
        .dht_valid(dht_valid), .checksum_fail_flag(dht_csum_fail), .dht_timeout_flag(dht_timeout)
    );

    wire signed [DATA_WIDTH-1:0] dht_temp_ext  = {{(DATA_WIDTH-8){1'b0}}, dht_temp};
    wire signed [DATA_WIDTH-1:0] dht_hum_ext   = {{(DATA_WIDTH-8){1'b0}}, dht_hum};
    // DHT11 channel value fed downstream is temperature (used directly by
    // ultrasonic_temp_comp.v); humidity is available for future gas-drift
    // compensation (README Sec.3 "Humidity-compensated gas reading") but is
    // not yet wired into a correction path -- reserved for a follow-up pass.

    // =========================================================================
    // Fault detector
    // =========================================================================
    wire fault_flag;
    wire [3:0] fault_type;

    fault_detector u_fault (
        .clk(clk), .rst(rst),
        .echo_timeout_flag(ultra_timeout),
        .dht_checksum_fail_flag(dht_csum_fail), .dht_timeout_flag(dht_timeout),
        .fault_flag(fault_flag), .fault_type(fault_type)
    );

    // =========================================================================
    // Sensor aggregator + timestamp + CDC FIFOs (Mandatory Feature #1 cont.)
    // =========================================================================
    wire [DATA_WIDTH-1:0] agg_ch0_data, agg_ch1_data, agg_ch2_data, agg_ch3_data;
    wire agg_ch0_valid, agg_ch1_valid, agg_ch2_valid, agg_ch3_valid;

    sensor_aggregator #(.DATA_WIDTH(DATA_WIDTH)) u_aggregator (
        .clk(clk), .rst(rst), .us_tick(us_tick),
        .gas_raw(gas_raw), .gas_valid(gas_valid),
        .flame_raw(flame_raw), .flame_valid(flame_valid),
        .ultra_raw(ultra_raw), .ultra_valid(ultra_valid),
        .dht_raw(dht_temp_ext), .dht_valid(dht_valid),
        .ch0_data(agg_ch0_data), .ch0_valid(agg_ch0_valid),
        .ch1_data(agg_ch1_data), .ch1_valid(agg_ch1_valid),
        .ch2_data(agg_ch2_data), .ch2_valid(agg_ch2_valid),
        .ch3_data(agg_ch3_data), .ch3_valid(agg_ch3_valid),
        .timestamp_now(timestamp_now)
    );

    // =========================================================================
    // 4x independent true moving average filters (Mandatory Feature #2)
    // =========================================================================
    wire [DATA_WIDTH-1:0] ma_gas, ma_flame, ma_ultra, ma_temp;
    wire ma_gas_valid, ma_flame_valid, ma_ultra_valid, ma_temp_valid;

    true_moving_avg #(.DATA_WIDTH(DATA_WIDTH), .MAX_WIN_LOG2(6)) u_ma_gas (
        .clk(clk), .rst(rst), .en(1'b1), .cfg_win_log2(win_log2_0),
        .data_in(agg_ch0_data), .data_valid_in(agg_ch0_valid),
        .data_out(ma_gas), .data_valid_out(ma_gas_valid)
    );
    true_moving_avg #(.DATA_WIDTH(DATA_WIDTH), .MAX_WIN_LOG2(6)) u_ma_flame (
        .clk(clk), .rst(rst), .en(1'b1), .cfg_win_log2(win_log2_1),
        .data_in(agg_ch1_data), .data_valid_in(agg_ch1_valid),
        .data_out(ma_flame), .data_valid_out(ma_flame_valid)
    );
    true_moving_avg #(.DATA_WIDTH(DATA_WIDTH), .MAX_WIN_LOG2(6)) u_ma_ultra (
        .clk(clk), .rst(rst), .en(1'b1), .cfg_win_log2(win_log2_2),
        .data_in(agg_ch2_data), .data_valid_in(agg_ch2_valid),
        .data_out(ma_ultra), .data_valid_out(ma_ultra_valid)
    );
    true_moving_avg #(.DATA_WIDTH(DATA_WIDTH), .MAX_WIN_LOG2(6)) u_ma_temp (
        .clk(clk), .rst(rst), .en(1'b1), .cfg_win_log2(win_log2_3),
        .data_in(agg_ch3_data), .data_valid_in(agg_ch3_valid),
        .data_out(ma_temp), .data_valid_out(ma_temp_valid)
    );

    // =========================================================================
    // Ultrasonic temperature compensation (uses filtered DHT11 temp)
    // =========================================================================
    wire signed [DATA_WIDTH-1:0] ultra_comp;
    wire ultra_comp_valid;

    ultrasonic_temp_comp #(.DIST_WIDTH(DATA_WIDTH)) u_ultra_comp (
        .clk(clk), .rst(rst),
        .distance_raw_mm(ma_ultra), .distance_valid(ma_ultra_valid),
        .temp_c(ma_temp),
        .distance_comp_mm(ultra_comp), .distance_comp_valid(ultra_comp_valid)
    );

    // =========================================================================
    // Sample-rate reconciliation: zero-order hold with mandatory all_seen guard
    // =========================================================================
    wire vector_valid, all_seen;

    sample_hold_sync #(.DATA_WIDTH(DATA_WIDTH)) u_hold (
        .clk(clk), .rst(rst),
        .ch0_data(ma_gas),   .ch0_valid(ma_gas_valid),
        .ch1_data(ma_flame), .ch1_valid(ma_flame_valid),
        .ch2_data(ultra_comp), .ch2_valid(ultra_comp_valid),
        .ch3_data(ma_temp),  .ch3_valid(ma_temp_valid),
        .hold_gas(hold_gas), .hold_flame(hold_flame), .hold_ultra(hold_ultra), .hold_temp(hold_temp),
        .vector_valid(vector_valid), .all_seen(all_seen)
    );

    // =========================================================================
    // System FSM: INIT -> CALIBRATE -> ACQUIRE -> PROCESS -> ALERT/FAULT
    // =========================================================================
    wire [2:0] fsm_state;
    wire       process_en;
    wire [1:0] alert_level_pipeline;

    system_fsm #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .WARMUP_TIME_MS(30_000)) u_fsm (
        .clk(clk), .rst(rst),
        .global_enable(ctrl_global_enable), .soft_reset(ctrl_soft_reset),
        .fault_flag(fault_flag), .all_seen(all_seen),
        .alert_level_in(alert_level_pipeline),
        .fsm_state(fsm_state), .acquire_en(acquire_en), .process_en(process_en),
        .alert_level_out(alert_level_fsm)
    );

    // =========================================================================
    // Decision pipeline: feature extraction -> thresholds -> Tiny MLP ->
    // fusion -> hybrid vote (modules 9-14, wrapped)
    // =========================================================================
    decision_pipeline #(.DATA_WIDTH(DATA_WIDTH)) u_decision (
        .clk(clk), .rst(rst),
        .vector_valid(vector_valid && process_en),
        .hold_gas(hold_gas), .hold_flame(hold_flame), .hold_ultra_comp(hold_ultra), .hold_temp(hold_temp),
        .fault_flag(fault_flag),
        .thresh_gas(thresh_gas), .thresh_flame(thresh_flame), .thresh_ultra(thresh_ultra), .thresh_temp(thresh_temp),
        .weight_wr_en(weight_wr_en), .weight_wr_addr(weight_wr_addr), .weight_wr_data(weight_wr_data),
        .alert_level(alert_level_pipeline), .anomaly_type(anomaly_type_pipeline), .confidence(confidence_pipeline)
    );

    // =========================================================================
    // Output Analytics System (Mandatory Feature #4)
    // =========================================================================
    assign alert_level    = alert_level_fsm;
    assign anomaly_type   = anomaly_type_pipeline;
    assign confidence     = confidence_pipeline;
    assign timestamp_out  = timestamp_now;

    assign status_reg = {21'd0, fault_type, fault_flag, alert_level_fsm, fsm_state};

    // relay: cut gas valve / ventilation / alarm circuit on CRITICAL or FAULT
    assign relay_trigger = (alert_level_fsm == 2'b10) || (alert_level_fsm == 2'b11);

    // buzzer_pattern: off/slow/fast/urgent/evacuate mapped from alert level
    reg [2:0] buzzer_pattern_r;
    always @(*) begin
        case (alert_level_fsm)
            2'b00: buzzer_pattern_r = 3'd0; // off
            2'b01: buzzer_pattern_r = 3'd1; // slow
            2'b10: buzzer_pattern_r = 3'd3; // urgent
            2'b11: buzzer_pattern_r = 3'd4; // evacuate (fault -- treat as worst case)
            default: buzzer_pattern_r = 3'd0;
        endcase
    end
    assign buzzer_pattern = buzzer_pattern_r;

    // =========================================================================
    // NEW: ~2 Hz blink generator (Kimi review Bug #8). Previously led_rgb's
    // FAULT encoding (3'b101) set a "flash" bit with no toggle source behind
    // it anywhere in the design -- just a static pattern. This free-running
    // counter toggles blink_2hz every ~250ms (2 Hz square wave), and that
    // toggle is ANDed into the flash bit only while FAULT is active, so the
    // LED genuinely blinks instead of just showing a constant red+bit.
    // =========================================================================
    localparam BLINK_HALF_PERIOD_CYC = CLK_FREQ_HZ / 4; // toggle every 250ms -> 2Hz square wave
    reg [$clog2(BLINK_HALF_PERIOD_CYC)-1:0] blink_cnt;
    reg blink_2hz;
    always @(posedge clk) begin
        if (rst) begin
            blink_cnt <= 0;
            blink_2hz <= 1'b0;
        end else if (blink_cnt >= BLINK_HALF_PERIOD_CYC - 1) begin
            blink_cnt <= 0;
            blink_2hz <= ~blink_2hz;
        end else begin
            blink_cnt <= blink_cnt + 1'b1;
        end
    end

    // led_rgb: green/yellow/red/flashing
    reg [2:0] led_rgb_r;
    always @(*) begin
        case (alert_level_fsm)
            2'b00: led_rgb_r = 3'b001; // green
            2'b01: led_rgb_r = 3'b010; // yellow
            2'b10: led_rgb_r = 3'b100; // red
            2'b11: led_rgb_r = {2'b10, blink_2hz}; // red, flash bit now actually toggles at 2Hz
            default: led_rgb_r = 3'b000;
        endcase
    end
    assign led_rgb = led_rgb_r;

    // lora_tx_ready: optional gateway sync, non-critical path -- simply mirrors
    // "a new telemetry frame is ready to be relayed," never gates the alert path
    assign lora_tx_ready = vector_valid;

    // NOTE: UART removed here per README Sec.4 (dashboard = PYNQ Jupyter
    // notebook polling AXI4-Lite, not a PL-driven serial stream) and per the
    // PS-UART0-vs-PL-fabric discussion -- see change log at top of file.

endmodule