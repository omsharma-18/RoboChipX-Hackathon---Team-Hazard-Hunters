// =============================================================================
// axi4_lite_reg.v
// -----------------------------------------------------------------------------
// Standard AXI4-Lite slave implementing the register map from README Sec.5:
//   0x00 CONTROL       R/W  soft reset, global enable
//   0x04 STATUS        R    FSM state, alert level, fault flags
//   0x08 THRESH_GAS    R/W
//   0x0C THRESH_FLAME  R/W
//   0x10 THRESH_ULTRA  R/W
//   0x14 THRESH_TEMP   R/W
//   0x18 WIN_LOG2_0    R/W  MAF window config, CH0 (gas)
//   0x1C WIN_LOG2_1    R/W  MAF window config, CH1 (flame)
//   0x20 WIN_LOG2_2    R/W  MAF window config, CH2 (ultrasonic)
//   0x24 WIN_LOG2_3    R/W  MAF window config, CH3 (DHT11)
//   0x28-0xFC MLP_WEIGHT_[n]  R/W streamed weight load, Q1.15
//   0x100-0x1FF MLP_WEIGHTS   R/W full 72-weight block (padded region)
//   0x200 ALERT_LEVEL  R    {anomaly_type[3:0], confidence[7:0], 6'b0, alert_level[1:0]}
//   0x204 TIMESTAMP    R    32-bit us-since-boot counter
//   0x208 CH_RAW_0     R    latest filtered gas value
//   0x20C CH_RAW_1     R    latest filtered flame value
//   0x210 CH_RAW_2     R    latest filtered/compensated ultrasonic distance
//   0x214 CH_RAW_3     R    latest filtered temperature
//
// FIX (Kimi review Critical Bug #3): the 0x200-0x214 dashboard readback block
// from README Sec.5 was previously missing entirely -- the read case defaulted
// to 32'd0 for every address past ADDR_WIN3, so the PYNQ Jupyter dashboard
// would read zeros for alert level, timestamp, and all four channel values.
// These are now real input ports, latched combinationally into the read mux.
//
// Contract (README Sec.5): every write is reflected in PL logic within one
// handshake cycle; all registers are tuning, never triggering -- a safety
// decision is never gated on a PS write completing.
// =============================================================================
module axi4_lite_reg #(
    parameter ADDR_WIDTH = 10,
    parameter DATA_WIDTH = 32
)(
    input  wire                          s_axi_aclk,
    input  wire                          s_axi_aresetn,

    // write address channel
    input  wire [ADDR_WIDTH-1:0]         s_axi_awaddr,
    input  wire                          s_axi_awvalid,
    output reg                           s_axi_awready,
    // write data channel
    input  wire [DATA_WIDTH-1:0]         s_axi_wdata,
    input  wire [(DATA_WIDTH/8)-1:0]     s_axi_wstrb,
    input  wire                          s_axi_wvalid,
    output reg                           s_axi_wready,
    // write response channel
    output reg  [1:0]                    s_axi_bresp,
    output reg                           s_axi_bvalid,
    input  wire                          s_axi_bready,
    // read address channel
    input  wire [ADDR_WIDTH-1:0]         s_axi_araddr,
    input  wire                          s_axi_arvalid,
    output reg                           s_axi_arready,
    // read data channel
    output reg  [DATA_WIDTH-1:0]         s_axi_rdata,
    output reg  [1:0]                    s_axi_rresp,
    output reg                           s_axi_rvalid,
    input  wire                          s_axi_rready,

    // ---------------- register-mapped connections to the rest of the design ----------------
    output reg                           ctrl_soft_reset,
    output reg                           ctrl_global_enable,
    input  wire [31:0]                   status_in,          // {fsm_state, alert_level, fault_flag, fault_type,...}

    output reg  signed [15:0]            thresh_gas,
    output reg  signed [15:0]            thresh_flame,
    output reg  signed [15:0]            thresh_ultra,
    output reg  signed [15:0]            thresh_temp,

    output reg  [2:0]                    win_log2_0,
    output reg  [2:0]                    win_log2_1,
    output reg  [2:0]                    win_log2_2,
    output reg  [2:0]                    win_log2_3,

    output reg                           weight_wr_en,
    output reg  [6:0]                    weight_wr_addr,
    output reg  signed [15:0]            weight_wr_data,

    // ---------------- NEW: dashboard readback inputs (README Sec.5, 0x200-0x214) ----------------
    input  wire [1:0]                    alert_level_in,
    input  wire [3:0]                    anomaly_type_in,
    input  wire [7:0]                    confidence_in,
    input  wire [31:0]                   timestamp_in,
    input  wire signed [15:0]            ch_raw_0_in,   // gas
    input  wire signed [15:0]            ch_raw_1_in,   // flame
    input  wire signed [15:0]            ch_raw_2_in,   // ultrasonic (temp-compensated)
    input  wire signed [15:0]            ch_raw_3_in    // temperature
);

    localparam ADDR_CONTROL      = 10'h000;
    localparam ADDR_STATUS       = 10'h004;
    localparam ADDR_THRESH_GAS   = 10'h008;
    localparam ADDR_THRESH_FLAME = 10'h00C;
    localparam ADDR_THRESH_ULTRA = 10'h010;
    localparam ADDR_THRESH_TEMP  = 10'h014;
    localparam ADDR_WIN0         = 10'h018;
    localparam ADDR_WIN1         = 10'h01C;
    localparam ADDR_WIN2         = 10'h020;
    localparam ADDR_WIN3         = 10'h024;
    localparam ADDR_WEIGHT_LO    = 10'h028;
    localparam ADDR_WEIGHT_HI    = 10'h0FC; // 0x28..0xFC -> 56 words -> but we only need 72, wrap into 0x100 block too
    localparam ADDR_WEIGHTBLK_LO = 10'h100;
    localparam ADDR_WEIGHTBLK_HI = 10'h1FF;

    // NEW: dashboard readback block (README Sec.5)
    localparam ADDR_ALERT_LEVEL  = 10'h200;
    localparam ADDR_TIMESTAMP    = 10'h204;
    localparam ADDR_CH_RAW_0     = 10'h208;
    localparam ADDR_CH_RAW_1     = 10'h20C;
    localparam ADDR_CH_RAW_2     = 10'h210;
    localparam ADDR_CH_RAW_3     = 10'h214;

    reg [ADDR_WIDTH-1:0] awaddr_latched;
    reg [ADDR_WIDTH-1:0] araddr_latched;

    // ---------------- write channel ----------------
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
        end else begin
            if (!s_axi_awready && s_axi_awvalid && s_axi_wvalid) begin
                s_axi_awready <= 1'b1;
                s_axi_wready  <= 1'b1;
                awaddr_latched <= s_axi_awaddr;
            end else begin
                s_axi_awready <= 1'b0;
                s_axi_wready  <= 1'b0;
            end
        end
    end

    wire do_write = s_axi_awready && s_axi_wready;

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            ctrl_soft_reset    <= 1'b0;
            ctrl_global_enable <= 1'b0;
            thresh_gas         <= 16'sd0;
            thresh_flame       <= 16'sd0;
            thresh_ultra       <= 16'sd0;
            thresh_temp        <= 16'sd0;
            win_log2_0         <= 3'd6; // default: gas window=64 (2^6)
            win_log2_1         <= 3'd4; // default: flame window=16 (2^4)
            win_log2_2         <= 3'd3; // default: ultra window=8 (2^3)
            win_log2_3         <= 3'd1; // default: dht11 window=2 (2^1)
            weight_wr_en       <= 1'b0;
            weight_wr_addr     <= 7'd0;
            weight_wr_data     <= 16'sd0;
            s_axi_bvalid       <= 1'b0;
            s_axi_bresp        <= 2'b00;
        end else begin
            weight_wr_en <= 1'b0;

            if (do_write) begin
                case (awaddr_latched)
                    ADDR_CONTROL: begin
                        ctrl_soft_reset    <= s_axi_wdata[0];
                        ctrl_global_enable <= s_axi_wdata[1];
                    end
                    ADDR_THRESH_GAS:   thresh_gas   <= s_axi_wdata[15:0];
                    ADDR_THRESH_FLAME: thresh_flame <= s_axi_wdata[15:0];
                    ADDR_THRESH_ULTRA: thresh_ultra <= s_axi_wdata[15:0];
                    ADDR_THRESH_TEMP:  thresh_temp  <= s_axi_wdata[15:0];
                    ADDR_WIN0: win_log2_0 <= s_axi_wdata[2:0];
                    ADDR_WIN1: win_log2_1 <= s_axi_wdata[2:0];
                    ADDR_WIN2: win_log2_2 <= s_axi_wdata[2:0];
                    ADDR_WIN3: win_log2_3 <= s_axi_wdata[2:0];
                    default: begin
                        if (awaddr_latched >= ADDR_WEIGHTBLK_LO && awaddr_latched <= ADDR_WEIGHTBLK_HI) begin
                            // full 72-weight block, word-addressed
                            weight_wr_en   <= 1'b1;
                            weight_wr_addr <= (awaddr_latched - ADDR_WEIGHTBLK_LO) >> 2;
                            weight_wr_data <= s_axi_wdata[15:0];
                        end else if (awaddr_latched >= ADDR_WEIGHT_LO && awaddr_latched <= ADDR_WEIGHT_HI) begin
                            // streamed weight-load window
                            weight_wr_en   <= 1'b1;
                            weight_wr_addr <= (awaddr_latched - ADDR_WEIGHT_LO) >> 2;
                            weight_wr_data <= s_axi_wdata[15:0];
                        end
                        // ADDR_ALERT_LEVEL / TIMESTAMP / CH_RAW_* are read-only:
                        // writes to them are accepted (AXI still returns OKAY,
                        // per the "every write gets a response" rule) but silently
                        // dropped -- no register in this block latches them.
                    end
                endcase

                s_axi_bvalid <= 1'b1;
                s_axi_bresp  <= 2'b00; // OKAY
            end else if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end
        end
    end

    // ---------------- read channel ----------------
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_arready <= 1'b0;
            araddr_latched <= {ADDR_WIDTH{1'b0}};
        end else begin
            if (!s_axi_arready && s_axi_arvalid) begin
                s_axi_arready  <= 1'b1;
                araddr_latched <= s_axi_araddr;
            end else begin
                s_axi_arready <= 1'b0;
            end
        end
    end

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_rvalid <= 1'b0;
            s_axi_rresp  <= 2'b00;
            s_axi_rdata  <= 32'd0;
        end else begin
            if (s_axi_arready && s_axi_arvalid && !s_axi_rvalid) begin
                s_axi_rvalid <= 1'b1;
                s_axi_rresp  <= 2'b00;
                case (araddr_latched)
                    ADDR_CONTROL:      s_axi_rdata <= {30'd0, ctrl_global_enable, ctrl_soft_reset};
                    ADDR_STATUS:       s_axi_rdata <= status_in;
                    ADDR_THRESH_GAS:   s_axi_rdata <= {{16{thresh_gas[15]}},   thresh_gas};
                    ADDR_THRESH_FLAME: s_axi_rdata <= {{16{thresh_flame[15]}}, thresh_flame};
                    ADDR_THRESH_ULTRA: s_axi_rdata <= {{16{thresh_ultra[15]}}, thresh_ultra};
                    ADDR_THRESH_TEMP:  s_axi_rdata <= {{16{thresh_temp[15]}},  thresh_temp};
                    ADDR_WIN0: s_axi_rdata <= {29'd0, win_log2_0};
                    ADDR_WIN1: s_axi_rdata <= {29'd0, win_log2_1};
                    ADDR_WIN2: s_axi_rdata <= {29'd0, win_log2_2};
                    ADDR_WIN3: s_axi_rdata <= {29'd0, win_log2_3};
                    // NEW: dashboard readback block (README Sec.5, fixes Kimi
                    // review Critical Bug #3 -- these previously fell through
                    // to `default: 32'd0`).
                    ADDR_ALERT_LEVEL: s_axi_rdata <= {16'd0, anomaly_type_in, confidence_in, 6'd0, alert_level_in};
                    ADDR_TIMESTAMP:   s_axi_rdata <= timestamp_in;
                    ADDR_CH_RAW_0:    s_axi_rdata <= {{16{ch_raw_0_in[15]}}, ch_raw_0_in};
                    ADDR_CH_RAW_1:    s_axi_rdata <= {{16{ch_raw_1_in[15]}}, ch_raw_1_in};
                    ADDR_CH_RAW_2:    s_axi_rdata <= {{16{ch_raw_2_in[15]}}, ch_raw_2_in};
                    ADDR_CH_RAW_3:    s_axi_rdata <= {{16{ch_raw_3_in[15]}}, ch_raw_3_in};
                    default:   s_axi_rdata <= 32'd0; // MLP weight readback intentionally not wired here
                endcase
            end else if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end

endmodule