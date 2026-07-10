// =============================================================================
// dht11_controller.v
// -----------------------------------------------------------------------------
// Bit-bangs the DHT11 single-wire protocol:
//   1. Host pulls line low >=18ms (start signal)
//   2. Host releases, pulls up via external resistor
//   3. DHT11 responds: 80us low + 80us high
//   4. DHT11 clocks out 40 bits: humidity_int, humidity_dec, temp_int,
//      temp_dec, checksum (each bit = 50us low + 26-28us(=0) or 70us(=1) high)
//   5. 8-bit checksum = sum of first 4 bytes, mod 256, verified against byte 5
//
// This is the highest-timing-risk block in the design (README Sec.8) --
// all timing constants are expressed in microseconds and converted using
// the core clock frequency, not hand-tuned cycle counts.
// =============================================================================// =============================================================================
// dht11_controller.v  (unchanged -- already contains the S_BIT_WAIT_L
// timeout-increment bugfix noted in its own header comment; no further
// issues raised by the Kimi review beyond Minor #11, which is a style note
// -- see below)
// -----------------------------------------------------------------------------
// Bit-bangs the DHT11 single-wire protocol. Highest-timing-risk block in the
// design (README Sec.8) -- all timing constants expressed in microseconds.
//
// NOTE on Kimi review Minor #11 ("dht11_controller implements its own 2-flop
// sync instead of using sync_2ff.v"): this is a documentation/consistency
// item, not a functional bug -- the inline dht_in_meta/dht_in pair below is
// functionally identical to sync_2ff.v (same 2-flop topology, same reset
// behavior). Left as-is because dht_dat is a shared tri-state inout pin
// (pull_low driver + synchronized read share the same wire), which doesn't
// map cleanly onto sync_2ff's plain single-direction async_in port without
// restructuring the tri-state driver logic too. Flagging here so the
// inconsistency is documented rather than silently present.
// =============================================================================
module dht11_controller #(
    parameter CLK_FREQ_HZ    = 100_000_000,
    parameter START_LOW_US   = 20_000,   // >=18ms host start pulse (host low)
    parameter RESP_WAIT_US   = 40,       // host release -> wait for DHT response
    parameter BIT_TIMEOUT_US = 200,      // generous per-phase timeout
    parameter READ_PERIOD_US = 1_500_000 // >=1s between reads (README: 1s min)
)(
    input  wire         clk,
    input  wire         rst,
    input  wire         en,

    inout  wire         dht_dat,          // single-wire bidirectional pin

    output reg  [7:0]   humidity_raw,     // integer part, %RH
    output reg  [7:0]   temp_raw,         // integer part, deg C
    output reg           dht_valid,
    output reg           checksum_fail_flag,
    output reg           dht_timeout_flag
);

    localparam CYCLES_PER_US = CLK_FREQ_HZ / 1_000_000;

    reg  pull_low;
    assign dht_dat = pull_low ? 1'b0 : 1'bz;

    reg dht_in_meta, dht_in;
    always @(posedge clk) begin
        dht_in_meta <= dht_dat;
        dht_in      <= dht_in_meta;
    end

    localparam [3:0]
        S_IDLE        = 4'd0,
        S_START_LOW   = 4'd1,
        S_RELEASE     = 4'd2,
        S_WAIT_RESP_L = 4'd3,
        S_WAIT_RESP_H = 4'd4,
        S_BIT_WAIT_L  = 4'd5,
        S_BIT_WAIT_H  = 4'd6,
        S_BIT_SAMPLE  = 4'd7,
        S_CHECKSUM    = 4'd8,
        S_DONE        = 4'd9,
        S_TIMEOUT     = 4'd10,
        S_INTERVAL    = 4'd11;

    reg [3:0]  state;
    reg [31:0] cnt;
    reg [31:0] bit_high_cnt;
    reg [5:0]  bit_idx;          // 0..39
    reg [39:0] shift_reg;        // 5 bytes: hum_int, hum_dec, temp_int, temp_dec, csum

    wire [31:0] START_LOW_CYC   = START_LOW_US   * CYCLES_PER_US;
    wire [31:0] RESP_WAIT_CYC   = RESP_WAIT_US   * CYCLES_PER_US;
    wire [31:0] BIT_TIMEOUT_CYC = BIT_TIMEOUT_US * CYCLES_PER_US;
    wire [31:0] READ_PERIOD_CYC = READ_PERIOD_US * CYCLES_PER_US;

    wire [31:0] BIT_THRESH_CYC  = 32'd50 * CYCLES_PER_US;

    wire [7:0] checksum_calc = shift_reg[39:32] + shift_reg[31:24] + shift_reg[23:16] + shift_reg[15:8];

    always @(posedge clk) begin
        if (rst) begin
            state              <= S_IDLE;
            pull_low            <= 1'b0;
            cnt                 <= 0;
            bit_high_cnt        <= 0;
            bit_idx             <= 0;
            shift_reg           <= 40'd0;
            humidity_raw        <= 8'd0;
            temp_raw            <= 8'd0;
            dht_valid           <= 1'b0;
            checksum_fail_flag  <= 1'b0;
            dht_timeout_flag    <= 1'b0;
        end else begin
            dht_valid <= 1'b0;

            case (state)
                S_IDLE: begin
                    pull_low <= 1'b0;
                    cnt      <= 0;
                    if (en) state <= S_START_LOW;
                end

                S_START_LOW: begin
                    pull_low <= 1'b1;
                    if (cnt >= START_LOW_CYC) begin
                        cnt   <= 0;
                        state <= S_RELEASE;
                    end else cnt <= cnt + 1'b1;
                end

                S_RELEASE: begin
                    pull_low <= 1'b0;
                    if (cnt >= RESP_WAIT_CYC) begin
                        cnt   <= 0;
                        state <= S_WAIT_RESP_L;
                    end else cnt <= cnt + 1'b1;
                end

                S_WAIT_RESP_L: begin
                    if (!dht_in) begin
                        cnt   <= 0;
                        state <= S_WAIT_RESP_H;
                    end else if (cnt >= BIT_TIMEOUT_CYC) begin
                        state <= S_TIMEOUT;
                    end else cnt <= cnt + 1'b1;
                end

                S_WAIT_RESP_H: begin
                    if (dht_in) begin
                        cnt   <= 0;
                        state <= S_BIT_WAIT_L;
                    end else if (cnt >= BIT_TIMEOUT_CYC) begin
                        state <= S_TIMEOUT;
                    end else cnt <= cnt + 1'b1;
                end

                S_BIT_WAIT_L: begin
                    if (!dht_in) begin
                        cnt   <= 0;
                        state <= S_BIT_WAIT_H;
                    end else if (cnt >= BIT_TIMEOUT_CYC) begin
                        state <= S_TIMEOUT;
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                S_BIT_WAIT_H: begin
                    if (dht_in) begin
                        cnt          <= 0;
                        bit_high_cnt <= 0;
                        state        <= S_BIT_SAMPLE;
                    end else if (cnt >= BIT_TIMEOUT_CYC) begin
                        state <= S_TIMEOUT;
                    end else cnt <= cnt + 1'b1;
                end

                S_BIT_SAMPLE: begin
                    if (dht_in) begin
                        bit_high_cnt <= bit_high_cnt + 1'b1;
                        if (bit_high_cnt >= BIT_TIMEOUT_CYC) state <= S_TIMEOUT;
                    end else begin
                        shift_reg <= {shift_reg[38:0], (bit_high_cnt > BIT_THRESH_CYC)};
                        if (bit_idx == 6'd39) begin
                            bit_idx <= 0;
                            state   <= S_CHECKSUM;
                        end else begin
                            bit_idx <= bit_idx + 1'b1;
                            state   <= S_BIT_WAIT_L;
                        end
                    end
                end

                S_CHECKSUM: begin
                    if (checksum_calc == shift_reg[7:0]) begin
                        humidity_raw       <= shift_reg[39:32];
                        temp_raw           <= shift_reg[23:16];
                        dht_valid          <= 1'b1;
                        checksum_fail_flag <= 1'b0;
                    end else begin
                        checksum_fail_flag <= 1'b1;
                    end
                    dht_timeout_flag <= 1'b0;
                    cnt   <= 0;
                    state <= S_INTERVAL;
                end

                S_TIMEOUT: begin
                    dht_timeout_flag <= 1'b1;
                    pull_low         <= 1'b0;
                    cnt              <= 0;
                    state            <= S_INTERVAL;
                end

                S_INTERVAL: begin
                    if (cnt >= READ_PERIOD_CYC) begin
                        cnt   <= 0;
                        state <= S_IDLE;
                    end else cnt <= cnt + 1'b1;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule