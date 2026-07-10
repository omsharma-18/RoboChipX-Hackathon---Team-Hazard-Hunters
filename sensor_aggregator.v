// =============================================================================
// sensor_aggregator.v
// -----------------------------------------------------------------------------
// Stamps every incoming sample (gas, flame, ultrasonic, dht11) with a 32-bit
// free-running microsecond timestamp and a 2-bit channel-ID tag, then pushes
// it through a per-channel async FIFO into the 100 MHz core clock domain.
// Sensor front-ends are wired to core_clk in this integration, but the FIFO
// boundary is preserved so any future front-end that moves to its own native
// clock domain (e.g. an independently-clocked ultrasonic timer) is already
// safe to cross without touching downstream logic.
// =============================================================================// =============================================================================
// sensor_aggregator.v  (unchanged -- no functional bug raised. Kimi review
// Minor #10 notes the async_fifo instances use the same clock on both
// wr/rd sides, which is technically wasteful (a real CDC FIFO isn't needed
// since everything's already in the 100MHz domain) -- documented here as a
// resource-usage note, not a correctness fix, since the module header
// already explains this is deliberate: the FIFO boundary is kept in place
// so a future front-end that moves to its own clock domain doesn't require
// touching downstream logic. No code change made.)
// -----------------------------------------------------------------------------
// Stamps every incoming sample (gas, flame, ultrasonic, dht11) with a 32-bit
// free-running microsecond timestamp and a 2-bit channel-ID tag, then pushes
// it through a per-channel FIFO into the 100 MHz core clock domain.
// =============================================================================
module sensor_aggregator #(
    parameter DATA_WIDTH = 16
)(
    input  wire                     clk,          // 100 MHz core clock (rd + wr side here)
    input  wire                     rst,

    input  wire                     us_tick,

    input  wire [DATA_WIDTH-1:0]    gas_raw,
    input  wire                     gas_valid,
    input  wire [DATA_WIDTH-1:0]    flame_raw,
    input  wire                     flame_valid,
    input  wire [DATA_WIDTH-1:0]    ultra_raw,
    input  wire                     ultra_valid,
    input  wire [DATA_WIDTH-1:0]    dht_raw,
    input  wire                     dht_valid,

    output wire [DATA_WIDTH-1:0]    ch0_data, output wire ch0_valid,
    output wire [DATA_WIDTH-1:0]    ch1_data, output wire ch1_valid,
    output wire [DATA_WIDTH-1:0]    ch2_data, output wire ch2_valid,
    output wire [DATA_WIDTH-1:0]    ch3_data, output wire ch3_valid,

    output reg  [31:0]              timestamp_now   // free-running us counter, exported
);

    always @(posedge clk) begin
        if (rst) timestamp_now <= 32'd0;
        else if (us_tick) timestamp_now <= timestamp_now + 32'd1;
    end

    localparam FIFO_W = DATA_WIDTH + 32 + 2; // {timestamp, chan_id, data}
    localparam FIFO_AW = 4;

    genvar g;
    generate
        for (g = 0; g < 4; g = g + 1) begin : CH_FIFO
            wire [DATA_WIDTH-1:0] raw_data;
            wire                  raw_valid;
            wire [FIFO_W-1:0]     wr_pkt;
            wire                  fifo_empty;
            wire [FIFO_W-1:0]     rd_pkt;

            assign raw_data  = (g==0) ? gas_raw   :
                                (g==1) ? flame_raw :
                                (g==2) ? ultra_raw : dht_raw;
            assign raw_valid = (g==0) ? gas_valid  :
                                (g==1) ? flame_valid:
                                (g==2) ? ultra_valid: dht_valid;

            assign wr_pkt = {timestamp_now, g[1:0], raw_data};

            async_fifo #(.DATA_WIDTH(FIFO_W), .ADDR_WIDTH(FIFO_AW)) u_fifo (
                .wr_clk  (clk),
                .wr_rst  (rst),
                .wr_en   (raw_valid),
                .wr_data (wr_pkt),
                .wr_full (),
                .rd_clk  (clk),
                .rd_rst  (rst),
                .rd_en   (!fifo_empty),
                .rd_data (rd_pkt),
                .rd_empty(fifo_empty)
            );

            reg rd_valid_d;
            always @(posedge clk) begin
                if (rst) rd_valid_d <= 1'b0;
                else     rd_valid_d <= !fifo_empty;
            end

            if (g==0) begin assign ch0_data = rd_pkt[DATA_WIDTH-1:0]; assign ch0_valid = rd_valid_d; end
            if (g==1) begin assign ch1_data = rd_pkt[DATA_WIDTH-1:0]; assign ch1_valid = rd_valid_d; end
            if (g==2) begin assign ch2_data = rd_pkt[DATA_WIDTH-1:0]; assign ch2_valid = rd_valid_d; end
            if (g==3) begin assign ch3_data = rd_pkt[DATA_WIDTH-1:0]; assign ch3_valid = rd_valid_d; end
        end
    endgenerate

endmodule