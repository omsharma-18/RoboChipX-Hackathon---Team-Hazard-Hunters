// =============================================================================
// true_moving_avg.v
// -----------------------------------------------------------------------------
// TRUE windowed moving average (FIR), NOT an exponential moving average (EMA).
// Implements a circular buffer with running-sum update:
//     running_sum <= running_sum - buffer[write_ptr] + data_in
// Division is power-of-2 only (arithmetic right-shift by cfg_win_log2).
// Window is runtime-configurable via cfg_win_log2 (AXI4-Lite driven upstream).
//
// One instance per sensor channel. Instantiated 4x at top level with different
// default windows (gas=64, flame=16, ultrasonic=8, dht11=2) per README Sec.3.
// =============================================================================
// =============================================================================
// true_moving_avg.v  (unchanged -- Kimi review Minor #13 flags a possible
// window-reconfig race with data_valid_in as a documentation item, not a
// functional bug requiring a code change: the reconfig branch already takes
// priority over the data_valid_in branch in the same always block, i.e. if
// cfg_win_log2 changes on the same cycle a sample arrives, the flush wins
// and that sample is dropped rather than being written into a half-flushed
// buffer. That is the safe behavior. Documented here per Minor #13.)
// -----------------------------------------------------------------------------
// TRUE windowed moving average (FIR), NOT an exponential moving average (EMA).
// Circular buffer + running-sum update; division is power-of-2 only
// (arithmetic right-shift). Window is runtime-configurable via cfg_win_log2.
// =============================================================================
module true_moving_avg #(
    parameter DATA_WIDTH   = 16,
    parameter MAX_WIN_LOG2 = 6            // max window = 2^6 = 64 (gas channel)
)(
    input  wire                          clk,
    input  wire                          rst,
    input  wire                          en,
    input  wire [2:0]                    cfg_win_log2,   // 0..6 => window 1..64
    input  wire [DATA_WIDTH-1:0]         data_in,
    input  wire                          data_valid_in,
    output reg  [DATA_WIDTH-1:0]         data_out,
    output reg                           data_valid_out
);
    localparam MAX_WIN = (1 << MAX_WIN_LOG2);
    localparam SUM_WIDTH = DATA_WIDTH + MAX_WIN_LOG2;
    reg [DATA_WIDTH-1:0]   buffer [0:MAX_WIN-1];
    reg [MAX_WIN_LOG2-1:0] wr_ptr;
    reg [SUM_WIDTH-1:0]    running_sum;
    reg [MAX_WIN_LOG2:0]   fill_count;      // extra bit: counts up to MAX_WIN
    reg [2:0]              win_log2_reg;
    wire [MAX_WIN_LOG2-1:0] window_size_m1 = (1 << cfg_win_log2) - 1'b1; // window-1
    wire [SUM_WIDTH-1:0]    new_sum        = running_sum - buffer[wr_ptr] + data_in;
    integer i;
    always @(posedge clk) begin
        if (rst) begin
            wr_ptr         <= 0;
            running_sum    <= 0;
            fill_count     <= 0;
            data_out       <= 0;
            data_valid_out <= 1'b0;
            win_log2_reg   <= cfg_win_log2;
            for (i = 0; i < MAX_WIN; i = i + 1) buffer[i] <= 0;
        end else begin
            data_valid_out <= 1'b0;
            // Window reconfiguration flushes filter state -- a running sum
            // computed under one window size is meaningless under another.
            if (cfg_win_log2 != win_log2_reg) begin
                win_log2_reg <= cfg_win_log2;
                wr_ptr       <= 0;
                running_sum  <= 0;
                fill_count   <= 0;
                for (i = 0; i < MAX_WIN; i = i + 1) buffer[i] <= 0;
            end else if (en && data_valid_in) begin
                running_sum        <= new_sum;
                buffer[wr_ptr]      <= data_in;
                if (wr_ptr == window_size_m1)
                    wr_ptr <= 0;
                else
                    wr_ptr <= wr_ptr + 1'b1;
                if (fill_count <= window_size_m1)
                    fill_count <= fill_count + 1'b1;
                if (fill_count >= window_size_m1) begin
                    data_out       <= new_sum >> cfg_win_log2;
                    data_valid_out <= 1'b1;
                end
            end
        end
    end
endmodule