// =============================================================================
// tiny_mlp.v
// -----------------------------------------------------------------------------
// Q1.15 fixed-point 4-8-4-2 "Tiny MLP" inference core (README Sec.7 note 1:
// this is a Tiny MLP, NOT a BNN -- no torch.sign() binarization, no XNOR /
// popcount logic anywhere in this file).
//
//   Input:  [gas_norm, flame_norm, ultra_norm, temp_norm]   (4, Q1.15)
//   Hidden1: 8 neurons, ReLU           (4x8  = 32 weights)
//   Hidden2: 4 neurons, ReLU           (8x4  = 32 weights)
//   Output:  2 neurons (argmax)        (4x2  =  8 weights)
//   Total weights: 72 (32+32+8), matching README Sec.3/Sec.5.
//
// Weights are loaded from BRAM/registers populated offline: PyTorch (float32)
// training -> post-training quantization to Q1.15 (round-to-nearest, clamp)
// -> .coe export -> loaded via AXI4-Lite MLP_WEIGHT_[n] registers at boot
// (README Sec.7 note 8). No resynthesis required to update weights.
//
// Single reused MAC, sequential FSM -- 72 weights is far too small to justify
// a fully-parallel or binarized datapath (README Sec.2 rationale). Latency
// ~76 MAC cycles + control overhead, well within the ~100 cycle / 1us budget
// at 100 MHz quoted in README Sec.3.
//
// This module is purely combinational-input / pipelined-output: whether it
// is actually consulted for the final verdict is decided by decision_pipeline.v,
// which forces hard-rules-only mode whenever fault_flag is asserted
// (README Sec.7 note 5). tiny_mlp.v itself always runs; gating happens downstream.
// =============================================================================
// =============================================================================
// tiny_mlp.v  (unchanged -- no issues raised against this module; Kimi
// review confirms Q1.15 4-8-4-2 weight indexing, ReLU-only-on-hidden-layers,
// and Q1.15 rescaling via bit-slice are all correct)
// =============================================================================
module tiny_mlp #(
    parameter DATA_WIDTH = 16   // Q1.15
)(
    input  wire                          clk,
    input  wire                          rst,
    input  wire                          start,          // pulse: begin inference
    input  wire signed [DATA_WIDTH-1:0]  gas_norm,
    input  wire signed [DATA_WIDTH-1:0]  flame_norm,
    input  wire signed [DATA_WIDTH-1:0]  ultra_norm,
    input  wire signed [DATA_WIDTH-1:0]  temp_norm,

    input  wire                          weight_wr_en,
    input  wire [6:0]                    weight_wr_addr,  // 0..71
    input  wire signed [DATA_WIDTH-1:0]  weight_wr_data,

    output reg                           mlp_valid,
    output reg                           mlp_class,       // 0 = safe, 1 = unsafe (argmax)
    output reg  [7:0]                    mlp_confidence   // 0-255, winning-neuron score proxy
);

    reg signed [DATA_WIDTH-1:0] weights [0:71];

    always @(posedge clk) begin
        if (weight_wr_en) weights[weight_wr_addr] <= weight_wr_data;
    end

    reg signed [DATA_WIDTH-1:0] layer_in  [0:7];
    reg signed [DATA_WIDTH-1:0] layer_out [0:7];

    localparam [2:0]
        S_IDLE    = 3'd0,
        S_L1      = 3'd1,
        S_L2      = 3'd2,
        S_L3      = 3'd3,
        S_ARGMAX  = 3'd4,
        S_DONE    = 3'd5;

    reg [2:0]  state;
    reg [3:0]  neuron_idx;
    reg [3:0]  input_idx;
    reg [6:0]  w_base;
    reg [3:0]  n_inputs;
    reg [3:0]  n_neurons;
    reg signed [2*DATA_WIDTH-1:0] mac_acc;

    integer k;

    always @(posedge clk) begin
        if (rst) begin
            state       <= S_IDLE;
            neuron_idx  <= 0;
            input_idx   <= 0;
            w_base      <= 0;
            mac_acc     <= 0;
            mlp_valid   <= 1'b0;
            mlp_class   <= 1'b0;
            mlp_confidence <= 8'd0;
            n_inputs    <= 0;
            n_neurons   <= 0;
            for (k = 0; k < 8; k = k + 1) begin
                layer_in[k]  <= {DATA_WIDTH{1'b0}};
                layer_out[k] <= {DATA_WIDTH{1'b0}};
            end
        end else begin
            mlp_valid <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start) begin
                        layer_in[0] <= gas_norm;
                        layer_in[1] <= flame_norm;
                        layer_in[2] <= ultra_norm;
                        layer_in[3] <= temp_norm;
                        n_inputs    <= 4'd4;
                        n_neurons   <= 4'd8;
                        w_base      <= 7'd0;
                        neuron_idx  <= 0;
                        input_idx   <= 0;
                        mac_acc     <= 0;
                        state       <= S_L1;
                    end
                end

                S_L1, S_L2, S_L3: begin
                    if (input_idx < n_inputs) begin
                        mac_acc   <= mac_acc + (layer_in[input_idx] * weights[w_base + (neuron_idx*n_inputs) + input_idx]);
                        input_idx <= input_idx + 1'b1;
                    end else begin
                        if (state == S_L3) begin
                            layer_out[neuron_idx] <= mac_acc[2*DATA_WIDTH-1 -: DATA_WIDTH];
                        end else begin
                            layer_out[neuron_idx] <= (mac_acc[2*DATA_WIDTH-1] ? {DATA_WIDTH{1'b0}}
                                                                                : mac_acc[2*DATA_WIDTH-2 -: DATA_WIDTH]);
                        end
                        mac_acc   <= 0;
                        input_idx <= 0;

                        if (neuron_idx == n_neurons - 1) begin
                            if (state == S_L1) begin
                                for (k = 0; k < 8; k = k + 1) layer_in[k] <= layer_out[k];
                                n_inputs   <= 4'd8;
                                n_neurons  <= 4'd4;
                                w_base     <= 7'd32;
                                neuron_idx <= 0;
                                state      <= S_L2;
                            end else if (state == S_L2) begin
                                for (k = 0; k < 4; k = k + 1) layer_in[k] <= layer_out[k];
                                n_inputs   <= 4'd4;
                                n_neurons  <= 4'd2;
                                w_base     <= 7'd64;
                                neuron_idx <= 0;
                                state      <= S_L3;
                            end else begin
                                state <= S_ARGMAX;
                            end
                        end else begin
                            neuron_idx <= neuron_idx + 1'b1;
                        end
                    end
                end

                S_ARGMAX: begin
                    if (layer_out[1] > layer_out[0]) begin
                        mlp_class <= 1'b1;
                        mlp_confidence <= (layer_out[1] + 16'sd32768) >> 8;
                    end else begin
                        mlp_class <= 1'b0;
                        mlp_confidence <= (layer_out[0] + 16'sd32768) >> 8;
                    end
                    mlp_valid <= 1'b1;
                    state <= S_DONE;
                end

                S_DONE: begin
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule