// =============================================================================
// File: early_prompt_late_correlator.v
// Module: early_prompt_late_correlator
// Project: Multi-Constellation Satellite GNSS Baseband Receiver ASIC
// Description: Complex In-Phase/Quadrature Early-Prompt-Late (EPL) Correlator.
//              - Complex carrier mixer: Rotates incoming RF ADC samples (I_in, Q_in)
//                with Carrier NCO (Cos, Sin) to strip Doppler carrier offset.
//              - Multiplies baseband signal with Early, Prompt, and Late PRN codes.
//              - 1ms coherent accumulators producing 24-bit integrated correlator
//                outputs (IE, QE, IP, QP, IL, QL) for DLL and PLL Costas loops.
// =============================================================================

`timescale 1ns / 1ps

module early_prompt_late_correlator (
    input  wire                 clk,
    input  wire                 rst_n,

    // Digitized RF Front-End ADC Samples (Signed 4-bit)
    input  wire signed [3:0]    adc_i,
    input  wire signed [3:0]    adc_q,

    // Carrier NCO Wipe-off Inputs
    input  wire signed [7:0]    carrier_cos,
    input  wire signed [7:0]    carrier_sin,

    // Code Replicas from Gold Code Generator
    input  wire                 code_early,
    input  wire                 code_prompt,
    input  wire                 code_late,

    // 1ms Epoch Dump / Reset Strobe
    input  wire                 epoch_dump,

    // Integrated Coherent Correlator Outputs (Latched on Epoch)
    output reg  signed [23:0]   out_i_early,
    output reg  signed [23:0]   out_q_early,
    output reg  signed [23:0]   out_i_prompt,
    output reg  signed [23:0]   out_q_prompt,
    output reg  signed [23:0]   out_i_late,
    output reg  signed [23:0]   out_q_late,
    output reg                  correlator_ready
);

    // 1. Complex Mixer: (I_in + j*Q_in) * (cos - j*sin)
    // I_bb = I_in*cos + Q_in*sin
    // Q_bb = Q_in*cos - I_in*sin
    reg signed [11:0] i_baseband;
    reg signed [11:0] q_baseband;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            i_baseband <= 12'sd0;
            q_baseband <= 12'sd0;
        end else begin
            i_baseband <= (adc_i * carrier_cos) + (adc_q * carrier_sin);
            q_baseband <= (adc_q * carrier_cos) - (adc_i * carrier_sin);
        end
    end

    // Convert binary chip (0, 1) to antipodal multiplier (+1, -1)
    wire signed [1:0] chip_e = code_early  ? -2'sd1 : 2'sd1;
    wire signed [1:0] chip_p = code_prompt ? -2'sd1 : 2'sd1;
    wire signed [1:0] chip_l = code_late   ? -2'sd1 : 2'sd1;

    // Coherent Accumulators (24-bit dynamic range)
    reg signed [23:0] acc_i_early;
    reg signed [23:0] acc_q_early;
    reg signed [23:0] acc_i_prompt;
    reg signed [23:0] acc_q_prompt;
    reg signed [23:0] acc_i_late;
    reg signed [23:0] acc_q_late;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_i_early      <= 24'sd0;
            acc_q_early      <= 24'sd0;
            acc_i_prompt     <= 24'sd0;
            acc_q_prompt     <= 24'sd0;
            acc_i_late       <= 24'sd0;
            acc_q_late       <= 24'sd0;
            out_i_early      <= 24'sd0;
            out_q_early      <= 24'sd0;
            out_i_prompt     <= 24'sd0;
            out_q_prompt     <= 24'sd0;
            out_i_late       <= 24'sd0;
            out_q_late       <= 24'sd0;
            correlator_ready <= 1'b0;
        end else begin
            correlator_ready <= 1'b0;

            if (epoch_dump) begin
                // Latch accumulated correlator outputs for software DSP / hardware tracking loops
                out_i_early      <= acc_i_early;
                out_q_early      <= acc_q_early;
                out_i_prompt     <= acc_i_prompt;
                out_q_prompt     <= acc_q_prompt;
                out_i_late       <= acc_i_late;
                out_q_late       <= acc_q_late;
                correlator_ready <= 1'b1;

                // Reset accumulators for next 1ms period
                acc_i_early      <= 24'sd0;
                acc_q_early      <= 24'sd0;
                acc_i_prompt     <= 24'sd0;
                acc_q_prompt     <= 24'sd0;
                acc_i_late       <= 24'sd0;
                acc_q_late       <= 24'sd0;
            end else begin
                // Coherent Correlation Accumulation
                acc_i_early  <= acc_i_early  + (i_baseband * chip_e);
                acc_q_early  <= acc_q_early  + (q_baseband * chip_e);
                acc_i_prompt <= acc_i_prompt + (i_baseband * chip_p);
                acc_q_prompt <= acc_q_prompt + (q_baseband * chip_p);
                acc_i_late   <= acc_i_late   + (i_baseband * chip_l);
                acc_q_late   <= acc_q_late   + (q_baseband * chip_l);
            end
        end
    end

endmodule
