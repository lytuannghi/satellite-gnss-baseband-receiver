// =============================================================================
// File: gnss_tracking_channel.v
// Module: gnss_tracking_channel
// Project: Multi-Constellation Satellite GNSS Baseband Receiver ASIC
// Description: Complete Single Satellite Tracking Channel.
//              - Integrates Carrier NCO, Code NCO, PRN Generator, and EPL Correlator.
//              - Evaluates Hardware Discriminators:
//                * DLL Code Phase Error: (IE - IL) * IP + (QE - QL) * QP
//                * PLL Costas Phase Error: QP * sign(IP)
//              - Calculates Prompt Power for C/N0 (Carrier-to-Noise Ratio) estimation.
// =============================================================================

`timescale 1ns / 1ps

module gnss_tracking_channel (
    input  wire                 clk,
    input  wire                 rst_n,

    // Channel Configuration
    input  wire                 channel_enable,
    input  wire [5:0]           cfg_prn_id,
    input  wire                 cfg_is_galileo,
    input  wire [31:0]          cfg_carrier_fcw,
    input  wire [31:0]          cfg_code_fcw,
    input  wire                 cfg_code_reset,

    // RF Front-End ADC Input
    input  wire signed [3:0]    adc_i,
    input  wire signed [3:0]    adc_q,

    // Correlator Outputs
    output wire signed [23:0]   out_i_prompt,
    output wire signed [23:0]   out_q_prompt,
    output wire signed [23:0]   out_i_early,
    output wire signed [23:0]   out_q_early,
    output wire signed [23:0]   out_i_late,
    output wire signed [23:0]   out_q_late,

    // Tracking Loop Discriminator Feedback
    output reg  signed [23:0]   dll_discriminator, // Code tracking error
    output reg  signed [23:0]   pll_discriminator, // Carrier tracking error
    output reg  [31:0]          prompt_power,      // IP^2 + QP^2

    // Timing Epoch
    output wire                 epoch_1ms_pulse,
    output wire                 channel_ready
);

    // 1. Dual Carrier/Code NCO
    wire signed [7:0] carrier_cos;
    wire signed [7:0] carrier_sin;
    wire [31:0]       carrier_phase;
    wire              chip_step_pulse;
    wire [31:0]       code_phase;

    carrier_code_nco u_nco (
        .clk            (clk),
        .rst_n          (rst_n && channel_enable),
        .carrier_fcw    (cfg_carrier_fcw),
        .code_fcw       (cfg_code_fcw),
        .carrier_cos    (carrier_cos),
        .carrier_sin    (carrier_sin),
        .carrier_phase  (carrier_phase),
        .chip_step_pulse(chip_step_pulse),
        .code_phase     (code_phase)
    );

    // 2. PRN Code Generator
    wire code_prompt;
    wire code_early;
    wire code_late;

    gold_code_generator u_prn_gen (
        .clk        (clk),
        .rst_n      (rst_n && channel_enable),
        .chip_step  (chip_step_pulse),
        .code_reset (cfg_code_reset),
        .prn_id     (cfg_prn_id),
        .is_galileo (cfg_is_galileo),
        .code_prompt(code_prompt),
        .code_early (code_early),
        .code_late  (code_late),
        .epoch_1ms  (epoch_1ms_pulse)
    );

    // 3. Early-Prompt-Late Correlator
    early_prompt_late_correlator u_correlator (
        .clk             (clk),
        .rst_n           (rst_n && channel_enable),
        .adc_i           (adc_i),
        .adc_q           (adc_q),
        .carrier_cos     (carrier_cos),
        .carrier_sin     (carrier_sin),
        .code_early      (code_early),
        .code_prompt     (code_prompt),
        .code_late       (code_late),
        .epoch_dump      (epoch_1ms_pulse),
        .out_i_early     (out_i_early),
        .out_q_early     (out_q_early),
        .out_i_prompt    (out_i_prompt),
        .out_q_prompt    (out_q_prompt),
        .out_i_late      (out_i_late),
        .out_q_late      (out_q_late),
        .correlator_ready(channel_ready)
    );

    // 4. Tracking Discriminators (Evaluated on Epoch Dump)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dll_discriminator <= 24'sd0;
            pll_discriminator <= 24'sd0;
            prompt_power      <= 32'd0;
        end else if (epoch_1ms_pulse) begin
            // Dot-Product DLL Discriminator: (IE - IL)*IP + (QE - QL)*QP
            dll_discriminator <= ((out_i_early - out_i_late) * (out_i_prompt >>> 8)) +
                                 ((out_q_early - out_q_late) * (out_q_prompt >>> 8));

            // Costas PLL Discriminator: QP * sign(IP)
            pll_discriminator <= (out_i_prompt >= 24'sd0) ? out_q_prompt : -out_q_prompt;

            // Prompt Signal Power: IP^2 + QP^2 (Scaled to avoid 32-bit overflow)
            prompt_power <= ((out_i_prompt >>> 6) * (out_i_prompt >>> 6)) +
                            ((out_q_prompt >>> 6) * (out_q_prompt >>> 6));
        end
    end

endmodule
