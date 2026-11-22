// =============================================================================
// File: satellite_gnss_receiver_top.v
// Module: satellite_gnss_receiver_top
// Project: Multi-Constellation Satellite GNSS Baseband Receiver ASIC
// Description: Multi-Constellation Satellite GNSS Baseband Receiver SoC Top-Level.
//              - Supports 32 parallel hardware tracking channels (GPS L1 C/A & Galileo E1).
//              - AMBA APB3 Slave configuration and telemetry bus.
//              - Digital RF Front-End ADC Interface (Signed 4-bit In-Phase & Quadrature).
//              - Dedicated Early-Prompt-Late correlators per satellite channel.
//              - 1ms Epoch Interrupt (PPS synchronized).
// =============================================================================

`timescale 1ns / 1ps

module satellite_gnss_receiver_top #(
    parameter CHANNELS = 32
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // Digital RF Front-End ADC Interface (from MAX2769 / NT1065 RF IC)
    input  wire signed [3:0]    rf_adc_i,
    input  wire signed [3:0]    rf_adc_q,

    // AMBA 3 APB Register Interface
    input  wire [11:0]          paddr,
    input  wire                 psel,
    input  wire                 penable,
    input  wire                 pwrite,
    input  wire [31:0]          pwdata,
    output wire [31:0]          prdata,
    output wire                 pready,
    output wire                 pslverr,

    // Timing & Interrupt Outputs
    output wire                 irq_epoch_1ms,
    output wire [CHANNELS-1:0]  chan_epoch_out
);

    // Register File Interconnect
    wire                 master_enable;
    wire [CHANNELS-1:0]  chan_enable;
    wire [5:0]           chan_prn_id     [0:CHANNELS-1];
    wire [CHANNELS-1:0]  chan_is_galileo;
    wire [31:0]          chan_carrier_fcw[0:CHANNELS-1];
    wire [31:0]          chan_code_fcw   [0:CHANNELS-1];

    wire [CHANNELS-1:0]  chan_epoch_pulse;
    wire signed [23:0]   chan_i_prompt   [0:CHANNELS-1];
    wire signed [23:0]   chan_q_prompt   [0:CHANNELS-1];
    wire signed [23:0]   chan_dll_disc   [0:CHANNELS-1];
    wire signed [23:0]   chan_pll_disc   [0:CHANNELS-1];
    wire [31:0]          chan_power      [0:CHANNELS-1];

    // APB3 Register Bank
    gnss_regs_apb #(
        .CHANNELS(CHANNELS)
    ) u_regs (
        .pclk            (clk),
        .presetn         (rst_n),
        .paddr           (paddr),
        .psel            (psel),
        .penable         (penable),
        .pwrite          (pwrite),
        .pwdata          (pwdata),
        .prdata          (prdata),
        .pready          (pready),
        .pslverr         (pslverr),
        .master_enable   (master_enable),
        .chan_enable     (chan_enable),
        .chan_prn_id     (chan_prn_id),
        .chan_is_galileo (chan_is_galileo),
        .chan_carrier_fcw(chan_carrier_fcw),
        .chan_code_fcw   (chan_code_fcw),
        .chan_epoch_pulse(chan_epoch_pulse),
        .chan_i_prompt   (chan_i_prompt),
        .chan_q_prompt   (chan_q_prompt),
        .chan_dll_disc   (chan_dll_disc),
        .chan_pll_disc   (chan_pll_disc),
        .chan_power      (chan_power),
        .irq_epoch_1ms   (irq_epoch_1ms)
    );

    // 32 Parallel Hardware Satellite Tracking Channels
    genvar ch;
    generate
        for (ch = 0; ch < CHANNELS; ch = ch + 1) begin: gen_channels
            gnss_tracking_channel u_channel (
                .clk              (clk),
                .rst_n            (rst_n && master_enable),
                .channel_enable   (chan_enable[ch]),
                .cfg_prn_id       (chan_prn_id[ch]),
                .cfg_is_galileo   (chan_is_galileo[ch]),
                .cfg_carrier_fcw  (chan_carrier_fcw[ch]),
                .cfg_code_fcw     (chan_code_fcw[ch]),
                .cfg_code_reset   (1'b0),
                .adc_i            (rf_adc_i),
                .adc_q            (rf_adc_q),
                .out_i_prompt     (chan_i_prompt[ch]),
                .out_q_prompt     (chan_q_prompt[ch]),
                .out_i_early      (),
                .out_q_early      (),
                .out_i_late       (),
                .out_q_late       (),
                .dll_discriminator(chan_dll_disc[ch]),
                .pll_discriminator(chan_pll_disc[ch]),
                .prompt_power     (chan_power[ch]),
                .epoch_1ms_pulse  (chan_epoch_pulse[ch]),
                .channel_ready    ()
            );
        end
    endgenerate

    assign chan_epoch_out = chan_epoch_pulse;

endmodule
