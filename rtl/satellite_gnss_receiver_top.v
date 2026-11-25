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

    // Packed Verilog-2005 Register File Interconnect Buses
    wire                    master_enable;
    wire [CHANNELS-1:0]     chan_enable;
    wire [CHANNELS*6-1:0]   chan_prn_id_bus;
    wire [CHANNELS-1:0]     chan_is_galileo;
    wire [CHANNELS*32-1:0]  chan_carrier_fcw_bus;
    wire [CHANNELS*32-1:0]  chan_code_fcw_bus;

    wire [CHANNELS-1:0]     chan_epoch_pulse;
    wire [CHANNELS*24-1:0]  chan_i_prompt_bus;
    wire [CHANNELS*24-1:0]  chan_q_prompt_bus;
    wire [CHANNELS*24-1:0]  chan_dll_disc_bus;
    wire [CHANNELS*24-1:0]  chan_pll_disc_bus;
    wire [CHANNELS*32-1:0]  chan_power_bus;

    // APB3 Register Bank
    gnss_regs_apb #(
        .CHANNELS(CHANNELS)
    ) u_regs (
        .pclk                (clk),
        .presetn             (rst_n),
        .paddr               (paddr),
        .psel                (psel),
        .penable             (penable),
        .pwrite              (pwrite),
        .pwdata              (pwdata),
        .prdata              (prdata),
        .pready              (pready),
        .pslverr             (pslverr),
        .master_enable       (master_enable),
        .chan_enable         (chan_enable),
        .chan_prn_id_bus     (chan_prn_id_bus),
        .chan_is_galileo     (chan_is_galileo),
        .chan_carrier_fcw_bus(chan_carrier_fcw_bus),
        .chan_code_fcw_bus   (chan_code_fcw_bus),
        .chan_epoch_pulse    (chan_epoch_pulse),
        .chan_i_prompt_bus   (chan_i_prompt_bus),
        .chan_q_prompt_bus   (chan_q_prompt_bus),
        .chan_dll_disc_bus   (chan_dll_disc_bus),
        .chan_pll_disc_bus   (chan_pll_disc_bus),
        .chan_power_bus      (chan_power_bus),
        .irq_epoch_1ms       (irq_epoch_1ms)
    );

    // 32 Parallel Hardware Satellite Tracking Channels
    genvar ch;
    generate
        for (ch = 0; ch < CHANNELS; ch = ch + 1) begin: gen_channels
            wire signed [23:0] ch_i_prompt;
            wire signed [23:0] ch_q_prompt;
            wire signed [23:0] ch_dll_disc;
            wire signed [23:0] ch_pll_disc;
            wire [31:0]        ch_power;

            assign chan_i_prompt_bus[ch*24 +: 24] = ch_i_prompt;
            assign chan_q_prompt_bus[ch*24 +: 24] = ch_q_prompt;
            assign chan_dll_disc_bus[ch*24 +: 24] = ch_dll_disc;
            assign chan_pll_disc_bus[ch*24 +: 24] = ch_pll_disc;
            assign chan_power_bus[ch*32 +: 32]    = ch_power;

            gnss_tracking_channel u_channel (
                .clk              (clk),
                .rst_n            (rst_n && master_enable),
                .channel_enable   (chan_enable[ch]),
                .cfg_prn_id       (chan_prn_id_bus[ch*6 +: 6]),
                .cfg_is_galileo   (chan_is_galileo[ch]),
                .cfg_carrier_fcw  (chan_carrier_fcw_bus[ch*32 +: 32]),
                .cfg_code_fcw     (chan_code_fcw_bus[ch*32 +: 32]),
                .cfg_code_reset   (1'b0),
                .adc_i            (rf_adc_i),
                .adc_q            (rf_adc_q),
                .out_i_prompt     (ch_i_prompt),
                .out_q_prompt     (ch_q_prompt),
                .out_i_early      (),
                .out_q_early      (),
                .out_i_late       (),
                .out_q_late       (),
                .dll_discriminator(ch_dll_disc),
                .pll_discriminator(ch_pll_disc),
                .prompt_power     (ch_power),
                .epoch_1ms_pulse  (chan_epoch_pulse[ch]),
                .channel_ready    ()
            );
        end
    endgenerate

    assign chan_epoch_out = chan_epoch_pulse;

endmodule
