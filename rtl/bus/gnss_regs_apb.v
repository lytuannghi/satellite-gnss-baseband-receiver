// =============================================================================
// File: gnss_regs_apb.v
// Module: gnss_regs_apb
// Project: Multi-Constellation Satellite GNSS Baseband Receiver ASIC
// Description: AMBA 3 APB Slave Configuration & Telemetry Register Bank.
//              Uses packed Verilog-2005 vectors for multi-channel buses.
//              Provides host access to 32 parallel tracking channels:
//              - Global Receiver Control & 1ms Epoch Interrupt
//              - Channel Allocation (PRN ID, GPS/Galileo selection)
//              - Carrier Doppler & Code Chipping Rate FCW registers
//              - Correlator I/Q, Power, DLL & PLL Discriminator Readback
// =============================================================================

`timescale 1ns / 1ps

module gnss_regs_apb #(
    parameter CHANNELS = 32
)(
    input  wire                     pclk,
    input  wire                     presetn,

    // APB3 Interface
    input  wire [11:0]              paddr,
    input  wire                     psel,
    input  wire                     penable,
    input  wire                     pwrite,
    input  wire [31:0]              pwdata,
    output reg  [31:0]              prdata,
    output wire                     pready,
    output wire                     pslverr,

    // Control Outputs to Tracking Channels (Packed Buses)
    output reg                      master_enable,
    output reg  [CHANNELS-1:0]      chan_enable,
    output reg  [CHANNELS*6-1:0]    chan_prn_id_bus,
    output reg  [CHANNELS-1:0]      chan_is_galileo,
    output reg  [CHANNELS*32-1:0]   chan_carrier_fcw_bus,
    output reg  [CHANNELS*32-1:0]   chan_code_fcw_bus,

    // Telemetry Inputs from Tracking Channels (Packed Buses)
    input  wire [CHANNELS-1:0]      chan_epoch_pulse,
    input  wire [CHANNELS*24-1:0]   chan_i_prompt_bus,
    input  wire [CHANNELS*24-1:0]   chan_q_prompt_bus,
    input  wire [CHANNELS*24-1:0]   chan_dll_disc_bus,
    input  wire [CHANNELS*24-1:0]   chan_pll_disc_bus,
    input  wire [CHANNELS*32-1:0]   chan_power_bus,

    output reg                      irq_epoch_1ms
);

    assign pready  = 1'b1;
    assign pslverr = 1'b0;

    integer i;
    wire [4:0] ch_idx = paddr[6:2];

    localparam DEFAULT_CARRIER_FCW = 32'h053CE76C;
    localparam DEFAULT_CODE_FCW    = 32'h014F39DB;

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            master_enable   <= 1'b0;
            chan_enable     <= {CHANNELS{1'b0}};
            chan_is_galileo <= {CHANNELS{1'b0}};
            irq_epoch_1ms   <= 1'b0;
            prdata          <= 32'd0;

            for (i = 0; i < CHANNELS; i = i + 1) begin
                chan_prn_id_bus[i*6 +: 6]       <= i[5:0] + 6'd1;
                chan_carrier_fcw_bus[i*32 +: 32] <= DEFAULT_CARRIER_FCW;
                chan_code_fcw_bus[i*32 +: 32]    <= DEFAULT_CODE_FCW;
            end
        end else begin
            if (|(chan_epoch_pulse & chan_enable))
                irq_epoch_1ms <= 1'b1;
            else
                irq_epoch_1ms <= 1'b0;

            // APB Write Operation
            if (psel && penable && pwrite) begin
                case (paddr[11:8])
                    4'h0: begin
                        if (paddr[7:0] == 8'h00) master_enable <= pwdata[0];
                        if (paddr[7:0] == 8'h04) chan_enable   <= pwdata[CHANNELS-1:0];
                    end
                    4'h1: begin
                        if (ch_idx < CHANNELS) begin
                            chan_prn_id_bus[ch_idx*6 +: 6] <= pwdata[5:0];
                            chan_is_galileo[ch_idx]        <= pwdata[6];
                        end
                    end
                    4'h2: begin
                        if (ch_idx < CHANNELS)
                            chan_carrier_fcw_bus[ch_idx*32 +: 32] <= pwdata;
                    end
                    4'h3: begin
                        if (ch_idx < CHANNELS)
                            chan_code_fcw_bus[ch_idx*32 +: 32] <= pwdata;
                    end
                    default: ;
                endcase
            end

            // APB Read Operation
            if (psel && !pwrite) begin
                case (paddr[11:8])
                    4'h0: begin
                        case (paddr[7:0])
                            8'h00: prdata <= {31'd0, master_enable};
                            8'h04: prdata <= chan_enable;
                            8'h08: prdata <= {31'd0, irq_epoch_1ms};
                            default: prdata <= 32'd0;
                        endcase
                    end
                    4'h1: begin
                        if (ch_idx < CHANNELS)
                            prdata <= {25'd0, chan_is_galileo[ch_idx], chan_prn_id_bus[ch_idx*6 +: 6]};
                        else
                            prdata <= 32'd0;
                    end
                    4'h2: prdata <= (ch_idx < CHANNELS) ? chan_carrier_fcw_bus[ch_idx*32 +: 32] : 32'd0;
                    4'h3: prdata <= (ch_idx < CHANNELS) ? chan_code_fcw_bus[ch_idx*32 +: 32] : 32'd0;
                    4'h4: begin
                        if (ch_idx < CHANNELS)
                            prdata <= {chan_i_prompt_bus[ch_idx*24 +: 16], chan_q_prompt_bus[ch_idx*24 +: 16]};
                        else
                            prdata <= 32'd0;
                    end
                    4'h5: prdata <= (ch_idx < CHANNELS) ? chan_power_bus[ch_idx*32 +: 32] : 32'd0;
                    4'h6: begin
                        if (ch_idx < CHANNELS)
                            prdata <= {{8{chan_dll_disc_bus[ch_idx*24 + 23]}}, chan_dll_disc_bus[ch_idx*24 +: 24]};
                        else
                            prdata <= 32'd0;
                    end
                    4'h7: begin
                        if (ch_idx < CHANNELS)
                            prdata <= {{8{chan_pll_disc_bus[ch_idx*24 + 23]}}, chan_pll_disc_bus[ch_idx*24 +: 24]};
                        else
                            prdata <= 32'd0;
                    end
                    default: prdata <= 32'd0;
                endcase
            end
        end
    end

endmodule
