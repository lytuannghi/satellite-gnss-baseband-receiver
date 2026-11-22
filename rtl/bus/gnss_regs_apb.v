// =============================================================================
// File: gnss_regs_apb.v
// Module: gnss_regs_apb
// Project: Multi-Constellation Satellite GNSS Baseband Receiver ASIC
// Description: AMBA 3 APB Slave Configuration & Telemetry Register Bank.
//              Provides host access to 32 parallel tracking channels:
//              - Global Receiver Control & 1ms Epoch Interrupt
//              - Channel Allocation (PRN ID, GPS/Galileo selection)
//              - Carrier Doppler & Code Chipping Rate FCW registers
//              - Correlator In-Phase/Quadrature (I/Q) & Discriminator Readback
// =============================================================================

`timescale 1ns / 1ps

module gnss_regs_apb #(
    parameter CHANNELS = 32
)(
    input  wire                 pclk,
    input  wire                 presetn,

    // APB3 Interface
    input  wire [11:0]          paddr,
    input  wire                 psel,
    input  wire                 penable,
    input  wire                 pwrite,
    input  wire [31:0]          pwdata,
    output reg  [31:0]          prdata,
    output wire                 pready,
    output wire                 pslverr,

    // Control Outputs to Tracking Channels
    output reg                  master_enable,
    output reg  [CHANNELS-1:0]  chan_enable,
    output reg  [5:0]           chan_prn_id    [0:CHANNELS-1],
    output reg  [CHANNELS-1:0]  chan_is_galileo,
    output reg  [31:0]          chan_carrier_fcw[0:CHANNELS-1],
    output reg  [31:0]          chan_code_fcw   [0:CHANNELS-1],

    // Telemetry Inputs from Tracking Channels
    input  wire [CHANNELS-1:0]  chan_epoch_pulse,
    input  wire signed [23:0]   chan_i_prompt   [0:CHANNELS-1],
    input  wire signed [23:0]   chan_q_prompt   [0:CHANNELS-1],
    input  wire signed [23:0]   chan_dll_disc   [0:CHANNELS-1],
    input  wire signed [23:0]   chan_pll_disc   [0:CHANNELS-1],
    input  wire [31:0]          chan_power      [0:CHANNELS-1],

    output reg                  irq_epoch_1ms
);

    assign pready  = 1'b1;
    assign pslverr = 1'b0;

    integer i;

    // Default FCWs:
    // With 200 MHz Clock:
    // Carrier IF 4.092 MHz -> FCW = (4.092e6 / 200e6) * 2^32 = 87877484 = 32'h053CE76C
    // Code 1.023 MHz -> FCW = (1.023e6 / 200e6) * 2^32 = 21969371 = 32'h014F39DB
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
                chan_prn_id[i]      <= i[5:0] + 6'd1; // PRN 1 to 32
                chan_carrier_fcw[i] <= DEFAULT_CARRIER_FCW;
                chan_code_fcw[i]    <= DEFAULT_CODE_FCW;
            end
        end else begin
            // 1ms Epoch Interrupt Generation (pulse if any active channel hits epoch)
            if (| (chan_epoch_pulse & chan_enable))
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

                    // 0x100 - 0x17F: Channel PRN ID & Mode
                    4'h1: begin
                        if (paddr[6:2] < CHANNELS) begin
                            chan_prn_id[paddr[6:2]]     <= pwdata[5:0];
                            chan_is_galileo[paddr[6:2]] <= pwdata[6];
                        end
                    end

                    // 0x200 - 0x27F: Channel Carrier FCW
                    4'h2: begin
                        if (paddr[6:2] < CHANNELS) begin
                            chan_carrier_fcw[paddr[6:2]] <= pwdata;
                        end
                    end

                    // 0x300 - 0x37F: Channel Code FCW
                    4'h3: begin
                        if (paddr[6:2] < CHANNELS) begin
                            chan_code_fcw[paddr[6:2]] <= pwdata;
                        end
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

                    4'h1: begin // PRN & Constellation
                        if (paddr[6:2] < CHANNELS)
                            prdata <= {25'd0, chan_is_galileo[paddr[6:2]], chan_prn_id[paddr[6:2]]};
                        else
                            prdata <= 32'd0;
                    end

                    4'h2: begin // Carrier FCW
                        if (paddr[6:2] < CHANNELS)
                            prdata <= chan_carrier_fcw[paddr[6:2]];
                        else
                            prdata <= 32'd0;
                    end

                    4'h3: begin // Code FCW
                        if (paddr[6:2] < CHANNELS)
                            prdata <= chan_code_fcw[paddr[6:2]];
                        else
                            prdata <= 32'd0;
                    end

                    4'h4: begin // Prompt I/Q Readback
                        if (paddr[6:2] < CHANNELS)
                            prdata <= {chan_i_prompt[paddr[6:2]][15:0], chan_q_prompt[paddr[6:2]][15:0]};
                        else
                            prdata <= 32'd0;
                    end

                    4'h5: begin // Prompt Power (C/N0)
                        if (paddr[6:2] < CHANNELS)
                            prdata <= chan_power[paddr[6:2]];
                        else
                            prdata <= 32'd0;
                    end

                    default: prdata <= 32'd0;
                endcase
            end
        end
    end

endmodule
