// =============================================================================
// File: carrier_code_nco.v
// Module: carrier_code_nco
// Project: Multi-Constellation Satellite GNSS Baseband Receiver ASIC
// Description: Dual Numerically Controlled Oscillator (Carrier NCO & Code NCO).
//              - Carrier NCO: 32-bit phase accumulator tracking Doppler shifts
//                (-10 kHz to +10 kHz) and intermediate frequency (IF).
//                Generates 8-bit quantized Sine and Cosine local carrier replicas.
//              - Code NCO: 32-bit phase accumulator tracking Doppler-induced code
//                dilation and synthesizing the 1.023 MHz chipping rate clock.
// =============================================================================

`timescale 1ns / 1ps

module carrier_code_nco (
    input  wire                 clk,
    input  wire                 rst_n,

    // NCO Frequency Control Words (from Tracking Loops)
    input  wire [31:0]          carrier_fcw,    // Carrier frequency control word
    input  wire [31:0]          code_fcw,       // Code chipping rate control word

    // Carrier Outputs (Doppler Wipe-off)
    output reg  signed [7:0]    carrier_cos,    // 8-bit In-phase carrier replica
    output reg  signed [7:0]    carrier_sin,    // 8-bit Quadrature carrier replica
    output reg  [31:0]          carrier_phase,  // Phase accumulator state

    // Code Chipping Outputs
    output reg                  chip_step_pulse,// 1-cycle strobe per chip advance
    output reg  [31:0]          code_phase      // Sub-chip phase accumulator state
);

    // 32-bit Phase Accumulators
    reg [31:0] carrier_acc;
    reg [31:0] code_acc;
    reg        code_msb_prev;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            carrier_acc     <= 32'd0;
            code_acc        <= 32'd0;
            code_msb_prev   <= 1'b0;
            chip_step_pulse <= 1'b0;
            carrier_phase   <= 32'd0;
            code_phase      <= 32'd0;
            carrier_cos     <= 8'sd127;
            carrier_sin     <= 8'sd0;
        end else begin
            // Accumulate Carrier Phase
            carrier_acc   <= carrier_acc + carrier_fcw;
            carrier_phase <= carrier_acc;

            // Generate 8-bit Sine/Cosine from top 2 phase bits (Quadrant mapping)
            case (carrier_acc[31:30])
                2'b00: begin carrier_cos <= 8'sd127; carrier_sin <= 8'sd0;   end // 0 deg
                2'b01: begin carrier_cos <= 8'sd0;   carrier_sin <= 8'sd127; end // 90 deg
                2'b10: begin carrier_cos <= -8'sd127;carrier_sin <= 8'sd0;   end // 180 deg
                2'b11: begin carrier_cos <= 8'sd0;   carrier_sin <= -8'sd127;end // 270 deg
            endcase

            // Accumulate Code Phase
            code_acc      <= code_acc + code_fcw;
            code_phase    <= code_acc;
            code_msb_prev <= code_acc[31];

            // Chip step pulse triggers on Code MSB transition (Clock edge equivalent)
            if (code_acc[31] && !code_msb_prev) begin
                chip_step_pulse <= 1'b1;
            end else begin
                chip_step_pulse <= 1'b0;
            end
        end
    end

endmodule
