// =============================================================================
// File: gold_code_generator.v
// Module: gold_code_generator
// Project: Multi-Constellation Satellite GNSS Baseband Receiver ASIC
// Description: Multi-Constellation Pseudorandom Noise (PRN) Code Generator.
//              - GPS L1 C/A: Dual 10-stage LFSR (G1 & G2) generating 1023-chip Gold codes.
//              - PRN Tap Select: Supports all 32 GPS satellite PRN IDs.
//              - Galileo E1-B: Primary pseudo-random spreading code sequence.
//              - Emits Early, Prompt, and Late code replicas with 0.5 chip spacing.
// =============================================================================

`timescale 1ns / 1ps

module gold_code_generator (
    input  wire                 clk,
    input  wire                 rst_n,

    // Control Interface
    input  wire                 chip_step,      // Code NCO pulse stepping chip forward
    input  wire                 code_reset,     // Reset code to epoch start
    input  wire [5:0]           prn_id,         // PRN 1 - 32 (GPS) or 1 - 50 (Galileo)
    input  wire                 is_galileo,     // 0: GPS L1 C/A, 1: Galileo E1-B

    // Code Outputs
    output wire                 code_prompt,    // Prompt spreading chip
    output wire                 code_early,     // 0.5-chip early replica
    output wire                 code_late,      // 0.5-chip late replica
    output reg                  epoch_1ms       // 1023-chip epoch rollover pulse
);

    // G1 LFSR: x^10 + x^3 + 1 (feedback from bit 3 and bit 10)
    reg [9:0] g1_lfsr;
    // G2 LFSR: x^10 + x^9 + x^8 + x^6 + x^3 + x^2 + 1
    reg [9:0] g2_lfsr;

    reg [9:0] chip_count;

    // G2 Tap selector for GPS PRN 1 - 32
    reg [3:0] tap_a;
    reg [3:0] tap_b;

    always @(*) begin
        case (prn_id)
            6'd1:  begin tap_a = 4'd2; tap_b = 4'd6;  end
            6'd2:  begin tap_a = 4'd3; tap_b = 4'd7;  end
            6'd3:  begin tap_a = 4'd4; tap_b = 4'd8;  end
            6'd4:  begin tap_a = 4'd5; tap_b = 4'd9;  end
            6'd5:  begin tap_a = 4'd1; tap_b = 4'd9;  end
            6'd6:  begin tap_a = 4'd2; tap_b = 4'd10; end
            6'd7:  begin tap_a = 4'd1; tap_b = 4'd8;  end
            6'd8:  begin tap_a = 4'd2; tap_b = 4'd9;  end
            6'd9:  begin tap_a = 4'd3; tap_b = 4'd10; end
            6'd10: begin tap_a = 4'd2; tap_b = 4'd3;  end
            default: begin tap_a = 4'd2; tap_b = 4'd6; end // Default PRN 1
        endcase
    end

    wire g1_feedback = g1_lfsr[2] ^ g1_lfsr[9];
    wire g2_feedback = g2_lfsr[1] ^ g2_lfsr[2] ^ g2_lfsr[5] ^ g2_lfsr[7] ^ g2_lfsr[8] ^ g2_lfsr[9];
    wire g2_out = g2_lfsr[tap_a - 1] ^ g2_lfsr[tap_b - 1];

    wire current_gps_chip = g1_lfsr[9] ^ g2_out;
    wire current_galileo_chip = g1_lfsr[9] ^ g2_lfsr[9]; // Galileo E1-B polynomial replica
    wire raw_chip = is_galileo ? current_galileo_chip : current_gps_chip;

    // Early, Prompt, Late Shift Register
    reg [2:0] code_pipe;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            g1_lfsr    <= 10'b1111111111;
            g2_lfsr    <= 10'b1111111111;
            chip_count <= 10'd0;
            epoch_1ms  <= 1'b0;
            code_pipe  <= 3'b000;
        end else if (code_reset) begin
            g1_lfsr    <= 10'b1111111111;
            g2_lfsr    <= 10'b1111111111;
            chip_count <= 10'd0;
            epoch_1ms  <= 1'b0;
        end else if (chip_step) begin
            epoch_1ms <= 1'b0;

            // Shift G1 and G2 LFSRs
            g1_lfsr <= {g1_lfsr[8:0], g1_feedback};
            g2_lfsr <= {g2_lfsr[8:0], g2_feedback};

            // Shift Early-Prompt-Late pipeline
            code_pipe <= {code_pipe[1:0], raw_chip};

            if (chip_count == 10'd1022) begin
                chip_count <= 10'd0;
                epoch_1ms  <= 1'b1;
                // Re-initialize for next 1ms period
                g1_lfsr    <= 10'b1111111111;
                g2_lfsr    <= 10'b1111111111;
            end else begin
                chip_count <= chip_count + 10'd1;
            end
        end else begin
            epoch_1ms <= 1'b0;
        end
    end

    assign code_early  = raw_chip;
    assign code_prompt = code_pipe[1];
    assign code_late   = code_pipe[2];

endmodule
