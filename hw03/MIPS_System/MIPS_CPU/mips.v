`timescale 1ns/1ps
`define mydelay 1

//--------------------------------------------------------------
// mips.v
// David_Harris@hmc.edu and Sarah_Harris@hmc.edu 23 October 2005
// Single-cycle MIPS processor
//--------------------------------------------------------------

// single-cycle MIPS processor
module mips(input         clk, reset,
            output [31:0] pc,
            input  [31:0] instr,
            output        memwrite,
            output [31:0] memaddr,
            output [31:0] memwritedata,
            input  [31:0] memreaddata);

  wire        signext, shiftl16, memtoreg, branch;
  wire        pcsrc, zero;
  wire        alusrc, regdst, regwrite, jump, jr; // <- modified line
  wire [3:0]  alucontrol; // <- modified line

  // Instantiate Controller
  controller c(
    .op         (instr[31:26]), 
		.funct      (instr[5:0]), 
		.zero       (zero),
		.signext    (signext),
		.shiftl16   (shiftl16),
		.memtoreg   (memtoreg),
		.memwrite   (memwrite),
		.pcsrc      (pcsrc),
		.alusrc     (alusrc),
		.regdst     (regdst),
		.regwrite   (regwrite),
		.jump       (jump),
    .jr         (jr), // <- modified line
		.alucontrol (alucontrol));

  // Instantiate Datapath
  datapath dp(
    .clk        (clk),
    .reset      (reset),
    .signext    (signext),
    .shiftl16   (shiftl16),
    .memtoreg   (memtoreg),
    .pcsrc      (pcsrc),
    .alusrc     (alusrc),
    .regdst     (regdst),
    .regwrite   (regwrite),
    .jump       (jump),
    .jr         (jr), // <- modified line
    .alucontrol (alucontrol),
    .zero       (zero),
    .pc         (pc),
    .instr      (instr),
    .aluout     (memaddr), 
    .writedata  (memwritedata),
    .readdata   (memreaddata));

endmodule

module controller(input  [5:0] op, funct,
                  input        zero,
                  output       signext,
                  output       shiftl16,
                  output       memtoreg, memwrite,
                  output       pcsrc, alusrc,
                  output       regdst, regwrite,
                  output       jump, jr, // <- modified line
                  output [3:0] alucontrol); // <- modified line

  wire [1:0] aluop;
  wire       branch;

  maindec md(
    .op       (op),
    .signext  (signext),
    .shiftl16 (shiftl16),
    .memtoreg (memtoreg),
    .memwrite (memwrite),
    .branch   (branch),
    .alusrc   (alusrc),
    .regdst   (regdst),
    .regwrite (regwrite),
    .jump     (jump),
    .aluop    (aluop));

  aludec ad( 
    .funct      (funct),
    .aluop      (aluop), 
    .jr         (jr), // <- modified line
    .alucontrol (alucontrol));

  assign pcsrc = branch & (op[0] ^ zero); // <- modified line

endmodule


module maindec(input  [5:0] op,
               output       signext,
               output       shiftl16,
               output       memtoreg, memwrite,
               output       branch, alusrc,
               output       regdst, regwrite,
               output       jump,
               output [1:0] aluop);

  reg [10:0] controls;

  assign {signext, shiftl16, regwrite, regdst, alusrc, branch, memwrite,
          memtoreg, jump, aluop} = controls;

  always @(*)
    case(op)
      6'b000000: controls <= #`mydelay 11'b00110000011; // Rtype
      6'b100011: controls <= #`mydelay 11'b10101001000; // LW
      6'b101011: controls <= #`mydelay 11'b10001010000; // SW
      6'b000100,
      6'b000101: controls <= #`mydelay 11'b10000100001; // BEQ, BNE, BNEZ (I-Type) <- modified line
      6'b001000,
      6'b001001: controls <= #`mydelay 11'b10101000000; // ADDI, ADDIU: only difference is exception
      6'b001101: controls <= #`mydelay 11'b00101000010; // ORI
      6'b001111: controls <= #`mydelay 11'b01101000000; // LUI
      6'b000010: controls <= #`mydelay 11'b00000000100; // J
      6'b000011: controls <= #`mydelay 11'b00100000100; // JAL (J-Type) <- modified line
      default:   controls <= #`mydelay 11'bxxxxxxxxxxx; // ???
    endcase

endmodule

module aludec(input      [5:0] funct,
              input      [1:0] aluop,
              output reg       jr, // <- modified line
              output reg [3:0] alucontrol); // <- modified line

  always @(*)
    begin // <- modified line
      jr <= #`mydelay 1'b0; // <- modified line
      case(aluop)
        2'b00: alucontrol <= #`mydelay 4'b0010;  // add <- modified line
        2'b01: alucontrol <= #`mydelay 4'b0110;  // sub <- modified line
        2'b10: alucontrol <= #`mydelay 4'b0001;  // or <- modified line
        default: case(funct)          // RTYPE
            6'b100000,
            6'b100001: alucontrol <= #`mydelay 4'b0010; // ADD, ADDU: only difference is exception <- modified line
            6'b100010,
            6'b100011: alucontrol <= #`mydelay 4'b0110; // SUB, SUBU: only difference is exception <- modified line
            6'b100100: alucontrol <= #`mydelay 4'b0000; // AND <- modified line
            6'b100101: alucontrol <= #`mydelay 4'b0001; // OR <- modified line
            6'b101010: alucontrol <= #`mydelay 4'b0111; // SLT <- modified line
            6'b001000:
              begin // <- modified line
                jr <= #`mydelay 1'b1; // <- modified line
                alucontrol <= #`mydelay 4'b1000; // JR (R-Type) <- modified line
              end // <- modified line
            6'b101011: alucontrol <= #`mydelay 4'b1111; // SLTU (R-Type) <- modified line
            default:   alucontrol <= #`mydelay 4'bxxxx; // ??? <- modified line
          endcase
      endcase
    end // <- modified line
    
endmodule

module datapath(input         clk, reset,
                input         signext,
                input         shiftl16,
                input         memtoreg, pcsrc,
                input         alusrc, regdst,
                input         regwrite, jump,
                input         jr, // <- modified line
                input  [3:0]  alucontrol, // <- modified line
                output        zero,
                output [31:0] pc,
                input  [31:0] instr,
                output [31:0] aluout, writedata,
                input  [31:0] readdata);

  wire [4:0]  writereg_candidate, writereg; // <- modified line
  wire [31:0] pc_candidate; // <- modified line
  wire [31:0] pcnext, pcnextbr, pcplus4, pcbranch;
  wire [31:0] signimm, signimmsh, shiftedimm;
  wire [31:0] srca, srcb;
  wire [31:0] result_candidate, result; // <- modified line
  wire        shift;

  // next PC logic
  flopr #(32) pcreg(
    .clk   (clk),
    .reset (reset),
    .d     (pcnext),
    .q     (pc));

  adder pcadd1(
    .a (pc),
    .b (32'b100),
    .y (pcplus4));

  sl2 immsh(
    .a (signimm),
    .y (signimmsh));
				 
  adder pcadd2(
    .a (pcplus4),
    .b (signimmsh),
    .y (pcbranch));

  mux2 #(32) pcbrmux(
    .d0  (pcplus4),
    .d1  (pcbranch),
    .s   (pcsrc),
    .y   (pcnextbr));

  mux2 #(32) pcmux(
    .d0   (pcnextbr),
    .d1   ({pcplus4[31:28], instr[25:0], 2'b00}),
    .s    (jump),
    .y    (pc_candidate)); // <- modified line

  mux2 #(32) jrmux( // <- modified line
    .d0   (pc_candidate), // <- modified line
    .d1   (aluout), // <- modified line
    .s    (jr), // <- modified line
    .y    (pcnext)); // <- modified line

  mux2 #(5) wamux( // <- modified line
    .d0   (writereg_candidate), // <- modified line
    .d1   (5'b11111), // <- modified line
    .s    (jump), // <- modified line
    .y    (writereg)); // <- modified line

  mux2 #32 wdmux( // <- modified line
    .d0   (result_candidate), // <- modified line
    .d1   (pcplus4), // <- modified line
    .s    (jump), // <- modified line
    .y    (result)); // <- modified line

  // register file logic
  regfile rf(
    .clk     (clk),
    .we      (regwrite),
    .ra1     (instr[25:21]),
    .ra2     (instr[20:16]),
    .wa      (writereg), // <- modified line
    .wd      (result), // <- modified line
    .rd1     (srca),
    .rd2     (writedata));

  mux2 #(5) wrmux(
    .d0  (instr[20:16]),
    .d1  (instr[15:11]),
    .s   (regdst),
    .y   (writereg_candidate)); // <- modified line

  mux2 #(32) resmux(
    .d0 (aluout),
    .d1 (readdata),
    .s  (memtoreg),
    .y  (result_candidate)); // <- modified line

  sign_zero_ext sze(
    .a       (instr[15:0]),
    .signext (signext),
    .y       (signimm[31:0]));

  shift_left_16 sl16(
    .a         (signimm[31:0]),
    .shiftl16  (shiftl16),
    .y         (shiftedimm[31:0]));

  // ALU logic
  mux2 #(32) srcbmux(
    .d0 (writedata),
    .d1 (shiftedimm[31:0]),
    .s  (alusrc),
    .y  (srcb));

  alu alu(
    .a       (srca),
    .b       (srcb),
    .alucont (alucontrol),
    .result  (aluout),
    .zero    (zero));
    
endmodule
