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

  wire        signext, shiftl16, memtoreg; // <- Modified Line
  wire        pcsrc, zero;
  wire        alusrc, regdst, regwrite, jump, jr;
  wire [3:0]  alucontrol;
  // ###### Modified Lines (Start) ######
  wire [31:0] ID_instr;
  wire        MEM_regwrite, WB_regwrite, EX_memread;
  wire [4:0]  ID_rs, ID_rt, EX_rs, EX_rt, MEM_rt;
  wire [4:0]  MEM_writereg_candidate, WB_writereg_candidate;
  wire [2:0]  forwarding_a, forwarding_b;
  wire        stall;
  // ###### Modified Lines (End) ######

  // Instantiate Controller
  controller c(
    // ###### Modified Lines (Start) ######
    .clk          (clk),
    .reset        (reset),
    .op           (ID_instr[31:26]), 
    .funct        (ID_instr[5:0]),
    .memread      (memread),
    .MEM_regwrite (MEM_regwrite),
    .WB_regwrite  (WB_regwrite),
    .EX_memread   (EX_memread),
    .stall        (stall),
    // ###### Modified Lines (End) ######
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
    .jr         (jr),
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
    .jr         (jr),
    .alucontrol (alucontrol),
    .zero       (zero),
    .pc         (pc),
    .instr      (instr),
    .aluout     (memaddr), 
    .writedata  (memwritedata),
    .readdata   (memreaddata),
    // ###### Modified Lines (Start) ######
    .ID_instr   (ID_instr),
    .ID_rs      (ID_rs),
    .ID_rt      (ID_rt),
    .EX_rs      (EX_rs),
    .EX_rt      (EX_rt),
    .MEM_rt     (MEM_rt),
    .MEM_writereg_candidate (MEM_writereg_candidate),
    .WB_writereg_candidate  (WB_writereg_candidate),
    .forwarding_a           (forwarding_a),
    .forwarding_b           (forwarding_b),
    .stall      (stall));
    // ###### Modified Lines (End) ######

  // ###### Modified Lines (Start) ######
  forwarding_unit fdu (
    .MEM_regwrite           (MEM_regwrite),
    .WB_regwrite            (WB_regwrite),
    .ID_rs                  (ID_rs),
    .ID_rt                  (ID_rt),
    .EX_rs                  (EX_rs),
    .EX_rt                  (EX_rt),
    .MEM_writereg_candidate (MEM_writereg_candidate),
    .WB_writereg_candidate  (WB_writereg_candidate),
    .forwarding_a           (forwarding_a),
    .forwarding_b           (forwarding_b));

  hazard_detector hdu (
    .reset       (reset),
    .EX_memread  (EX_memread),
    .MEM_memread (memread),
    .MEM_rt      (MEM_rt),
    .ID_rs       (ID_rs),
    .ID_rt       (ID_rt),
    .EX_rt       (EX_rt),
    .stall       (stall));
  // ###### Modified Lines (End) ######
endmodule

module controller(input  [5:0] op, funct,
                  input        zero,
                  // ###### Modified Lines (Start) ######
                  input        clk, reset, stall,
                  output       memread,
                  output       EX_memread, MEM_regwrite, WB_regwrite,
                  // ###### Modified Lines (End) ######
                  output       signext,
                  output       shiftl16,
                  output       memtoreg, memwrite,
                  output       pcsrc, alusrc,
                  output       regdst, regwrite,
                  output       jump, jr,
                  output [3:0] alucontrol);

  wire       branch;

  // ###### Modified Lines (Start) ######
  wire [11:0] ID_control_candidate;
  wire [11:0] ID_control;
  wire        ID_regdst;
  wire [2:0]  ID_aluop;
  wire        ID_alusrc;
  wire        ID_shiftl16;
  wire        ID_branch;
  wire        ID_ifbne;
  wire        ID_memread;
  wire        ID_memwrite;
  wire        ID_memtoreg;
  wire        ID_regwrite;
  wire        EX_regdst;
  wire [2:0]  EX_aluop;
  wire        EX_alusrc;
  wire        EX_shiftl16;
  wire        EX_jr;
  wire [3:0]  EX_MEM;
  wire [1:0]  EX_WB;
  wire [5:0]  EX_funct;
  wire        MEM_branch;
  wire        MEM_ifbne;
  wire        MEM_memread;
  wire        MEM_memwrite;
  wire        MEM_zero;
  wire        MEM_jr;
  wire [1:0]  MEM_WB;
  wire        WB_memtoreg;
  // ###### Modified Lines (End) ######

  // ###### Modified Lines (Start) ######
  assign ID_control_candidate = {ID_regdst, ID_aluop, ID_alusrc, ID_shiftl16, ID_branch, ID_ifbne, ID_memread, ID_memwrite, ID_memtoreg, ID_regwrite};
  assign EX_memread = EX_MEM[1];
  assign MEM_regwrite = MEM_WB[0];
  assign regdst = EX_regdst;
  assign alusrc = EX_alusrc;
  assign shiftl16 = EX_shiftl16;
  assign memread = MEM_memread;
  assign memwrite = MEM_memwrite;
  assign jr = MEM_jr;
  assign memtoreg = WB_memtoreg;
  assign regwrite = WB_regwrite;
  // ###### Modified Lines (End) ######

  maindec md(
    .op       (op),
    .signext  (signext),
    .shiftl16 (ID_shiftl16), // <- Modified Line
    .memtoreg (ID_memtoreg), // <- Modified Line
    .memread  (ID_memread), // <- Modified Line
    .memwrite (ID_memwrite), // <- Modified Line
    .branch   (ID_branch), // <- Modified Line
    .ifbne    (ID_ifbne), // <- Modified Line
    .alusrc   (ID_alusrc), // <- Modified Line
    .regdst   (ID_regdst), // <- Modified Line
    .regwrite (ID_regwrite), // <- Modified Line
    .jump     (jump),
    .aluop    (ID_aluop)); // <- Modified Line

  aludec ad( 
    .funct      (EX_funct), // <- Modified Line
    .aluop      (EX_aluop), // <- Modified Line 
    .jr         (EX_jr), // <- Modified Line
    .alucontrol (alucontrol));

  assign pcsrc = MEM_branch & (MEM_ifbne ^ MEM_zero);

  // ###### Modified Lines (Start) ######
  mux2 #(12) stallmux (    
    .d0 (12'b0),
    .d1 (ID_control_candidate),    
    .s  (~stall),    
    .y  (ID_control)
  );    
  flopr #(18) ID_EX_control_ff (   
    .clk     (clk),
    .reset   (reset),   
    .d       ({ID_control, funct}),   
    .q       ({EX_regdst, EX_aluop, EX_alusrc, EX_shiftl16, EX_MEM, EX_WB, EX_funct})   
  );    
  flopr #(8) EX_MEM_control_ff (   
    .clk     (clk),   
    .reset   (reset),   
    .d       ({EX_MEM, EX_WB, zero, EX_jr}),
    .q       ({MEM_branch, MEM_ifbne, MEM_memread, MEM_memwrite, MEM_WB, MEM_zero, MEM_jr})    
  );    
  flopr #(2) MEM_WB_control_ff (   
    .clk     (clk),   
    .reset   (reset),   
    .d       (MEM_WB),    
    .q       ({WB_memtoreg, WB_regwrite})   
  );
  // ###### Modified Lines (End) ######
endmodule


module maindec(input  [5:0] op,
               output       signext,
               output       shiftl16,
               output       memtoreg, memwrite,
               output       branch, alusrc,
               output       regdst, regwrite,
               output       jump,
               output [1:0] aluop,
               output       memread, ifbne); // <- Modified Line

  reg [12:0] controls; // <- Modified Line

  assign {signext, shiftl16, regwrite, regdst, alusrc, branch, memwrite,
          memtoreg, jump, aluop, ifbne, memread} = controls; // <- Modified Line

  always @(*)
    case(op)
      // ###### Modified Lines (Start) ######
      6'b000000: controls <= #`mydelay 13'b0011000001100; // Rtype
      6'b100011: controls <= #`mydelay 13'b1010100100001; // LW
      6'b101011: controls <= #`mydelay 13'b1000101000000; // SW
      6'b000100: controls <= #`mydelay 13'b1000010000100; // BEQ
      6'b001000, 
      6'b001001: controls <= #`mydelay 13'b1010100000000; // ADDI, ADDIU: only difference is exception
      6'b001101: controls <= #`mydelay 13'b0010100001000; // ORI
      6'b001111: controls <= #`mydelay 13'b0110100000000; // LUI
      6'b000010: controls <= #`mydelay 13'b0000000010000; // J
      6'b000011: controls <= #`mydelay 13'b0010000010000; // JAL (J-Type)
      6'b000101: controls <= #`mydelay 13'b1000010000110; // BNE, BNEZ (I-Type)
      default:   controls <= #`mydelay 13'bxxxxxxxxxxxxx; // ???
      // ###### Modified Lines (End) ######
    endcase

endmodule

module aludec(input      [5:0] funct,
              input      [1:0] aluop,
              output reg       jr,
              output reg [3:0] alucontrol);

  always @(*)
    begin
      jr <= #`mydelay 1'b0;
      case(aluop)
        2'b00: alucontrol <= #`mydelay 4'b0010;  // add
        2'b01: alucontrol <= #`mydelay 4'b0110;  // sub
        2'b10: alucontrol <= #`mydelay 4'b0001;  // or
        default: case(funct)          // RTYPE
            6'b100000,
            6'b100001: alucontrol <= #`mydelay 4'b0010; // ADD, ADDU: only difference is exception
            6'b100010,
            6'b100011: alucontrol <= #`mydelay 4'b0110; // SUB, SUBU: only difference is exception
            6'b100100: alucontrol <= #`mydelay 4'b0000; // AND
            6'b100101: alucontrol <= #`mydelay 4'b0001; // OR
            6'b101010: alucontrol <= #`mydelay 4'b0111; // SLT
            6'b001000:
              begin
                jr <= #`mydelay 1'b1;
                alucontrol <= #`mydelay 4'b1000; // JR (R-Type)
              end
            6'b101011: alucontrol <= #`mydelay 4'b1111; // SLTU (R-Type)
            default:   alucontrol <= #`mydelay 4'bxxxx; // ???
          endcase
      endcase
    end
endmodule

module datapath(input         clk, reset,
                input         signext,
                input         shiftl16,
                input         memtoreg, pcsrc,
                input         alusrc, regdst,
                input         regwrite, jump,
                input         jr,
                input  [3:0]  alucontrol,
                output        zero,
                output [31:0] pc,
                input  [31:0] instr,
                output [31:0] aluout, writedata,
                input  [31:0] readdata,
                // ###### Modified Lines (Start) ######
                output [31:0] ID_instr,
                output [4:0]  ID_rs, ID_rt, EX_rs, EX_rt, MEM_rt,
                output [4:0]  MEM_writereg_candidate, WB_writereg_candidate,
                input  [2:0]  forwarding_a, forwarding_b,
                input         stall);
                // ###### Modified Lines (End) ######

  wire [4:0]  ID_rd, EX_rd, EX_writereg_candidate, writereg; // <- Modified Line
  wire [31:0] pc_candidate;
  wire [31:0] pcnext, pcnextbr, pcplus4, pcbranch;

  // ###### Modified Lines (Start) ######
  wire [31:0] ID_signext_imm, ID_writedata, ID_srca;
  wire [31:0] EX_srca, EX_srcb, EX_writedata, EX_aluout;
  wire [31:0] EX_signext_imm, EX_signext_immsh, EX_shiftedimm;
  wire [31:0] MEM_aluout, MEM_writedata;
  wire [31:0] WB_readdata, WB_aluout;
  wire [31:0] ID_forwarding_a, ID_forwarding_b;
  wire [31:0] forwarding_a_1, forwarding_a_0;
  wire [31:0] forwarding_b_1, forwarding_b_0;
  // ###### Modified Lines (End) ######

  wire [31:0] result_candidate, result;
  wire        shift;

  // ###### Modified Lines (Start) ######
  assign ID_rs = ID_instr[25:21];
  assign ID_rt = ID_instr[20:16];
  assign ID_rd = ID_instr[15:11];
  assign aluout = MEM_aluout;
  assign writedata = MEM_writedata;
  // ###### Modified Lines (End) ######

  // next PC logic
  flopenr #(32) pcreg( // <- Modified Line
    .clk   (clk),
    .reset (reset),
    .en    (~stall), // <- Modified Line
    .d     (pcnext),
    .q     (pc));

  adder pcadd1(
    .a (pc),
    .b (32'b100),
    .y (pcplus4));

  sl2 immsh(
    .a (EX_signext_imm), // <- Modified Line
    .y (EX_signext_immsh)); // <- Modified Line
         
  adder pcadd2(
    .a (pcplus4),
    .b (EX_signext_immsh), // <- Modified Line
    .y (pcbranch));

  mux2 #(32) pcbrmux(
    .d0  (pcplus4),
    .d1  (pcbranch),
    .s   (pcsrc),
    .y   (pcnextbr));

  mux2 #(32) pcmux(
    .d0   (pcnextbr),
    .d1   ({pcplus4[31:28], ID_instr[25:0], 2'b00}), // <- Modified Line
    .s    (jump),
    .y    (pc_candidate));

  mux2 #(32) jrmux(
    .d0   (pc_candidate),
    .d1   (MEM_aluout), // <- Modified Line
    .s    (jr),
    .y    (pcnext));

  mux2 #(5) wamux(
    .d0   (WB_writereg_candidate), // <- Modified Line
    .d1   (5'b11111),
    .s    (jump),
    .y    (writereg));

  mux2 #(32) wdmux(
    .d0   (result_candidate),
    .d1   (pcplus4),
    .s    (jump),
    .y    (result));

  // register file logic
  regfile rf(
    .clk     (clk),
    .we      (regwrite),
    .ra1     (ID_rs), // <- Modified Line
    .ra2     (ID_rt), // <- Modified Line
    .wa      (writereg),
    .wd      (result),
    .rd1     (ID_srca), // <- Modified Line
    .rd2     (ID_writedata)); // <- Modified Line

  mux2 #(5) wrmux(
    .d0  (EX_rt), // <- Modified Line
    .d1  (EX_rd), // <- Modified Line
    .s   (regdst),
    .y   (EX_writereg_candidate)); // <- Modified Line

  mux2 #(32) resmux(
    .d0 (WB_aluout), // <- Modified Line
    .d1 (WB_readdata), // <- Modified Line
    .s  (memtoreg),
    .y  (result_candidate));

  sign_zero_ext sze(
    .a       (ID_instr[15:0]), // <- Modified Line
    .signext (signext),
    .y       (ID_signext_imm[31:0])); // <- Modified Line

  shift_left_16 sl16(
    .a         (EX_signext_imm[31:0]), // <- Modified Line
    .shiftl16  (shiftl16),
    .y         (EX_shiftedimm[31:0])); // <- Modified Line

  // ALU logic
  mux2 #(32) srcbmux(
    .d0 (forwarding_b_0), // <- Modified Line
    .d1 (EX_shiftedimm[31:0]), // <- Modified Line
    .s  (alusrc),
    .y  (EX_srcb)); // <- Modified Line

  alu alu(
    .a       (forwarding_a_0), // <- Modified Line
    .b       (EX_srcb), // <- Modified Line
    .alucont (alucontrol),
    .result  (EX_aluout), // <- Modified Line
    .zero    (zero));
    
  // ###### Modified Lines (Start) ######
  // Pipeline Logic
  flopenr #(32) IF_ID_ff (
    .clk    (clk),
    .reset  (reset),
    .en     (~stall),
    .d      ({instr}),
    .q      ({ID_instr})
  );

  flopr #(111) ID_EX_ff (
    .clk    (clk),
    .reset  (reset),
    .d      ({ID_forwarding_a, ID_forwarding_b, ID_signext_imm, ID_rs, ID_rt, ID_rd}),
    .q      ({EX_srca, EX_writedata, EX_signext_imm, EX_rs, EX_rt, EX_rd})
  );
  
  flopr #(74) EX_MEM_ff (
    .clk    (clk),
    .reset  (reset),
    .d      ({EX_aluout, EX_writedata, EX_writereg_candidate, EX_rt}),
    .q      ({MEM_aluout, MEM_writedata, MEM_writereg_candidate, MEM_rt})
  );

  flopr #(69) MEM_WB_ff (
    .clk    (clk),
    .reset  (reset),
    .d      ({readdata, MEM_aluout, MEM_writereg_candidate}),
    .q      ({WB_readdata, WB_aluout, WB_writereg_candidate})
  );

  mux2 #(32) ID_WB_forwarding_a (
    .d0 (ID_srca),
    .d1 (result_candidate),
    .s  (forwarding_a[2]),
    .y  (ID_forwarding_a)
  );

  mux2 #(32) ID_WB_forwarding_b (
    .d0 (ID_writedata),
    .d1 (result_candidate),
    .s  (forwarding_b[2]),
    .y  (ID_forwarding_b)
  );

  mux2 #(32) EX_MEM_forwarding_a (
    .d0 (EX_srca),
    .d1 (MEM_aluout),
    .s  (forwarding_a[1]),
    .y  (forwarding_a_1)
  );

  mux2 #(32) EX_MEM_forwarding_b (
    .d0 (EX_writedata),
    .d1 (MEM_aluout),
    .s  (forwarding_b[1]),
    .y  (forwarding_b_1)
  );
  
  mux2 #(32) EX_WB_forwarding_a (
    .d0 (forwarding_a_1),
    .d1 (result_candidate),
    .s  (forwarding_a[0]),
    .y  (forwarding_a_0)
  );

  mux2 #(32) EX_WB_forwarding_b (
    .d0 (forwarding_b_1),
    .d1 (result_candidate),
    .s  (forwarding_b[0]),
    .y  (forwarding_b_0)
  );
  // ###### Modified Lines (End) ######
endmodule

// ###### Modified Lines (Start) ######
module forwarding_unit(input            MEM_regwrite, WB_regwrite,
                       input      [4:0] ID_rs, ID_rt, EX_rs, EX_rt,
                       input      [4:0] MEM_writereg_candidate, WB_writereg_candidate,
                       output reg [2:0] forwarding_a,
                       output reg [2:0] forwarding_b);

  always @(*) 
    begin                 
      if (WB_regwrite & (WB_writereg_candidate == ID_rs) & (ID_rs != 0)) forwarding_a[2] = 1'b1;
      else forwarding_a[2] = 1'b0;
      if (MEM_regwrite & (MEM_writereg_candidate == EX_rs) & (EX_rs != 0)) forwarding_a[1:0] = 2'b10;
      else if (WB_regwrite & (WB_writereg_candidate == EX_rs) & (EX_rs != 0)) forwarding_a[1:0] = 2'b01;
      else forwarding_a[1:0] = 2'b00;
      if (WB_regwrite & (WB_writereg_candidate == ID_rt) & (ID_rt != 0)) forwarding_b[2] = 1'b1;
      else forwarding_b[2] = 1'b0;
      if (MEM_regwrite & (MEM_writereg_candidate == EX_rt) & (EX_rt != 0)) forwarding_b[1:0] = 2'b10;
      else if (WB_regwrite & (WB_writereg_candidate == EX_rt) & (EX_rt != 0)) forwarding_b[1:0] = 2'b01;
      else forwarding_b[1:0] = 2'b00;
    end
endmodule

module hazard_detector (input       reset,
                        input       EX_memread, MEM_memread,
                        input [4:0] MEM_rt, EX_rt, ID_rs, ID_rt,
                        output reg  stall);

  always @(*)
    begin
      if (reset) stall = 1'b0;
      else if (EX_memread & ((EX_rt == ID_rs) | (EX_rt == ID_rt))) stall = 1'b1;
      else if (MEM_memread & ((MEM_rt == ID_rs) | (MEM_rt == ID_rt))) stall = 1'b1;
      else stall = 1'b0;
    end
endmodule
// ###### Modified Lines (End) ######