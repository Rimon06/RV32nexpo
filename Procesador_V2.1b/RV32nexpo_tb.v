`timescale 1ns / 1ps
`define BENCH
module RV32nexpo_tb();

 
wire [31:0] PC,mem_out,mem_address;       // program counter
wire[3:0] mem_mask; 
reg  [31:0] mem_in;    // current instruction
reg rst=1,_RST=1;
wire mem_read, mem_write, error,end_prog;

RV32nexpo uut(
  .rst(_RST),
  .clk(clk),
  .error(error),
  .end_prog(end_prog),
  .InstrData(instr),
  .InstrAddr(PC),
  .mem_in(mem_in), // datos de entrada    
  .mem_out(mem_out), //datos de salida
  .mem_address(mem_address),
  .mem_mask(mem_mask),
  .mem_read(mem_read),
  .mem_write(mem_write)
);
`include "../riscv_disassembly.v"

// Bloque de sincronizacion de rst asincrono con rst sincrono
always @(posedge clk)
  if (rst)
    _RST <= 1;
  else
    _RST <=0;

// Lectura de la nueva instrucion
reg  [31:0] MEM [0:255];
reg  [31:0] RAM [0:255];
integer i;
initial begin
    // Load in the program/initial memory state into the memory module
    for(i=0;i<256;i=i+1) MEM[i]=0;
    for(i=0;i<256;i=i+1) RAM[i]=0;
    $readmemh("codigo.hex", MEM);
end
//                               addi x0, x0, 0 === nop
//                             imm   rs1  add  rd   ALUREG
reg [31:0] instr = 32'b000000000000_00000_000_00000_0010011;
always @(posedge clk) begin
    instr <= MEM[PC[9:2]];
end

// Lectura del dato externo
always @(negedge clk) begin
  if (_RST)
    mem_in <= 0;
  else if (mem_read) 
    mem_in <= RAM[mem_address[9:2]];
end

reg [31:0] leds=0;
// Escritura del dato del procesador
always @(posedge clk) begin
  if(mem_write) begin
    if(mem_mask[0]) RAM[mem_address[9:2]][ 7:0 ] <= mem_out[ 7:0 ];
    if(mem_mask[1]) RAM[mem_address[9:2]][15:8 ] <= mem_out[15:8 ];
    if(mem_mask[2]) RAM[mem_address[9:2]][23:16] <= mem_out[23:16];
    if(mem_mask[3]) RAM[mem_address[9:2]][31:24] <= mem_out[31:24];
  end
  leds <= RAM[mem_address[9:2]];
end

// Reloj
reg clk=0;
always @(*) #2 clk<=~clk;

initial begin
  $dumpvars(0, RV32nexpo_tb);
  rst=1;
  #3rst=0;
  #1000 $finish;
end
wire [31:0] D_instr = uut.FD_instr;

reg [31:0] E_instr,M_instr,W_instr, M_pc,W_pc;
wire JoB = uut.E_JumpOrBranch;
wire [31:0] F_pc = uut.F_pc,
            D_pc = uut.FD_pc,
            E_pc = uut.DE_pc;
wire St_F  = uut.Stall_F,  // Stall
     St_D = uut.Stall_D,
     St_E = uut.Stall_E,
     St_M = uut.Stall_M;
wire Fl_D = uut.Flush_D,   // Flush
     Fl_E = uut.Flush_E,
     Fl_M = uut.Flush_M, 
     Fl_W = uut.Flush_W;

wire Fwd_rs1_M = uut.E_M_fwd_rs1,
     Fwd_rs1_W = uut.E_W_fwd_rs1,
     Fwd_rs2_M = uut.E_M_fwd_rs2,
     Fwd_rs2_W = uut.E_W_fwd_rs2;
always @(posedge clk) begin
  if (!St_E) E_instr <= (Fl_E || uut.FD_nop) ? uut.I_NOP : D_instr;
  if (!St_M) begin
    M_instr <= (Fl_M || uut.DE_nop) ? uut.I_NOP : E_instr;
    M_pc <= E_pc;
  end

  W_instr <= (Fl_W || uut.EM_nop) ? uut.I_NOP : M_instr;
  W_pc <= M_pc;

  $display("* (SF)%10d ------------------------------------------------------------ *",uut.cycle);
    $write( "| (%c ) IF| -pc: %8h ",St_F?"*":" ", F_pc);
    $write("\n");

    $write("| (%c%c) ID| -pc: %8h ",St_D?"*":" ",Fl_D?"*":" ",D_pc);
    $write("[%s%s] ",uut.loadHazard && uut.rs1Hazard?"*":" ",
                     uut.loadHazard && uut.rs2Hazard?"*":" ");
    riscv_disasm(D_instr,D_pc);
    $write("\n");
    
    $write("| (%c%c) Ex| -pc: %8h ",St_E?"*":" ",Fl_E?"*":" ", E_pc);
    if(uut.DE_nop)
        $write("[  ] ");
    else
        $write("[%s%s] ", 
              riscv_disasm_readsRs1(E_instr)   ? 
              (Fwd_rs1_M?"M":Fwd_rs1_W?"W":" ") : " " , 
              riscv_disasm_readsRs2(E_instr)   ? 
              (Fwd_rs2_M?"M":Fwd_rs2_W?"W":" ") : " ");
    riscv_disasm(E_instr,E_pc);
    $write("\n");

    $write("| (%c%c) Me| -pc: %8h      ",St_M?"*":" ",Fl_M?"*":" ", M_pc);
    riscv_disasm(M_instr,M_pc);
    $write("\n");

    $write( "| ( %c) Wb| -pc: %8h      ",Fl_W?"*":" ",W_pc);
    riscv_disasm(W_instr,W_pc);
    $write("\n");
  end
/*always @(posedge clk) begin
  $display("---------");
  $display("PC=%0d",uut.pc);
  $display("instruction %b_%b_%b_%b_%b_%b",instr[31:25],instr[24:20],instr[19:15],instr[14:12],instr[11:7],instr[6:0]);
  case (1'b1)
    uut.D_isALUreg:
            $display("ALUreg rd=%d rs1=%d rs2=%d funct3=%b",uut.rd, uut.rs1, uut.rs2, uut.funct3);
    uut.D_isALUimm:
            $display("ALUimm rd=%d rs1=%d imm=%0d funct3=%b",uut.rd, uut.rs1, uut.imm, uut.funct3);
    uut.D_isBRANCH:
            $display("BRANCH rs1=%d rs2=%d funct3=%b branch?=%b",uut.rs1,uut.rs2,uut.funct3,uut.PC_src1);
    uut.D_isJAL:    
            $display("JAL rd=%d imm=%0d -> Nextpc=%d",uut.rs1,uut.imm,uut.pcNext);
    uut.D_isJALR: 
            $display("JALR rd=%d rs1=%d imm=%0d -> Nextpc=%d",uut.rd,uut.rs1,uut.pcNext);
    uut.D_isAUIPC:
            $display("AUIPC rd=%d imm=%0d",uut.rd,uut.imm);
    uut.D_isLUI:  
            $display("LUI rd=%d imm=%0d",uut.rd,uut.imm);  
    uut.D_isLOAD:
            $display("LOAD   rd=%d rs1=%d imm=%0d funct3=%b",  uut.rd, uut.rs1, uut.imm, uut.funct3);
    uut.D_isSTORE:
            $display("STORE  rs2=%d rs1=%d imm=%0d funct3=%b", uut.rs2, uut.rs1, uut.imm, uut.funct3);
    uut.D_isEBREAK: begin
           $display("SYSTEM");
           #3 rst=1;
           $display("............................................................");
           #3 rst=0;
           #50 $finish;
           end
  endcase
  $display("Reg(rs1)=%d Reg(rs2)=%d Reg(rd)<-%d",uut.reg1Data,uut.reg2Data,uut.WBData);

  end*/

endmodule