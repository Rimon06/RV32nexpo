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
integer i;
initial begin
    // Load in the program/initial memory state into the memory module
    for(i=0;i<256;i=i+1) MEM[i]=0;
    $readmemh("codigo.hex", MEM);
end

// Bloque de sincronizacion de rst asincrono con rst sincrono
always @(posedge clk) begin
  if (rst)
    _RST <= 1;
  else
    _RST <=0;
end

// Lectura de la nueva instrucion
reg  [31:0] MEM [0:255];
//                               addi x0, x0, 0 === nop
//                             imm   rs1  add  rd   ALUREG
reg [31:0] instr = 32'b000000000000_00000_000_00000_0010011;
always @(posedge clk) begin
  // if (_RST)// La instruccion cuando ocurre el rst asincrono. De tal forma que se sincroniza..
  //   instr <= 32'b000000000000_00000_000_00000_0010011;
  // else 
  if (!end_prog) // en un rst sincrono
    instr <= MEM[PC[9:2]];
end

// Lectura del dato externo
always @(negedge clk) begin
  if (_RST)
    mem_in <= 0;
  else if (mem_read) 
    mem_in <= MEM[mem_address[9:2]];
end

reg [31:0] leds=0;
// Escritura del dato del procesador
always @(posedge clk) begin
  if(mem_write) begin
    if(mem_mask[0]) MEM[mem_address[9:2]][ 7:0 ] <= mem_out[ 7:0 ];
    if(mem_mask[1]) MEM[mem_address[9:2]][15:8 ] <= mem_out[15:8 ];
    if(mem_mask[2]) MEM[mem_address[9:2]][23:16] <= mem_out[23:16];
    if(mem_mask[3]) MEM[mem_address[9:2]][31:24] <= mem_out[31:24];
  end
  leds <= MEM[mem_address[9:2]];
end

reg clk=0;
always @(*) #2 clk<=~clk;

initial begin
  $dumpvars(0, RV32nexpo_tb);
  rst=1;
  #3rst=0;
  #1000 $finish;
end

always @(posedge clk) begin
  $display("---------");
  $display("PC=%0d",uut.pc);
  $display("instruction %b_%b_%b_%b_%b_%b",instr[31:25],instr[24:20],instr[19:15],instr[14:12],instr[11:7],instr[6:0]);
  case (1'b1)
    uut.isALU: if (uut.opcode[5])
            $display("ALUreg rd=%d rs1=%d rs2=%d funct3=%b",uut.rd, uut.rs1, uut.rs2, uut.funct3);
          else
            $display("ALUimm rd=%d rs1=%d imm=%0d funct3=%b",uut.rd, uut.rs1, uut.imm, uut.funct3);
    uut.isBRANCH:
            $display("BRANCH rs1=%d rs2=%d funct3=%b branch?=%b",uut.rs1,uut.rs2,uut.funct3,uut.PC_src1);
    uut.isJAL:    
            $display("JAL rd=%d imm=%0d -> Nextpc=%d",uut.rs1,uut.imm,uut.pcNext);
    uut.isJALR: 
            $display("JALR rd=%d rs1=%d imm=%0d -> Nextpc=%d",uut.rd,uut.rs1,uut.pcNext);
    uut.isAUIPC:
            $display("AUIPC rd=%d imm=%0d",uut.rd,uut.imm);
    uut.isLUI:  
            $display("LUI rd=%d imm=%0d",uut.rd,uut.imm);  
    uut.isLOAD:
            $display("LOAD   rd=%d rs1=%d imm=%0d funct3=%b",  uut.rd, uut.rs1, uut.imm, uut.funct3);
    uut.isSTORE:
            $display("STORE  rs2=%d rs1=%d imm=%0d funct3=%b", uut.rs2, uut.rs1, uut.imm, uut.funct3);
    uut.isSYSTEM: begin
           $display("SYSTEM");
           #3 rst=1;
           $display("............................................................");
           #3 rst=0;
           #50 $finish;
           end
  endcase
  $display("Reg(rs1)=%d Reg(rs2)=%d Reg(rd)<-%d",uut.reg1Data,uut.reg2Data,uut.WBData);

  end

endmodule