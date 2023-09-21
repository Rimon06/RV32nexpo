//RV32nexpo.v
// V0.7: monociclo sin depurador
// 
// * Instruccion fence es un nop!
// * ebreak está implementado
// * ecall no está implementado ...
// RV32I
//`include "../registros_32.v"
//`include "../ALU.v"
//`include "../interfaz_mem.v"
//------RV32nexpo()----------------------------------------------------------------------
module RV32nexpo (
  input rst,
  input clk,
  output reg error, // entrega 1 si hay error
  output reg end_prog, // entrega 1 si termina la ejecucion (con ebreak)
  // Bus de memoria de instrucciones
  input [31:0] instr, 
  output reg [31:0] pc,
  // bus de memoria de datos
  //    bus de datos:
  input [31:0] mem_in, // datos de entrada    
  output [31:0] mem_out, //datos de salida
  //    bus de direcciones
  output [31:0] mem_address,
  //    bus de control
  output [3:0] mem_mask, // mascara del byte a escribir
  output mem_read,// señal de lectura de memoria
  output mem_write // señal de escritura de la memoria
); //------------------------------------------------------------------------------------

  // * Declaracion de las señales de datos
  wire [31:0] pcNext, pcSuma, pcMas4, pcMasImm; // Señales relacionadas a PC
  
  wire [31:0] reg1Data, reg2Data, WBData;// señales de datos para registros
  // 
  wire [31:0] ALUin1,ALUin2,ALUout; // Señales de entrada y salida de la ALU
  wire ALUbranch;
  // señales decodificadas directamente de la instruccion
  wire [6:0] opcode; 
  wire [4:0] rd, rs1, rs2;
  wire [2:0] funct3;
  wire [6:0] funct7;
  reg [31:0] imm; // inmediato


  // * Declaracion de las señales del control principal
  reg loadpcNext, // seleccion y control de PC
      isALU,isLOAD,isSTORE,isAUIPC,isLUI,isJAL,isJALR,isBRANCH,isSYSTEM,isFENCE, // identificacion de tipo de instruccion
      RegWrite, // habilitacion de escritura del registro
      ALUSrc1,ALUSrc2, // selector de entradas de ALU
      PcPlus4toReg,MemtoReg, // selector de registro de escritura (WBData)
      funQual // Indica cuando hay una operacion alternativa a la usual (tal como resta y desplazamiento a la derecha aritmetico)
      ;
  wire [3:0] ALUCtrl;
  wire PC_src1, PC_src2;

  // *** operacion del softcore ***
  
  // *-1-* Fetch *-1-*
  
  // --- Seleccion de PC (Registro que guarda el PC actual)
  initial pc=0;
  always @(posedge clk) // control: {loadpcNext,rst}
    if (rst)
      pc<=0;
    else if (loadpcNext)
      pc<=pcNext;

  assign pcMas4 = pc+4;

  // *-2-* Decode *-2-*

  // --- Decodificacion directa de la instruccion
  assign opcode = instr[6:0];
  assign rd     = instr[11:7];
  assign funct3 = instr[14:12];
  assign funct7 = instr[31:25];
  assign rs1    = instr[19:15]  & {5{!isLUI}}; // Si es LUI: se fuerza al registro que lea x0
  assign rs2    = instr[24:20];

  /// --- Control principal:!!!!!!!!!!!!!!!!!1
  always @(*) begin
    // caso usual si no aparece en un opcode
    end_prog=0;
    error=0;
    loadpcNext=1; // 0: detiene la ejecución del procesador; 1: carga el valor de pcNext a ActPC
    isALU=0; // 1: instruccion tipo R o ALUimm. La operacion de la ALU depende de la instruccion
    isLOAD=0; isSTORE=0; isAUIPC=0; isLUI=0; isJAL=0; isJALR=0; isBRANCH=0; isSYSTEM=0; isFENCE=0;
    RegWrite=0; // 0: no escribe en rd; 1: escribe
    ALUSrc1=0; // 0: reg(rs1); 1: ActPC
    ALUSrc2=0; // 0: reg(rs2); 1: Imm
    PcPlus4toReg=0; //1: reg(rd)=ActPC+4 0: = (depende de MemtoReg)
    MemtoReg=0; // 0: reg(rd)=ALUout; 1: = read_data;
    funQual=0; // 0: op usual; 1: op alternativa (para sub y sra)

    case(opcode)
      7'b00_000_11: begin //load
        isLOAD=1;   // Lee de la memoria
        RegWrite=1; // se guarda valor en Reg(rd)
        ALUSrc2=1;  // ALUin2=imm
        MemtoReg=1; // Reg(rd)=ReadData
      end
      7'b00_011_11: begin //fence
        isFENCE=1;
      end
      7'b00_100_11: begin //ALUimm
        isALU=1;  // operacion con ALU
        RegWrite=1; // se guarda valor en Reg(rd)
        ALUSrc2=1; // ALUin2=imm
        funQual = (funct3[1:0] == 2'b01)? funct7[5] : 1'b0; // En estas isntrucciones, solo se presta atencion a este cualificador cuando funct3[1:0] indica desplazamientos.
      end
      7'b00_101_11: begin //AUIPC
        isAUIPC=1; // Se lee PC
        ALUSrc1=1; // ALUin1=pc
        ALUSrc2=1; // ALUin2=imm
        RegWrite=1; // se guarda valor en Reg(rd)
        // se guarda ALUout =pc+imm en Reg(rd)
      end
      7'b01_000_11: begin //store
        isSTORE=1;   // Escribe en la memoria
        ALUSrc2=1;  // ALUin2=imm
        MemtoReg=1'bx;    // \ No importa, 
        PcPlus4toReg=1'bx;// /  no se escriben los registros
      end
      7'b01_100_11: begin //ALUreg
        isALU=1;  // operacion variada de ALU
        RegWrite=1; // se guarda valor en Reg(rd)
        funQual=funct7[5];
      end
      7'b01_101_11: begin //LUI
        isLUI=1; // se escribe imm=UImm
        ALUSrc2=1;  // ALUin2=imm
        RegWrite=1; // se guarda valor en Reg(rd)
      end
      7'b11_000_11: begin //branch
        isBRANCH=1;
        // lee los dos registros
        MemtoReg=1'bx;    // \ No importa, 
        PcPlus4toReg=1'bx;// /  no se escriben los registros
      end
      7'b11_001_11: begin//jalr
        isJALR=1;
        // ALUin1=reg
        ALUSrc2=1; // ALUin2=imm
        MemtoReg=1'bx; // No importa, se guardara pc+4
        PcPlus4toReg=1;// reg(rd)=pc+4;
        RegWrite=1; // se guarda valor en Reg(rd)
      end
      7'b11_011_11: begin //jal
        isJAL=1;
        ALUSrc1=1; // ALUin1=pc
        ALUSrc2=1; // ALUin2=imm
        MemtoReg=1'bx; // No importa, se guardara pc+4
        PcPlus4toReg=1;// reg(rd)=pc+4;
        RegWrite=1; // se guarda valor en Reg(rd)
      end
      7'b11_100_11: begin //Sistema
        isSYSTEM=1;
        if (instr[20]==1) begin // ebreak
          loadpcNext=0;
          end_prog=1;
        end // else ecall (no implementado)
      end

      default: begin // error!
        error=1;
        loadpcNext=0;
      end
    endcase
  end

  // --- Decodificacion del inmediato segun formatos 
  wire [31:0] Iimm = {{21{instr[31]}}, instr[30:20]};
  wire [31:0] Simm = {{21{instr[31]}}, instr[30:25], instr[11:7]};
  wire [31:0] Bimm = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
  wire [31:0] Jimm = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};   
  wire [31:0] Uimm = {instr[31], instr[30:12], {12{1'b0}}};
  // La seleccion del inmediato viene a continuación ...

  // - Seleccion de inmediato
  always @(*) begin // Consume...
    case(opcode)
      7'b00_000_11: //load
        imm = Iimm;
      7'b00_100_11: //ALUimm
        imm = Iimm;
      7'b00_101_11: //AUIPC
        imm = Uimm;
      7'b01_000_11: //store
        imm = Simm;
      7'b01_101_11: //LUI
        imm = Uimm;
      7'b11_000_11: //branch
        imm = Bimm;
      7'b11_001_11: //jalr
        imm = Iimm;
      7'b11_011_11: //jal
        imm = Jimm;
      7'b11_100_11: //Sistema
        imm = Iimm;
      default: // Caso: ALUreg y fence
        imm = 32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx;
    endcase
  end
  
  // --- 32 Registros
  registros reg32( // control: RegWrite
    .clk(clk),
    .wr_ena(RegWrite),
    .readReg1(rs1),
    .readReg2(rs2),
    .reg1(reg1Data),
    .reg2(reg2Data),
    .wr_reg(rd),
    .wr_data(WBData) );

  // *-3-* Execute *-3-*
  // Control de la ALU, y del ALUSrc1,2

  // --- Seleccion de entradas
  assign ALUin1 = (ALUSrc1)?  pc : reg1Data;
  assign ALUin2 = (ALUSrc2)? imm : reg2Data;
  
  // --- Control principal de ALU
  assign ALUCtrl = ((isALU||isBRANCH))?  
                    {funQual,funct3}:
                    4'b0000;

  // --- Bloque ALU
  ALU alu( 
    .ALUCtrl(ALUCtrl),
    .in1(ALUin1),
    .in2(ALUin2),
    .ALUout(ALUout),
    .branch(ALUbranch));

  // *-4-* Memoria *-4-*
  // control: isSTORE(MemWrite), isLOAD(MemRead), funct3={unsigned,data_width}
  
  // Estos cables son redundantes, pero son los nombres que posee el bloque de memoria en Patterson & Hennesy(2019)
  wire [31:0] address = ALUout;
  wire [31:0] read_data;
  wire [31:0] write_data = reg2Data;
  wire MemWrite = isSTORE;

  // --- Bloque de ajuste del dato proveniente de la memoria.
  LoadfromMEM load ( // control funct3 y address[1:0]
    .addr_LSB(address[1:0]), // solo 2 bits menos significativos...
    .data_width(funct3[1:0]),
    .sin_signo(funct3[2]),
    .data_mem(mem_in), // Entrada de datos
    .data_readed(read_data) ); // lectura final

  // --- Bloque de ajuste del dato que va a la memoria
  StoretoMEM store (
    .addr_LSB(address[1:0]), // igualmente al bloque load
    .memWrite(MemWrite), //
    .data_width(funct3[1:0]),
    .data(write_data), // Entrada de datos
    .wr_datatoMem(mem_out), // señales de escritura
    .mask_data(mem_mask));

  // --- Asignacion de los puertos I/O de RV32nexpo
  assign mem_address = {address[31:2],2'b00}; // siempre buscará en memoria de manera alineada
  assign mem_read = isLOAD; 
  assign mem_write = MemWrite;
  // mem_out se asigno en store

  // *-5-* WriteBack *-5-*
  // Control del dato de retorno (PcPlus4ToReg y MemtoReg) y del pc siguiente (PC_src1,2)

  // --- Bloque selector del dato para WBreg
  assign WBData = (PcPlus4toReg? pcMas4 :
                  (    MemtoReg? read_data:
                                 ALUout));

  // --- Control de seleccion de pcNext (basado en control principal)
  assign PC_src1 = isBRANCH && ALUbranch; // 0: ActPC+4; 1: ActPC+imm;
  assign PC_src2 = isJAL || isJALR; // 0: (ActPC+4 o ActPC+imm); 1: ALUout;

  // --- Suma de ActPC con inmediato
  assign pcMasImm = pc+imm;

  // --- Seleccion del pcNext
  assign pcSuma = (PC_src1)?(pcMasImm):pcMas4; 
  assign pcNext = (PC_src2)?(ALUout):pcSuma;  

endmodule