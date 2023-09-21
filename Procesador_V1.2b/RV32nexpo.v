//RV32nexpo.v
// V1.0b: monociclo con pre-fetch. Penalty cuando ocurre salto. Ciclo extra para load
// 
// * Instruccion fence es un nop!
// * ebreak detiene el programa implementado
// * ecall no está implementado ...
// * implementado ZiCSR con los CSR del timer implementados... Custom CSR por observar! (detallar cuando deputador)
// RV32I

//------RV32nexpo()----------------------------------------------------------------------
module RV32nexpo (
  input rst,
  input clk,
  output reg error, // entrega 1 si hay error
  output reg end_prog, // entrega 1 si termina la ejecucion (con ebreak)
  // Bus de memoria de instrucciones
  input [31:0] InstrData, 
  output [31:0] InstrAddr,
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
  wire [31:0] pcNext, pcJump, pcMas4, pcMasImm; // Señales relacionadas a PC
  reg [31:0] pc, pcFetch; // pc actual. Valor actual de la instrucciones...

  wire [31:0] reg1Data, reg2Data, WBData;// señales de datos para registros
  wire [31:0] ALUin1,ALUin2,ALUout; // Señales de entrada y salida de la ALU
  wire ALUbranch;

  // señales decodificadas directamente de la instruccion
  wire  [6:0] opcode; 
  wire  [4:0] rd, rs1, rs2;
  wire  [2:0] funct3;
  wire  [6:0] funct7;
  reg  [31:0] imm; // inmediato
  wire [11:0] csr;

  // * Declaracion de estados
  localparam _START = 0;
  localparam _RUN = 1;
  localparam _ERROR = 2;
  localparam _WAIT = 3;
  localparam _DEBUG = 4;
  reg [2:0] state, nextState;


  // * Declaracion de las señales del control principal
  reg loadpcNext, // seleccion y control de PC
      RegWrite, // Hab. de escritura del registro
      ALUSrc1,ALUSrc2, // selector de entradas de ALU
      PcPlus4toReg,MemtoReg, // selectores del reg de escritura (WBData)
      funQual, // Op. alternativa a SRL o + (SRA o -)
      CSRen, CSRRead,CSRWrite, // Hab. de CSR
      PC_src1, PC_src2; 
  wire isALUreg,isALUimm,isLOAD,isSTORE,isAUIPC,isLUI,isJAL,isJALR,isBRANCH,isSYSTEM,isFENCE;

  wire [3:0] ALUCtrl;
  // * Declaracion de señales de control para el funcionamiento del prefetch y de la lectura síncrona...

  reg flushEN=1;
  wire jump;
  // #################*** operacion del softcore ***#####################
  
  // *-1-* Fetch *-1-*
  
  // --- Seleccion de PC (Registro que guarda el PC actual)
  initial pc=0;
  initial pcFetch = 0;
  
  always @(posedge clk) // control: {loadpcNext,rst}
    if (rst) begin
      pc <= 0;
      pcFetch <= 0;
    end else if (loadpcNext) begin
      pc<=pcFetch;
      pcFetch<=pcNext; // El problema aqui es que no es pcNext sino pcFetch.......
    end
  assign InstrAddr = (loadpcNext) ? pcFetch : pc;

  assign pcMas4 = pcFetch+4;
  
  // --- generacion de siguiente instruccion. Si flushEN, entonces nop
  wire [31:0] instr = flushEN ? 32'h0000_0013 : InstrData;

  // *-2-* Decode *-2-*
  
  // --- Decodificacion directa de la instruccion
  assign opcode = instr[6:0];
  assign rd     = instr[11:7];
  assign funct3 = instr[14:12];
  assign rs1    = instr[19:15]; 
  assign rs2    = instr[24:20];
  assign funct7 = instr[31:25];
  assign csr    = instr[31:20];

  // --- Maquina de estado
  always @(posedge clk)
    if (rst)
      state <= _START;
    else
      state <= nextState;
  always @(posedge clk)
    if (rst)
      flushEN<=1;
    else
      flushEN<=jump; 

  assign jump = PC_src2;

  always @(*) begin
    nextState = state;
    //flushEN=0;
    loadpcNext = 1;
    case (state)
      _START: begin
        nextState = _RUN;
       // flushEN=1;
      end
      _RUN: begin
        if (isLOAD)  begin
          nextState = _WAIT;
          loadpcNext = 0;
        end
        if (jump) nextState = _WAIT;

        if (error) begin
          nextState = _ERROR;
          loadpcNext = 0;
        end
        if (end_prog) begin
          nextState = _DEBUG;
          loadpcNext = 0;
        end
      end
      _WAIT: begin
          nextState = _RUN;
      end
      _DEBUG: begin
        loadpcNext = 0;
        //flushEN=1;
      end
      _ERROR: begin
        loadpcNext = 0;
       // flushEN=1;
      end
      default: nextState = _START;
    endcase
  end
  
  assign isLOAD   = (opcode == 7'b00_000_11);
  assign isFENCE  = (opcode == 7'b00_011_11);
  assign isALUimm = (opcode == 7'b00_100_11);
  assign isAUIPC  = (opcode == 7'b00_101_11);
  assign isSTORE  = (opcode == 7'b01_000_11);
  assign isALUreg = (opcode == 7'b01_100_11);
  assign isLUI    = (opcode == 7'b01_101_11);
  assign isBRANCH = (opcode == 7'b11_000_11);
  assign isJALR   = (opcode == 7'b11_001_11);
  assign isJAL    = (opcode == 7'b11_011_11);
  assign isSYSTEM = (opcode == 7'b11_100_11);

  (*parallel_case*)
  always @(*) begin
    // caso usual si no aparece en un opcode
    end_prog=0; error=0;
    RegWrite=0; // 0: no escribe en rd; 1: escribe
    ALUSrc1=0; // 0: reg(rs1); 1: ActPC
    ALUSrc2=0; // 0: reg(rs2); 1: Imm
    PcPlus4toReg=0; //1: reg(rd)=ActPC+4 0: = (depende de MemtoReg)
    MemtoReg=0; // 0: reg(rd)=ALUout; 1: = read_data;
    funQual=1'b0; // 0: op usual; 1: op alternativa (para sub y sra)
    CSRen = 0; CSRWrite=0;CSRRead=0;
    PC_src1 = 1'bx; PC_src2 = 0;
    case(1'b1) 
      isLOAD: begin //load
        ALUSrc2=1;  // ALUin2=imm
        MemtoReg=1; // Reg(rd)=ReadData
        RegWrite = (state==_WAIT); // se guarda valor en Reg(rd)
      end
       isFENCE: begin //fence
         //isFENCE=1; // No hace nada, literalmente....
       end
      isALUimm: begin //ALUimm
        RegWrite=1; // se guarda valor en Reg(rd)
        ALUSrc2=1; // ALUin2=imm
        funQual = (funct3[1:0] == 2'b01)? funct7[5] : 1'b0; // En estas instrucciones, solo se presta atencion a este cualificador cuando funct3[1:0] indica desplazamientos.
      end
      isAUIPC: begin //AUIPC
        ALUSrc1=1; // ALUin1=pc
        ALUSrc2=1; // ALUin2=imm
        RegWrite=1; // se guarda valor en Reg(rd)
        // se guarda ALUout =pc+imm en Reg(rd)
      end
      isSTORE: begin //store
        ALUSrc2=1;  // ALUin2=imm
        MemtoReg=1'bx;    // \ No importa, 
        PcPlus4toReg=1'bx;// /  no se escriben los registros
      end
      isALUreg: begin //ALUreg
        RegWrite=1; // se guarda valor en Reg(rd)
        funQual=funct7[5];
      end
      isLUI: begin //LUI
        ALUSrc2=1;  // ALUin2=imm
        RegWrite=1; // se guarda valor en Reg(rd)
      end
      isBRANCH: begin //branch
        // lee los dos registros
        MemtoReg=1'bx;    // \ No importa, 
        PcPlus4toReg=1'bx;// /  no se escriben los registros
        PC_src2 = ALUbranch; // salta si ALUbranch
        PC_src1 = 0; // pc+imm
      end
      isJALR: begin//jalr
        // ALUin1=reg
        ALUSrc2=1; // ALUin2=imm
        MemtoReg=1'bx; // No importa, se guardara pc+4
        PcPlus4toReg=1;// reg(rd)=pc+4;
        RegWrite=1; // se guarda valor en Reg(rd)
        PC_src1 = 1; // ALUout
        PC_src2 = 1; // salta
      end
      isJAL: begin //jal
        ALUSrc1=1'bx; // ALUin1=pc
        ALUSrc2=1'bx; // ALUin2=imm
        MemtoReg=1'bx; // No importa, se guardara pc+4
        PcPlus4toReg=1;// reg(rd)=pc+4;
        RegWrite=1; // se guarda valor en Reg(rd)
        PC_src1 = 0; // pc+imm
        PC_src2 = 1; // salta
      end
      isSYSTEM: begin //Sistema
        if (funct3 == 3'b0) begin
          if (instr[20]==1) begin // ebreak
            end_prog=1;
          end // else ecall (no implementado)
        end else begin // CSR
          CSRen = 1;
          RegWrite = 1;
          if (funct3[1]) begin // CSRRW[I]
            CSRWrite = 1;
            CSRRead = ~|rd;
          end else begin // CSRRS[I] o CSRRC[I]
            CSRRead = 1;
            CSRWrite = ~|rs1;
          end
        end
      end

      default: begin // error!
        error=1;
        ALUSrc1 = 1'bx;
        ALUSrc2 = 1'bx;
      end
    endcase
  end

  // --- Decodificacion del inmediato segun formatos 
  wire [31:0] Iimm = {{21{instr[31]}}, instr[30:20]};
  wire [31:0] Simm = {{21{instr[31]}}, instr[30:25], instr[11:7]};
  wire [31:0] Bimm = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
  wire [31:0] Jimm = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};   
  wire [31:0] Uimm = {instr[31], instr[30:12], {12{1'b0}}};

  // - Seleccion de inmediato

  (*parallel_case*) 
  always @(*) begin
    case (1'b1)
      isAUIPC:  imm = Uimm;
      isLUI:    imm = Uimm;
      isBRANCH: imm = Bimm;
      isSTORE:  imm = Simm;
      isJAL:    imm = Jimm;
      default:  imm = Iimm;
    endcase
  end
  
  // --- 32 Registros
  registros reg32( // control: RegWrite
    .clk(clk),
    .wr_ena(RegWrite),
    .readReg1(rs1 & {5{!isLUI}}), // Si es LUI: se fuerza al registro que lea x0
    .readReg2(rs2),
    .reg1(reg1Data),
    .reg2(reg2Data),
    .wr_reg(rd),
    .wr_data(WBData) );

  // *-3-* Execute *-3-*
  
  // -------- ALU --------
  // --- Seleccion de entradas
  assign ALUin1 = (ALUSrc1)?  pc : reg1Data;
  assign ALUin2 = (ALUSrc2)? imm : reg2Data;
  
  // --- Control principal de ALU
  assign ALUCtrl = ((isALUreg||isALUimm||isBRANCH))?  
                    {funQual,funct3}:
                    4'b0000;

  // --- Bloque ALU
  ALU alu( 
    .ALUCtrl(ALUCtrl),
    .in1(ALUin1),
    .in2(ALUin2),
    .ALUout(ALUout),
    .branch(ALUbranch));

  // --- Bloque CSR
  // Definicion de los CSR 
  reg [63:0] cycle,instret; // registro time implementado como cycle...
  reg [31:0] PWM_;
  
  // * Etapa de Lectura de CSR *--- 
    reg [31:0] CSR_value;
    always @(*) case (csr)
      12'h800: CSR_value = PWM_;
      12'hC00: CSR_value = cycle[31:0];
      12'hC01: CSR_value = cycle[31:0];
      12'hC02: CSR_value = instret[31:0];
      12'hC80: CSR_value = cycle[63:32];
      12'hC81: CSR_value = cycle[63:32];
      12'hC82: CSR_value = instret[63:32];
      default: CSR_value = 32'b0;
    endcase

  // * Etapa de modificacion de CSR *---
    // Valor de entrada
    wire [31:0] CSR_Xtern = funct3[2] ? {27'b0,rs1} // 1:inmediato extendido
                                    : reg1Data;       // 0: Registro
    // Modificacion de CSR
    wire [31:0] CSR_mod = funct3[0] ? (CSR_value & (~CSR_Xtern)) // 1: clear bits
                                    : (CSR_value | CSR_Xtern);   // 0: set bits
    // Escritura de CSR (valor a escribir)
    wire [31:0] CSR_WrIN = funct3[1] ? CSR_mod    // 1: (set/clear)
                                     : CSR_Xtern; // 0: valor externo

  // * Etapa de escritura de CSR *--- (Revisar operacion de los CSR al final)
  // *** Para agregar nuevos CSR: la escritura con la señal CSRWrite es prioridad, y debe guardar CSR_WrIN

  // *-4-* Memoria *-4-*
  // control: isSTORE(MemWrite), isLOAD(MemRead), funct3={unsigned,data_width}
  
  // Redundantes, pero son los nombres que posee el bloque de memoria en Patterson & Hennesy(2019)
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
  assign WBData = (PcPlus4toReg? pcFetch :
                  (    MemtoReg? read_data:
                  (       CSRen? CSR_value :
                                 ALUout)));

  // --- Control de seleccion de pcNext (basado en control principal)
 // assign PC_src1 = isJAL | (isBRANCH && ALUbranch); // 0: ActPC+4; 1: ActPC+imm;
  //assign PC_src2 = isJALR; // 0: (ActPC+4 o ActPC+imm); 1: ALUout;

  // --- Suma de ActPC con inmediato
  assign pcMasImm = pc+imm;

  // --- Seleccion del pcNext
  assign pcJump = (PC_src1)? ALUout : pcMasImm; 
  assign pcNext = (PC_src2)? pcJump : pcMas4;  

  // *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-

  // Definicion de los CSR implementados
  // PWM_: registro de prueba. No tiene necesariamente el valor csr
  always @(posedge clk)
    if (rst)
      PWM_<=0;
    else if (CSRWrite) 
      if (csr == 12'h800)
        PWM_ <= CSR_WrIN;
        // otros CSR son solo lectura

  //wire fin_inst = ~ (flushEN | stop_addr);

  // cycle/time_ e instret
  always @(posedge clk) begin   // Contadores de sistema
    if (rst) begin
      cycle <= 64'b0;
      instret <= 64'b0;
    end else begin
      cycle <= cycle+1;
      instret <= instret + (state==_RUN);
    end
  end
endmodule