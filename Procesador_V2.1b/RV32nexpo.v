//RV32nexpo.v
// V2.1b: Segmentado.
// * Segmentacion de 5 etapas
// * No posee flush ni stall aplicados
// * Todos los peligros de datos y control posibles persisten
// * Instruccion fence es un nop!
// * ebreak detiene el programa implementado
// * ecall no está implementado ...
// * implementado ZiCSR: Timers implementados... Custom CSR por observar! (detallar depurador)
// RV32I_Zicsr

//------RV32nexpo()----------------------------------------------------------------------
module RV32nexpo (
  input rst,
  input clk,
  // Bus de control y estado del cpu
  //input halt,
  output error, 
  output end_prog,
  // Bus de memoria de instrucciones
  input [31:0] InstrData, // bus de instruccion
  output [31:0] InstrAddr, // bus de direccion de instruccion
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

  parameter INIT_ADDRESS = 32'h0000_0000; // Direccion inicial. Así será fácilmente cambiada...
  localparam I_NOP = 32'h0000_0013; // addi x0,x0,0

  // #################*** operacion del softcore ***#####################

  // -- Definicion de los CSR 
  reg [63:0] cycle,instret; // registro time implementado como cycle...
  reg [31:0] PWM_;
 
  // -- Definicion de las señales de control de la segmentación
  wire Stall_F,Stall_D,Stall_E,Stall_M; // Estancamiento del flujo de los segmentos siguientes
  wire Flush_D,Flush_E,Flush_M,Flush_W; // Generacion de "burbujas" en el segmento que viene
  
  // Niveles: Fetch-Decode-Execute-Memory-WriteBack (F/D/E/M/Wb)

  // -1- %%%%%%%%%%%%%%%%%%%%%%%%%%*       *%%%%%%%%%%%%%%%%%%%%%%%%%% -1-
  // --------------------------***** Fetch *****--------------------------

    // --- Instruccion a buscar
    assign InstrAddr = Stall_F? FD_pc : F_pc; 
    // --- Sumador de pc con numero 4
    wire[31:0] F_pcMas4 = F_pc+4;

    // Registros de Segmentacion F->D ##############################################
    always @(posedge clk) begin
      if (rst) begin// -- Operacion en Reset
        F_pc  <= INIT_ADDRESS;
      end else begin

      if (!Stall_F) begin // Operacion Usual
          FD_pc <= F_pc; // Deberia ocurrir en el Stall_D
          F_pc <= F_pcMas4; 
        end
        if (E_JumpOrBranch) // -- Ocurre salto
          F_pc <= E_JumpAddr;
      end
      
      FD_nop <= Flush_D | rst;

    end  // ########################################################################
    
    reg [31:0] F_pc=INIT_ADDRESS, FD_pc=0;
    reg FD_nop=1;
    // --- generacion de siguiente instruccion. Si FD_nop, entonces nop
    wire [31:0] FD_instr = FD_nop ? I_NOP : InstrData;
  // -1- %%%%%%%%%%%%%%%%%%%%%%%%%%*%%%%%%%*%%%%%%%%%%%%%%%%%%%%%%%%%% -1-


  // -2- %%%%%%%%%%%%%%%%%%%%%%%%%%*        *%%%%%%%%%%%%%%%%%%%%%%%%%% -2-
  // --------------------------***** Decode *****--------------------------
    
    // -------- Decodificacion del a instruccion -----------
    // --- Decodificacion directa de la instruccion
    wire [7:0] D_opcode = FD_instr[6:0];
    wire [4:0] D_rd     = FD_instr[11:7];
    wire [2:0] D_funct3 = FD_instr[14:12];
    wire [4:0] D_rs1    = FD_instr[19:15]; 
    wire [4:0] D_rs2    = FD_instr[24:20];
    wire [7:0] D_funct7 = FD_instr[31:25];
    
    // --- Decodificacion del tipo de instruccion (control)
    wire D_isLOAD   = (D_opcode == 7'b00_000_11);
    wire D_isFENCE  = (D_opcode == 7'b00_011_11);
    wire D_isALUimm = (D_opcode == 7'b00_100_11);
    wire D_isAUIPC  = (D_opcode == 7'b00_101_11);
    wire D_isSTORE  = (D_opcode == 7'b01_000_11);
    wire D_isALUreg = (D_opcode == 7'b01_100_11);
    wire D_isLUI    = (D_opcode == 7'b01_101_11);
    wire D_isBRANCH = (D_opcode == 7'b11_000_11);
    wire D_isJALR   = (D_opcode == 7'b11_001_11);
    wire D_isJAL    = (D_opcode == 7'b11_011_11);
    wire D_isSYSTEM = (D_opcode == 7'b11_100_11);
    wire D_isEBREAK = D_isSYSTEM & D_funct3ZERO & FD_instr[20];
    wire D_isCSR    = D_isSYSTEM & !D_funct3ZERO;

    // --- Decodificacion del inmediato segun formatos 
    wire [31:0] D_Iimm = {{21{FD_instr[31]}}, FD_instr[30:20]};
    wire [31:0] D_Simm = {{21{FD_instr[31]}}, FD_instr[30:25], FD_instr[11:7]};
    wire [31:0] D_Bimm = {{20{FD_instr[31]}}, FD_instr[7], FD_instr[30:25], FD_instr[11:8], 1'b0};
    wire [31:0] D_Jimm = {{12{FD_instr[31]}}, FD_instr[19:12], FD_instr[20], FD_instr[30:21], 1'b0};   
    wire [31:0] D_Uimm = {FD_instr[31], FD_instr[30:12], {12{1'b0}}};

    // --- Seleccion de inmediato
    reg [31:0] D_imm;
    (*parallel_case*) 
    always @(*) case (1'b1)
      D_isAUIPC:  D_imm = D_Uimm;
      D_isLUI:    D_imm = D_Uimm;
      D_isBRANCH: D_imm = D_Bimm;
      D_isSTORE:  D_imm = D_Simm;
      D_isJAL:    D_imm = D_Jimm;
      default:    D_imm = D_Iimm;
    endcase

    // ----------- 32 Registros -----------
    wire [31:0] D_rs1Data, D_rs2Data, Wb_rdData; // Dato de los registros
    wire [4:0] Wb_rd; 
    wire Wb_RegWrite;
    registros reg32( // control: RegWrite
      .clk(clk),
      .wr_ena(Wb_RegWrite),
      .readReg1(D_rs1 & {5{!D_isLUI}}), // (TODO) Eliminar señal de aqui. Si es LUI: se fuerza al registro que lea x0
      .readReg2(D_rs2),
      .reg1(D_rs1Data),
      .reg2(D_rs2Data),
      .wr_reg(Wb_rd), 
      .wr_data(Wb_rdData) ); 

    // ------------- Control Principal -----------------
    // --- Señales de ayuda para la decodificacion o deteccion de peligros
    wire D_rdNoZero = |D_rd;
    wire D_rs1Read = !(D_isJAL || D_isLUI || D_isAUIPC);
    wire D_rs2Read = D_isALUreg || D_isBRANCH || D_isSTORE; // isSystem no es necesario, pero no importa si lo es || D_isSYSTEM;
    wire D_funct3ZERO = (D_funct3 == 3'b0);
    
    // --- Generacion del Control Principal que se propagará al resto de la segmentacion
    reg D_RegWrite, D_ALUSrc1, D_ALUSrc2, D_PCPlus4toReg, D_funQual,
        D_CSRWrite, D_CSRRead, D_PC_src1, D_PC_src2, D_error;
    //wire D_error = ~(D_isLOAD||D_isFENCE||D_isALUimm||D_isAUIPC||D_isSTORE||D_isALUreg||D_isLUI||D_isBRANCH||D_isJAL||D_isJALR||D_isSYSTEM);
    // wire D_ALUSrc1 = D_isAUIPC || D_isJAL;
    // wire D_ALUSrc2 = D_isLOAD || D_isALUimm || D_isAUIPC || D_isSTORE || D_isLUI || D_isJALR;
    // wire D_funQual = D_funct7[5] & (D_isALUreg | (D_isALUimm & D_funct3[1:0]==2'b01));
    // wire D_RegWrite = ~ (D_isFENCE || D_isSTORE || D_isBRANCH || D_isEBREAK) && D_ORrd;
    // wire D_CSRWrite = D_isCSR && (D_funct3[1]? 1 : ~|D_rs1);
    // wire D_CSRRead = D_isCSR && (D_funct3[1]? ~D_ORrd : 1);
    wire D_ALUFun = D_isALUreg || D_isALUimm || D_isBRANCH;
    /*parallel_case*/
    always @(*) begin
      // caso usual si no aparece en un opcode
      D_error=0;
      D_RegWrite = D_rdNoZero; // 0: no escribe en rd; 1: escribe (si D_rdNoZero)
      D_ALUSrc1=0; // 0: reg(rs1); 1: pc (DE_pc)
      D_ALUSrc2=0; // 0: reg(rs2); 1: Imm
      D_funQual=0; // 0: op usual; 1: op alternativa (para sub y sra)
      D_CSRWrite=0; D_CSRRead=0; // 0: no Escribe/lee en CSR, 1: si hace la accion
      case(1'b1) 
        D_isLOAD: begin // load
          D_ALUSrc2=1;  // ALUin1=reg / ALUin2=imm // lee en memoria y guarda en Reg(rd)
          end
        D_isFENCE: begin // fence
          D_RegWrite = 0; // fence es un nop!
          end
        D_isALUimm: begin // ALUimm
          D_ALUSrc2=1; // ALUin1=reg / ALUin2=imm
          D_funQual = (D_funct3[1:0] == 2'b01)? D_funct7[5] : 1'b0; // En estas instrucciones, solo se presta atencion a este cualificador cuando funct3[1:0] indica desplazamientos.
          end
        D_isAUIPC: begin // AUIPC
          D_ALUSrc1=1; // ALUin1=pc
          D_ALUSrc2=1; // ALUin2=imm // guarda pc+imm en Reg(rd)
          end
        D_isSTORE: begin //store
          D_ALUSrc2=1;  // ALUin1=reg / ALUin2=imm
          D_RegWrite=0; // No guarda en los registros, pero si en memoria
          end
        D_isALUreg: begin //ALUreg
          D_funQual = D_funct7[5]; // modificador se activa con restador
          end
        D_isLUI: begin //LUI
          D_ALUSrc2=1;  // ALUin1=32'b0 / ALUin2=imm
          end
        D_isBRANCH: begin //branch
          D_RegWrite=0; // Branch no escribe en registros
          end 
        D_isJALR: begin//jalr
          D_ALUSrc2=1; // ALUin1=reg / ALUin2=imm
          end
        D_isJAL: begin //jal
          D_ALUSrc1=1'b1; // ALUin1=pc
          D_ALUSrc2=1'bx; // ALUin2=imm
          end
        D_isEBREAK: begin // ebreak
          D_RegWrite=0; // ebreak no escribe en registros
          end 
        D_isCSR: begin // csr (read/write)
          if (D_funct3[1]) begin // CSRRW[I]
            D_CSRWrite = 1;
            D_CSRRead = D_rdNoZero;
          end else begin // CSRRS[I] o CSRRC[I]
            D_CSRRead = 1;
            D_CSRWrite = |D_rs1;
          end
          end
        default: begin 
          D_error=1; // error!
          D_RegWrite=0;
          end
      endcase
    end

    // Registros de Segmentacion D->E ############################################## 
    always @(posedge clk) begin
      if(!Stall_D) begin // operacion usual
       // señales de datos
        DE_rs1Data <= D_rs1Data;
        DE_rs2Data <= D_rs2Data;
        DE_imm <= D_imm;
        DE_pc <= FD_pc;
        DE_rs1 <= D_rs1;
        DE_rs2 <= D_rs2;
        DE_rd <= D_rd;
        DE_funct3 <= D_funct3;

        // señales de control
        DE_nop      <= FD_nop;
        DE_MemWrite <= D_isSTORE;
        DE_MemRead  <= D_isLOAD;
        DE_RegWrite <= D_RegWrite; // D_isALUimm | D_isALUreg | D_isLUI | D_isJAL | D_isJALR; // | ...
        DE_ALUSrc1  <= D_ALUSrc1;
        DE_ALUSrc2  <= D_ALUSrc2;
        DE_ALUCtrl  <= {D_funQual, (D_ALUFun ? D_funct3 : 3'b0)};
        DE_isJALR   <= D_isJALR;
        DE_Jump     <= D_isJAL || D_isJALR;
        DE_isBRANCH <= D_isBRANCH;
        DE_isCSR    <= D_isCSR;
        DE_isEBREAK <= D_isEBREAK;
        DE_CSRWrite <= D_CSRWrite;
        DE_CSRRead  <= D_CSRRead;
        DE_error    <= D_error;
      end

      if (Flush_E | FD_nop) begin
        DE_nop      <= 1'b1;
        DE_MemWrite <= 1'b0;
        DE_MemRead  <= 1'b0;
        DE_RegWrite <= 1'b0;
        DE_ALUSrc1  <= 1'b0; // (TODO) al rehacer las operaciones tipo U, se puede eliminar esta señal !!
        DE_ALUSrc2  <= 1'b0;
        DE_ALUCtrl  <= 4'b0;
        DE_isJALR   <= 1'b0;
        DE_Jump     <= 1'b0;
        DE_isBRANCH <= 1'b0;
        DE_isCSR    <= 1'b0;
        DE_isEBREAK <= 1'b0;
        DE_CSRWrite <= 1'b0;
        DE_CSRRead  <= 1'b0;
        DE_error    <= 1'b0;
      end
    end  // ########################################################################
      reg [31:0] DE_pc, DE_rs1Data, DE_rs2Data, DE_imm;
      reg [4:0] DE_rs1, DE_rs2, DE_rd;
      reg [3:0] DE_ALUCtrl;
      reg [2:0] DE_funct3;
      reg DE_nop, DE_MemWrite, DE_RegWrite, DE_MemRead, DE_ALUSrc1,
          DE_ALUSrc2, DE_isJALR, DE_Jump, DE_isBRANCH, DE_isEBREAK,
          DE_isCSR, DE_CSRWrite, DE_CSRRead, DE_error;
  // -2- %%%%%%%%%%%%%%%%%%%%%%%%%%*%%%%%%%%*%%%%%%%%%%%%%%%%%%%%%%%%%%%% -2-  


  // -3- %%%%%%%%%%%%%%%%%%%%%%%%%%*         *%%%%%%%%%%%%%%%%%%%%%%%%%% -3-
  // --------------------------***** Execute *****--------------------------
  
    // ------- Unidad de Forwarding -------
    // de M -> E
    wire E_M_fwd_rs1 = EM_RegWrite && (DE_rs1 == EM_rd);
    wire E_M_fwd_rs2 = EM_RegWrite && (DE_rs2 == EM_rd);
    // de Wb -> E
    wire E_W_fwd_rs1 = MW_RegWrite && (DE_rs1 == MW_rd); 
    wire E_W_fwd_rs2 = MW_RegWrite && (DE_rs2 == MW_rd);
    // Señal de entrada rs1 y rs2 traidas de la etapa M, W o D
    wire [31:0] E_rs1Data = E_M_fwd_rs1 ? EM_resultado:
                            E_W_fwd_rs1 ? Wb_rdData   :
                                          DE_rs1Data  ;
    wire [31:0] E_rs2Data = E_M_fwd_rs2 ? EM_resultado:
                            E_W_fwd_rs2 ? Wb_rdData   :
                                          DE_rs2Data  ;

    // -------- ALU --------
    // --- Seleccion de entradas
    wire [31:0] E_ALUin1 = (DE_ALUSrc1)?  DE_pc : E_rs1Data;
    wire [31:0] E_ALUin2 = (DE_ALUSrc2)? DE_imm : E_rs2Data;
    
    // --- Bloque ALU y Branch
    wire [31:0] E_ALUout;
    wire E_TakeBranch;
    ALU alu( 
      .ALUCtrl(DE_ALUCtrl),
      .in1(E_ALUin1),
      .in2(E_ALUin2),
      .ALUout(E_ALUout),
      .branch(E_TakeBranch));
    
    // -------- Bloque Salto --------
    // --- Suma de pc con inmediato
    wire [31:0] E_pcMasImm = DE_pc+DE_imm;
    
    // --- resultado sera DE_pc+4 si DE_Jump==1
    wire [31:0] E_pcMas4 = FD_pc; // en el contexto de los saltos, FD_pc = DE_pc+4 (no realizo una suma)
    
    // --- Direccion de salto definida por DE_isJALR
    wire [31:0] E_JumpAddr = DE_isJALR ? E_ALUout : E_pcMasImm;
   
    // --- Control de Salto
    wire E_JumpOrBranch = DE_Jump || (DE_isBRANCH && E_TakeBranch);

    // ----- Asignacion de señales de control de salida -----
    assign end_prog = DE_isEBREAK;
    assign error = DE_error;
    wire halt = !rst & (DE_isEBREAK | DE_error);

    // Registros de Segmentacion E->M ##############################################
    always @(posedge clk) begin
      if (!Stall_E) begin
        // señales de datos
        EM_resultado <= DE_Jump? E_pcMas4  : // Aqui se puede agregar las opciones de AUIPC y LUI:
        //            DE_AUIPC? E_pcMasImm: // (TODO)
        //              DE_LUI? DE_imm    : // Corroborar
                                E_ALUout  ;
        EM_rs2Data   <= E_rs2Data;
        EM_csrDataIn <= DE_funct3[2] ? {27'b0,DE_rs1} // 1: inmediato extendido
                                     : E_rs1Data;     // 0: Normal
        EM_csr      <= DE_imm[11:0];
        EM_rd       <= DE_rd;
        EM_funct3   <= DE_funct3;

        // señales de control
        EM_nop      <= DE_nop; // indica un nop
        EM_MemRead  <= DE_MemRead; // Memoria
        EM_MemWrite <= DE_MemWrite;
        EM_CSRRead  <= DE_CSRRead; // CSR
        EM_CSRWrite <= DE_CSRWrite;
        EM_isCSR    <= DE_isCSR;
        EM_RegWrite <= DE_RegWrite;
      end

      if (Flush_M | DE_nop) begin // señales de control
        EM_nop      <= 1'b1;
        EM_MemRead  <= 1'b0; // Memoria
        EM_MemWrite <= 1'b0;
        EM_CSRRead  <= 1'b0; // CSR
        EM_CSRWrite <= 1'b0;
        EM_isCSR    <= 1'b0;
        EM_RegWrite <= 1'b0;
      end
    end  // ########################################################################
    reg [31:0] EM_resultado, EM_rs2Data, EM_csrDataIn;
    reg [11:0] EM_csr;
    reg [4:0] EM_rd;
    reg [2:0] EM_funct3;
    reg EM_nop, EM_MemRead, EM_MemWrite, EM_RegWrite,
        EM_isCSR, EM_CSRRead, EM_CSRWrite;
  // -3- %%%%%%%%%%%%%%%%%%%%%%%%%%*%%%%%%%%%*%%%%%%%%%%%%%%%%%%%%%%%%%% -3-
  

  // -4- %%%%%%%%%%%%%%%%%%%%%%%%%%*        *%%%%%%%%%%%%%%%%%%%%%%%%%% -4-
  // --------------------------***** Memory *****--------------------------

    // -------- CSR --------
    // --- Etapa de Lectura de CSR
    reg [31:0] M_CSRData; 
    always @(*) case (EM_csr)
      12'h800: M_CSRData = PWM_;
      12'hC00: M_CSRData = cycle[31:0];
      12'hC01: M_CSRData = cycle[31:0];
      12'hC02: M_CSRData = instret[31:0];
      12'hC80: M_CSRData = cycle[63:32];
      12'hC81: M_CSRData = cycle[63:32];
      12'hC82: M_CSRData = instret[63:32];
      default: M_CSRData = 32'b0;
    endcase

    // --- Modificacion del CSR
    wire [31:0] M_csrMod = EM_funct3[0] ? (M_CSRData & (~EM_csrDataIn)) // 1: clear bits
                                        : (M_CSRData | EM_csrDataIn);   // 0: set bits
    // --- Valor a escribir en el CSR
    wire [31:0] M_CSRtoWrite = EM_funct3[1] ? M_csrMod      // 1: (set/clear)
                                            : EM_csrDataIn; // 0: valor externo

    // --- Etapa de escritura de CSR 
      // Revisar al final la operacion de los CSR
      // *** En esta etapa se actualizan dl registro CSR con M_CSRtoWrite si EM_CSRWrite se activó
    
    // -------- Bloque de Escritura Memoria RAM --------
    wire [31:0] M_address = EM_resultado; 
    wire [31:0] M_WriteData = EM_rs2Data; 

    // --- Ajuste para escribir en la memoria
    StoretoMEM store (
      .addr_LSB(M_address[1:0]), // igualmente al bloque load
      .memWrite(EM_MemWrite), //
      .data_width(EM_funct3[1:0]),
      .data(M_WriteData), // Entrada de datos
      .wr_datatoMem(mem_out), // señales de escritura
      .mask_data(mem_mask));

    // --- Asignacion de los puertos I/O de RV32nexpo
    assign mem_address = {M_address[31:2],2'b00}; // siempre buscará en memoria de manera alineada
    assign mem_read = EM_MemRead; 
    assign mem_write = EM_MemWrite;
    // mem_out se asignó en store

    // Registros de Segmentacion M->Wb #############################################
    always @(posedge clk) begin
      if(!Stall_M) begin
        // Señales de datos
        MW_address_LSB <= M_address[1:0];
        MW_resultado <= EM_isCSR ? M_CSRData
                                 : EM_resultado;
        MW_funct3  <= EM_funct3;
        MW_rd      <= EM_rd;
        // señales de control
        MW_nop      <= EM_nop;
        MW_MemRead  <= EM_MemRead;
        MW_RegWrite <= EM_RegWrite;
      end

      if (Flush_W | EM_nop) begin
        MW_nop <= 1'b1;
        MW_MemRead <= 1'b0;
        MW_RegWrite <= 1'b0;
      end

    end // ########################################################################
    reg [31:0] MW_resultado;
    reg [4:0] MW_rd;
    reg [2:0] MW_funct3;
    reg [1:0] MW_address_LSB;
    reg MW_nop, MW_MemRead, MW_RegWrite;  
  // -4- %%%%%%%%%%%%%%%%%%%%%%%%%%*%%%%%%%%*%%%%%%%%%%%%%%%%%%%%%%%%%% -4-


  // -5- %%%%%%%%%%%%%%%%%%%%%%%%%%*           *%%%%%%%%%%%%%%%%%%%%%%%%%% -5-
  // --------------------------***** WriteBack *****--------------------------
    
    // --- Bloque de ajuste de la lectura de la memoria.
    wire [31:0] Wb_MemoryData;
    LoadfromMEM load ( 
      .addr_LSB(MW_address_LSB), // solo 2 bits menos significativos...
      .data_width(MW_funct3[1:0]),
      .sin_signo(MW_funct3[2]),
      .data_mem(mem_in), // Entrada de datos
      .data_readed(Wb_MemoryData) ); // lectura final
    
    // --- Multiplexor de seleccion final
    assign Wb_RegWrite = MW_RegWrite;
    assign Wb_rd = MW_rd;
    assign Wb_rdData = MW_MemRead ? Wb_MemoryData :
                                    MW_resultado  ;
  // -5- %%%%%%%%%%%%%%%%%%%%%%%%%%*%%%%%%%%%%%*%%%%%%%%%%%%%%%%%%%%%%%%%% -5-

  // *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-

  //-------- Unidad de Deteccion de Peligro -------
  // --- Los peligros de Datos se detectan desde en la etapaE.
  //  (TODO) las señales de forwarding se detectaran en D pasando por los reg D->E para ser utilizadas en la etapa E
  
  // --- Deteccion de Hazard por lecturas
  wire rs1Hazard  = D_rs1Read && (D_rs1 == DE_rd); // 
  wire rs2Hazard  = D_rs2Read && (D_rs2 == DE_rd); // 

  wire loadHazard = (DE_MemRead || DE_CSRRead) && // Si ocurre lecturas de memoria
                    (rs1Hazard || rs2Hazard); 

  //--------- Control del pipelining ---------
  // (TODO) Agregar lógica para el depurador! 
  assign Flush_D = E_JumpOrBranch;
  assign Flush_E = E_JumpOrBranch | loadHazard;
  assign Flush_M = DE_isEBREAK; // Se hace un flush si es EBREAK
  assign Flush_W = 0;
  
  assign Stall_F = loadHazard | halt;
  assign Stall_D = loadHazard | halt;
  assign Stall_E = halt;
  assign Stall_M = 0;

  //-----

  //----- Espacio de los CSR
  //--- PWM_: registro de prueba. No tiene necesariamente el valor csr
  always @(posedge clk)
    if (rst)
      PWM_<=0;
    else if (EM_CSRWrite) begin
      if (EM_csr == 12'h800)
        PWM_ <= M_CSRtoWrite;
    end // otros CSR son solo lectura

  //--- cycle/time_ e instret
  always @(posedge clk) begin   // Contadores de sistema
    if (rst) begin
      cycle <= 64'b0;
      instret <= 64'b0;
    end else begin
      cycle <= cycle+1;
      if (!MW_nop) instret <= instret + 1;
    end
  end


  `ifdef BENCH      
    `include "../riscv_disassembly.v"
    wire [31:0] D_instr = FD_instr;
    reg [31:0] E_instr,M_instr,W_instr, M_pc, W_pc;
    wire JoB = E_JumpOrBranch;
    wire [31:0] D_pc = FD_pc,
                E_pc = DE_pc;
    wire St_F = Stall_F,  // Stall
         St_D = Stall_D,
         St_E = Stall_E,
         St_M = Stall_M,
         Fl_D = Flush_D,   // Flush
         Fl_E = Flush_E,
         Fl_M = Flush_M, 
         Fl_W = Flush_W;

    wire Fwd_rs1_M = E_M_fwd_rs1,
         Fwd_rs1_W = E_W_fwd_rs1,
         Fwd_rs2_M = E_M_fwd_rs2,
         Fwd_rs2_W = E_W_fwd_rs2;
    always @(posedge clk) begin
      if (!St_E) E_instr <= (Fl_E || FD_nop) ? I_NOP : D_instr;
      if (!St_M) begin
        M_instr <= (Fl_M || DE_nop) ? I_NOP : E_instr;
        M_pc <= E_pc;
      end

      W_instr <= (Fl_W || EM_nop) ? I_NOP : M_instr;
      W_pc <= M_pc;

      $display("* (SF)%10d ------------------------------------------------------------ *", cycle);
        $write( "| (%c ) IF| -pc: %8h ",St_F?"*":" ", F_pc);
        $write("\n");

        $write("| (%c%c) ID| -pc: %8h ",St_D?"*":" ",Fl_D?"*":" ",D_pc);
        $write("[%s%s] ",loadHazard && rs1Hazard?"*":" ",
                         loadHazard && rs2Hazard?"*":" ");
        riscv_disasm(D_instr,D_pc);
        $write("\n");
        
        $write("| (%c%c) Ex| -pc: %8h ",St_E?"*":" ",Fl_E?"*":" ", E_pc);
        if(DE_nop)
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
  `endif      
endmodule