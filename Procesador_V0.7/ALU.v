// ALU.v

/*
* ALU (.ALUCtrl(), .in1(), .in2(), .ALUout(), .branch());
*   La ALU realiza 8 operaciones principales: suma, desplazamiento hacia la izquierda, establece en 1 si i1<i2 con signo y sin signo,
*   XOR, desplazamiento hacia la derecha, OR y AND. 
*   La señal esp_fun en el bit 3 de ALUCtrl sirve para realizar la resta en vez de una suma, y un desplazamiento hacia la derecha
*   aritmetico en vez de lógico.
*/
module ALU (
  //control
  input [3:0] ALUCtrl, // Control de operaciones de la ALU: {esp_fun, funcion} 
  // Entradas
  input [31:0] in1, // entrada 1,
  input [31:0] in2, // entrada 2
  //salida
  output reg [31:0] ALUout, // salida de la ALU
  // salida de control
  output reg branch // indica si ocurre o no un salto!
);

  // ** Decodificacion del control dentro del ALU
  wire [2:0] funcion = ALUCtrl[2:0]; // generalmente indica uno de los 8 casos de funct3 del opcode
  wire       esp_fun = ALUCtrl[3]; // indica una operacion especial de ADD (SUB) y de SLL (SRL/SRA)
  wire [4:0]   shamt = in2[4:0]; // para el desplazamiento, siempre utiliza los 5 bits menos significativos de in2
 
  // ** Bloque de desplazamiento: realiza desplazamiento hacia la izquierda o derecha
  wire [31:0] shift_out;
  shift_leftright shifts(
    .in(in1),
    .shamt(shamt),
    .der(funcion[2]), // funcion[2] indica si es izquierda(0) o derecha(1)
    .arith(esp_fun),
    .out(shift_out));

  // ** Bloque de resta: señales utilizadas:
  // minus es una resta de 33 bits entre in1 e in2,
  //    esto implica que minus[32] equivale al préstamo de dicha resta
  // LT es comparacion con signo (in1 < in2)
  //    signos iguales: se toma el préstamo de la resta (si hay préstamo, i1 es menor)
  //    signos distintos: solo basta observar el signo de in1 (i1 es menor si es negativo, esto es signo=1)
  // LTU es comparacion sin signo, por lo que solo se observa si ocurre un prestamo.
  // EQ es (in1==in2), esto es, basta obsercar que minus sea 0 
  // las señales LT y LTU se utilizan tanto en la operacionde la ALU como en branch
  wire [32:0] minus = {1'b0,in1} + ({1'b1, ~in2} + 33'b1); //in1 - in2
  wire  LT  = (in1[31] ^ in2[31]) ? in1[31] : minus[32];
  wire  LTU = minus[32];
  wire  EQ  = (minus[31:0] == 0);
  
 
  // ** bloque de operaciones de la ALU: el resultado dependen de las señales de control de entrada.
  always @(*) begin
    case(funcion)
      3'b000: ALUout <= esp_fun ? minus[32:0]: in1+in2; // ADD/ADDI/SUB o instrucciones store/load o (tipo U)
      3'b001: ALUout <= shift_out; // SLL/SLI
      3'b010: ALUout <= {31'b0,LT}; // SLT/SLTI
      3'b011: ALUout <= {31'b0,LTU}; // SLTU/SLTUI
      3'b100: ALUout <= in1 ^ in2; // XOR/XORI
      3'b101: ALUout <= shift_out;  // SRL/SRLI/SRA/SRAI
      3'b110: ALUout <= in1 | in2; // OR/ORI
      3'b111: ALUout <= in1 & in2; // AND/ANDI
    endcase
  end  

  // ** bloque branch:
  //    define si ocurre un branch o no, basado en lo que indique funcion y las tres señales logicas LT,LTU y EQ
  //(* parallel_case, full_case *)   // Averiguar para que sirve !!!!!!!!!!!!!
  always @(*) begin    
    case(funcion)
      3'b000:  branch  =  EQ;   // BEQ
      3'b001:  branch  = !EQ;   // BNE
      3'b100:  branch  =  LT;   // BLT
      3'b101:  branch  = !LT;   // BGE
      3'b110:  branch  =  LTU;  // BLTU
      3'b111:  branch  = !LTU;  // BGEU
      default: branch  = 1'bx; // don't care...
    endcase
  end 
endmodule

/* shift_leftright(.in(),.shamt(),.der(),.arith(),.out())
*/
module shift_leftright(
  // entradas
  input [31:0] in, // dato de entrada
  input [4:0] shamt,// numero de dezplazamiento
  // control
  input der, // indica si el resultado se desplaza hacia la derecha (1)
  input arith, // Indiica, cuando der = 1, que el resultado de un dezplazamiento hacia la derecha es aritmetico
  // salida
  output [31:0] out // dato de salida (dato desplazado)
);
  // desplazamientos auxiliares
  reg [31:0] SLL_4, SLL_3, SLL_2, SLL_1, SLL_0;

  // Señal que mantiene el signo aritmetico al hacer un desplazamiento a la derecha aritmetico
  wire rb;
  assign rb = arith&(in[31]);

  // Cada bit de shamt representa un desplazamiente de la potencia de 2. shamt[4] desplaza 2^4=16 bits, shamt[3] desplaza
  // 8 bits, etc... por ello, 'in' pasa por etapas, donde por cada shamt[i] == 1 realiza 2^i desplazamientos de bits según
  // y shamt[i]==0 mantiene el valor anterior!
  always @(*) begin
    if (der) begin
      SLL_4 = !shamt[4]? in : {{16{rb}},in[31:16]};
      SLL_3 = !shamt[3]? SLL_4 : {{8{rb}},SLL_4[31:8]};
      SLL_2 = !shamt[2]? SLL_3 : {{4{rb}},SLL_3[31:4]};
      SLL_1 = !shamt[1]? SLL_2 : {{2{rb}},SLL_2[31:2]};
      SLL_0 = !shamt[0]? SLL_1 : {rb,SLL_1[31:1]};
    end else begin
      SLL_4 = !shamt[4]? in : {in[15:0],16'b0};
      SLL_3 = !shamt[3]? SLL_4 : {SLL_4[23:0],8'b0};
      SLL_2 = !shamt[2]? SLL_3 : {SLL_3[27:0],4'b0};
      SLL_1 = !shamt[1]? SLL_2 : {SLL_2[29:0],2'b0};
      SLL_0 = !shamt[0]? SLL_1 : {SLL_1[30:0],1'b0};
    end
  end
  assign out = SLL_0;
endmodule
