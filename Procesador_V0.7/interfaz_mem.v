// interfaz_mem.v

/* StoretoMEM (.addr_LSB(),.data(),.memWrite(),.data_width(),.wr_datatoMem(),.mask_data());
*/
module StoretoMEM (//********************************************************************
  // Entradas de Store
  input [1:0] addr_LSB, // 2 bits menos significativo de la direccion
  input [31:0] data, // Data to save!
  // entrada de control
  input memWrite, // señal de control que permite escribir en memoria!
  input [1:0] data_width, // 00: byte; 01:half-word; 10:word
  // salidas a la memoria!
  output reg [31:0] wr_datatoMem,
  output [3:0] mask_data
);//*************************************************************************************

  reg [3:0] mask;
  assign mask_data = {4{memWrite}} & mask; // bit-wise and entre memWrite con mask
  
  // ** Bloque de alineacion: se revisa si se guarda byte, half-w o word, y el dato de entrada se alinea
  //    al byte/half-word correspondiente a los dos bits menos significativos de address. mask indica cual
  //    de los 4 bytes se realiza la operacion de escritura.
  always @(*) begin
    case (data_width[1:0])
      2'b00: begin// Se guarda dato byte (sb)
        case (addr_LSB)
          2'b00: begin // byte 0
            wr_datatoMem = {24'bxxxxxxxxxxxxxxxxxxxxxxxx,data[7:0]};
            mask = 4'b0001;
          end
          2'b01: begin // byte 1
            wr_datatoMem = {16'bxxxxxxxxxxxxxxxx,data[7:0],8'bxxxxxxxx};
            mask = 4'b0010;
          end
          2'b10: begin // byte 2
            wr_datatoMem = {8'bxxxxxxxx,data[7:0],16'bxxxxxxxxxxxxxxxx};
            mask = 4'b0100;
          end
          2'b11: begin // byte 3
            wr_datatoMem = {data[7:0],24'bxxxxxxxxxxxxxxxxxxxxxxxx};
            mask = 4'b1000;
          end
        endcase
      end
      
      2'b01: begin // Se guarda dato half-word (sh)
        case (addr_LSB[1])
          1'b0: begin // byte 1-0
            wr_datatoMem = {16'bxxxxxxxxxxxxxxxx,data[15:0]};
            mask = 4'b0011;
          end
          1'b1: begin // byte 3-2
            wr_datatoMem = {data[15:0],16'bxxxxxxxxxxxxxxxx};
            mask = 4'b1100;
          end
        endcase
      end

      2'b10: begin // no dependen del data_byte, pues se alineará a 00 siempre!
        wr_datatoMem = data;
        mask = 4'b1111;
      end

      default : begin
        wr_datatoMem = 32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx;
        mask = 4'b0000;
      end
    endcase
  end
endmodule

/* StoretoMEM (.addr_LSB(),.data_mem(),.data_width(),.sin_signo(),.data_readed());
*/
module LoadfromMEM(//***************************************************************
  // entradas
  input [1:0] addr_LSB, // 2 bits menos significativo de la direccion
  input [31:0] data_mem, // dato proveniente de la memoria
  // control
  input [1:0] data_width, // 00: byte; 01:half-word; 10:word
  input sin_signo, // indica si el dato cargado debe extenderse con cero o con signo
  // salida al procesador
  output reg [31:0] data_readed // dato alineado, hacia el procesador
);//********************************************************************************

  // Decodificando data_byte y data_half addr para lb/lbu y lh/lhu!
  wire  [7:0] data_byte;
  wire [15:0] data_half;
  assign data_half = addr_LSB[1] ? data_mem[31:16] : data_mem[15:0];
  assign data_byte = addr_LSB[0] ? data_half[15:8] : data_half[7:0]; 

  // señal que extiende el signo para byte o half-word
  wire signo_byte,signo_half;
  assign signo_byte = (sin_signo) ? 0 :  data_byte[7];
  assign signo_half = (sin_signo) ? 0 : data_half[15];

  // ** Bloque multiplexor final, dependiendo del tamaño del dato, y si debe extenderse con signo o no,
  //    el dato de la memoria se ajusta para ser recibido por el procesador
  always @(*) begin
    case (data_width)
      2'b00: data_readed = {{24{signo_byte}},data_byte};
      2'b01: data_readed = {{16{signo_half}},data_half};
      2'b10: data_readed = data_mem;
      default: data_readed = 32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx;
    endcase
  end
endmodule