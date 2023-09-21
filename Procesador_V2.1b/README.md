# Gateware
El procesador RV32nexpo en la version 2.1 agrega las 5 etapas de segmentación calasicas 
(Fetch->Decode->Execute->Memory->Writeback).
Los archivos que unen al procesador RV32nexpo son:
* RV32nexpo.v
  * registros_32.v
  * ALU.v
  * interfaz_mem.v
 
# Testbench
El codigo de prueba, dado en codigo.hex se utiliza en RV32nexpo_tb.v. 
Este codigo de prueba no es muy bueno, y actualmente no está subido el codigo fuente.

pipev2.txt puede verse lo simulado a través de Icarus Verilog. 
En pipelining2.gtkw se puede observar las formas de onda predeterminadas guardads en dump.vcd
