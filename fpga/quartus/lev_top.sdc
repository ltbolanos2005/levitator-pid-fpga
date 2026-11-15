# Restricciones de tiempo (reloj de 50 MHz). Agregar este archivo al proyecto.
create_clock -name clk -period 20.000 [get_ports clk]
derive_clock_uncertainty
# Entradas/salidas asincronas (boton, UART del Arduino, UART MODBUS, LEDs):
# no se analizan en tiempo (las entradas pasan por sincronizadores de 2 etapas)
set_false_path -from [get_ports {rst_n_in ard_rx mb_rx}]
set_false_path -to   [get_ports {ard_tx mb_tx mb_de leds[*]}]
