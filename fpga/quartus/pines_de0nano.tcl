# =====================================================================
# Pines del levitador para la Terasic DE0-Nano (EP4CE22F17C6)
# Quartus II 13.1: con el proyecto abierto -> Tools > Tcl Scripts...
#                  -> seleccionar este archivo -> Run
# =====================================================================
set_location_assignment PIN_R8  -to clk
set_location_assignment PIN_J15 -to rst_n_in
set_location_assignment PIN_D3  -to ard_rx
set_location_assignment PIN_C3  -to ard_tx
set_location_assignment PIN_A2  -to mb_rx
set_location_assignment PIN_A3  -to mb_tx
set_location_assignment PIN_B3  -to mb_de
set_location_assignment PIN_A15 -to {leds[0]}
set_location_assignment PIN_A13 -to {leds[1]}
set_location_assignment PIN_B13 -to {leds[2]}
set_location_assignment PIN_A11 -to {leds[3]}
set_location_assignment PIN_D1  -to {leds[4]}
set_location_assignment PIN_F3  -to {leds[5]}

foreach s {clk rst_n_in ard_rx ard_tx mb_rx mb_tx mb_de leds[0] leds[1] leds[2] leds[3] leds[4] leds[5]} {
    set_instance_assignment -name IO_STANDARD "3.3-V LVTTL" -to $s
}
set_instance_assignment -name WEAK_PULL_UP_RESISTOR ON -to mb_rx
set_instance_assignment -name WEAK_PULL_UP_RESISTOR ON -to ard_rx

export_assignments
puts "Pines del levitador asignados."
