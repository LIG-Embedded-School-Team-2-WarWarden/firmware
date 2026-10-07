`timescale 1ns / 1ps

// PS GPIO resets to input mode (tri_state=1). Keep the external output low
// until software enables output mode and explicitly requests a high level.
module laser_gpio_gate (
    input  wire request,
    input  wire tri_state,
    output wire sense,
    output wire laser_enable
);
    assign laser_enable = request & ~tri_state;
    assign sense = laser_enable;
endmodule
