# Confluence page 15073283, V5 (checked 2026-10-02 KST).
# Zybo Z7-10, 3.3 V Pmod logic. GND is a physical connection, not an HDL port.
# https://raw.githubusercontent.com/Digilent/digilent-xdc/master/Zybo-Z7-Master.xdc

set_property -dict {PACKAGE_PIN V15 IOSTANDARD LVCMOS33 PULLUP true} [get_ports servo_uart_rx]
set_property -dict {PACKAGE_PIN W15 IOSTANDARD LVCMOS33 DRIVE 8 SLEW SLOW} [get_ports servo_uart_tx]
set_property -dict {PACKAGE_PIN T14 IOSTANDARD LVCMOS33 DRIVE 8 SLEW SLOW PULLDOWN true} [get_ports laser_enable]

# UART is asynchronous to the PS clock; the PS peripheral performs sampling.
# Limit false paths to this UART input, rather than suppressing other timing checks.
set_false_path -from [get_ports servo_uart_rx]
