proc finish_servo_project {} {
    set project_dir /home/user/servo_test
    set verify_dir [file join $project_dir verification]
    open_project [file join $project_dir servo_test.xpr]
    open_bd_design [get_files */servo_system.bd]
    set ps [get_bd_cells processing_system7_0]
    foreach prop {PCW_UART0_PERIPHERAL_ENABLE PCW_UART0_UART0_IO PCW_UART0_BAUD_RATE PCW_UART1_PERIPHERAL_ENABLE PCW_UART1_UART1_IO PCW_GPIO_EMIO_GPIO_ENABLE PCW_GPIO_EMIO_GPIO_WIDTH} {
        puts "PS_SETTING $prop=[get_property CONFIG.$prop $ps]"
    }
    if {[get_property CONFIG.PCW_UART0_UART0_IO $ps] ne "EMIO" || [get_property CONFIG.PCW_GPIO_EMIO_GPIO_WIDTH $ps] != 1} {error "Unexpected PS configuration"}
    open_run impl_1
    foreach {port pin direction} {servo_uart_rx V15 IN servo_uart_tx W15 OUT laser_enable T14 OUT} {
        set obj [get_ports $port]
        if {[llength $obj] != 1 || [get_property PACKAGE_PIN $obj] ne $pin || [get_property IOSTANDARD $obj] ne "LVCMOS33" || [get_property DIRECTION $obj] ne $direction} {error "Implemented pin mismatch: $port"}
        puts "PINMAP_VERIFIED $port $pin LVCMOS33 $direction"
    }
    report_drc -file [file join $verify_dir implemented_drc.rpt] -force
    report_timing_summary -file [file join $verify_dir implemented_timing.rpt]
    set severe [get_drc_violations -quiet -filter {SEVERITY == Error || SEVERITY == {Critical Warning}}]
    if {[llength $severe] != 0} {error "Unresolved severe DRC violations: $severe"}
    write_hw_platform -fixed -include_bit -force -file [file join $project_dir servo_test.xsa]
    close_project
}
if {[catch {finish_servo_project} problem]} {
    puts stderr "PINMAP_FINALIZE_FAILED: $problem"
    puts stderr $::errorInfo
    exit 1
}
puts "PINMAP_SETUP_OK"
exit 0
