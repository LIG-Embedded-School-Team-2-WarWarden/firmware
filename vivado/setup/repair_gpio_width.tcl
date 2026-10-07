# Correct the PS7 user selection that controls the derived EMIO GPIO width.
proc repair_and_build {} {
    set project_dir /home/user/servo_test
    set verify_dir [file join $project_dir verification]
    set_param general.maxThreads 4
    open_project [file join $project_dir servo_test.xpr]
    open_bd_design [get_files */servo_system.bd]
    set ps [get_bd_cells processing_system7_0]
    if {[get_property CONFIG.PCW_UART0_UART0_IO $ps] ne "EMIO"} {error "Unexpected UART0 mapping"}
    set_property CONFIG.PCW_GPIO_EMIO_GPIO_IO {1} $ps
    validate_bd_design
    if {[get_property CONFIG.PCW_GPIO_EMIO_GPIO_WIDTH $ps] != 1} {error "GPIO width callback did not produce 1"}
    save_bd_design
    close_bd_design [current_bd_design]
    open_bd_design [get_files */servo_system.bd]
    set ps [get_bd_cells processing_system7_0]
    foreach prop {PCW_UART0_PERIPHERAL_ENABLE PCW_UART0_UART0_IO PCW_UART0_BAUD_RATE PCW_UART1_PERIPHERAL_ENABLE PCW_UART1_UART1_IO PCW_GPIO_EMIO_GPIO_ENABLE PCW_GPIO_EMIO_GPIO_IO PCW_GPIO_EMIO_GPIO_WIDTH} {
        puts "PS_SETTING $prop=[get_property CONFIG.$prop $ps]"
    }
    if {[get_property CONFIG.PCW_GPIO_EMIO_GPIO_WIDTH $ps] != 1 || [get_property CONFIG.PCW_UART1_UART1_IO $ps] ne "MIO 48 .. 49"} {error "Saved PS settings did not match"}
    write_bd_tcl -force [file join $verify_dir servo_system.tcl]
    set bd [get_files */servo_system.bd]
    generate_target all $bd
    set wrapper [make_wrapper -files $bd -top]
    set_property top servo_system_wrapper [get_filesets sources_1]
    update_compile_order -fileset sources_1
    reset_run synth_1
    launch_runs synth_1 -jobs 4
    wait_on_run synth_1
    if {![string match "*Complete*" [get_property STATUS [get_runs synth_1]]]} {error "Synthesis failed"}
    puts "PINMAP_STAGE SYNTHESIS_OK"
    launch_runs impl_1 -to_step write_bitstream -jobs 4
    wait_on_run impl_1
    if {![string match "*Complete*" [get_property STATUS [get_runs impl_1]]]} {error "Implementation failed"}
    open_run impl_1
    foreach {port pin direction} {servo_uart_rx V15 IN servo_uart_tx W15 OUT laser_enable T14 OUT} {
        set obj [get_ports $port]
        if {[llength $obj] != 1 || [get_property PACKAGE_PIN $obj] ne $pin || [get_property IOSTANDARD $obj] ne "LVCMOS33" || [get_property DIRECTION $obj] ne $direction} {error "Implemented pin mismatch: $port"}
        puts "PINMAP_VERIFIED $port $pin LVCMOS33 $direction"
    }
    report_io -file [file join $verify_dir implemented_io.rpt] -force
    report_drc -file [file join $verify_dir implemented_drc.rpt] -force
    report_timing_summary -file [file join $verify_dir implemented_timing.rpt]
    set severe [get_drc_violations -quiet -filter {SEVERITY == Error || SEVERITY == {Critical Warning}}]
    if {[llength $severe] != 0} {error "Unresolved severe DRC violations: $severe"}
    write_hw_platform -fixed -include_bit -force -file [file join $project_dir servo_test.xsa]
    puts "PINMAP_STAGE IMPLEMENTATION_AND_XSA_OK"
    close_project
}
if {[catch {repair_and_build} problem]} {
    puts stderr "PINMAP_REPAIR_FAILED: $problem"
    puts stderr $::errorInfo
    exit 1
}
puts "PINMAP_SETUP_OK"
exit 0
