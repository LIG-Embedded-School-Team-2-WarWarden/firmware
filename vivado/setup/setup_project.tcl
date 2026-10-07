# Vivado 2022.1. Configure the existing, empty servo_test project.
# Run from the VM's internal copy of this folder after closing its GUI project.
proc configure_servo_project {} {
    set assets_dir [file normalize [file dirname [info script]]]
    set project_file /home/user/servo_test/servo_test.xpr
    set project_dir [file dirname $project_file]
    set verify_dir [file join $project_dir verification]
    file mkdir $verify_dir
    set_param board.repoPaths [list /home/user/.Xilinx/Vivado/2022.1/xhub/board_store/xilinx_board_store]
    set_param general.maxThreads 4
    open_project $project_file
    if {[get_property PART [current_project]] ne "xc7z010clg400-1"} {error "Unexpected target chip"}
    if {[llength [get_files -quiet -of_objects [get_filesets sources_1]]] != 0} {
        error "Project is no longer empty; refusing to replace existing design sources"
    }
    set boards [get_board_parts -quiet digilentinc.com:zybo-z7-10:part0:1.1]
    if {[llength $boards] != 1} {error "Expected Zybo Z7-10 preset not found"}
    set_property board_part [lindex $boards 0] [current_project]
    set_property target_language Verilog [current_project]
    add_files -norecurse [file join $assets_dir laser_gpio_gate.v]
    add_files -fileset constrs_1 -norecurse [file join $assets_dir servo_pinmap.xdc]
    create_bd_design servo_system
    create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 processing_system7_0
    apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 -config {make_external "FIXED_IO, DDR" apply_board_preset "1" Master "Disable" Slave "Disable"} [get_bd_cells processing_system7_0]
    set ps [get_bd_cells processing_system7_0]
    set_property -dict [list \
        CONFIG.PCW_UART0_PERIPHERAL_ENABLE {1} \
        CONFIG.PCW_UART0_UART0_IO {EMIO} \
        CONFIG.PCW_UART0_GRP_FULL_ENABLE {0} \
        CONFIG.PCW_GPIO_PERIPHERAL_ENABLE {1} \
        CONFIG.PCW_GPIO_EMIO_GPIO_ENABLE {1} \
        CONFIG.PCW_USE_M_AXI_GP0 {0} \
        CONFIG.PCW_EN_CLK0_PORT {0} \
        CONFIG.PCW_EN_RST0_PORT {0}] $ps
    # WIDTH is derived by PS7 callbacks; select the number through GPIO_IO.
    set_property CONFIG.PCW_GPIO_EMIO_GPIO_IO {1} $ps
    # UART1 MIO48/49 and the board DDR/Ethernet/USB/SD presets stay enabled.
    if {[get_property CONFIG.PCW_UART1_UART1_IO $ps] ne "MIO 48 .. 49"} {
        error "Board USB UART console mapping changed unexpectedly"
    }
    create_bd_port -dir I servo_uart_rx
    create_bd_port -dir O servo_uart_tx
    create_bd_port -dir O laser_enable
    connect_bd_net [get_bd_ports servo_uart_rx] [get_bd_pins processing_system7_0/UART0_RX]
    connect_bd_net [get_bd_ports servo_uart_tx] [get_bd_pins processing_system7_0/UART0_TX]
    create_bd_cell -type module -reference laser_gpio_gate laser_gpio_gate_0
    connect_bd_net [get_bd_pins processing_system7_0/GPIO_O] [get_bd_pins laser_gpio_gate_0/request]
    connect_bd_net [get_bd_pins processing_system7_0/GPIO_T] [get_bd_pins laser_gpio_gate_0/tri_state]
    connect_bd_net [get_bd_pins processing_system7_0/GPIO_I] [get_bd_pins laser_gpio_gate_0/sense]
    connect_bd_net [get_bd_ports laser_enable] [get_bd_pins laser_gpio_gate_0/laser_enable]
    validate_bd_design
    if {[get_property CONFIG.PCW_GPIO_EMIO_GPIO_WIDTH $ps] != 1} {
        error "GPIO EMIO width did not remain 1 after validation"
    }
    save_bd_design
    write_bd_tcl -force [file join $verify_dir servo_system.tcl]
    set bd [get_files */servo_system.bd]
    generate_target all $bd
    set wrapper [make_wrapper -files $bd -top]
    add_files -norecurse $wrapper
    set_property top servo_system_wrapper [get_filesets sources_1]
    update_compile_order -fileset sources_1
    puts "PINMAP_STAGE CONFIGURED"
    launch_runs synth_1 -jobs 4
    wait_on_run synth_1
    if {![string match "*Complete*" [get_property STATUS [get_runs synth_1]]]} {
        error "Synthesis did not complete: [get_property STATUS [get_runs synth_1]]"
    }
    puts "PINMAP_STAGE SYNTHESIS_OK"
    launch_runs impl_1 -to_step write_bitstream -jobs 4
    wait_on_run impl_1
    if {![string match "*Complete*" [get_property STATUS [get_runs impl_1]]]} {
        error "Implementation did not complete: [get_property STATUS [get_runs impl_1]]"
    }
    open_run impl_1
    foreach {port pin direction} {servo_uart_rx V15 IN servo_uart_tx W15 OUT laser_enable T14 OUT} {
        set obj [get_ports $port]
        if {[llength $obj] != 1 || [get_property PACKAGE_PIN $obj] ne $pin || [get_property IOSTANDARD $obj] ne "LVCMOS33" || [get_property DIRECTION $obj] ne $direction} {
            error "Implemented pin mismatch for $port"
        }
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
if {[catch {configure_servo_project} problem]} {
    puts stderr "PINMAP_SETUP_FAILED: $problem"
    puts stderr $::errorInfo
    exit 1
}
puts "PINMAP_SETUP_OK"
exit 0
