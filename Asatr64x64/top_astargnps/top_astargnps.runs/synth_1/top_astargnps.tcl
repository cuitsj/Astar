# 
# Synthesis run script generated by Vivado
# 

set TIME_start [clock seconds] 
proc create_report { reportName command } {
  set status "."
  append status $reportName ".fail"
  if { [file exists $status] } {
    eval file delete [glob $status]
  }
  send_msg_id runtcl-4 info "Executing : $command"
  set retval [eval catch { $command } msg]
  if { $retval != 0 } {
    set fp [open $status w]
    close $fp
    send_msg_id runtcl-5 warning "$msg"
  }
}
set_param tcl.collectionResultDisplayLimit 0
set_param xicom.use_bs_reader 1
create_project -in_memory -part xc7vx485tffg1761-2

set_param project.singleFileAddWarning.threshold 0
set_param project.compositeFile.enableAutoGeneration 0
set_param synth.vivado.isSynthRun true
set_msg_config -source 4 -id {IP_Flow 19-2162} -severity warning -new_severity info
set_property webtalk.parent_dir G:/01Astar/Asatr64x64/top_astargnps/top_astargnps.cache/wt [current_project]
set_property parent.project_path G:/01Astar/Asatr64x64/top_astargnps/top_astargnps.xpr [current_project]
set_property XPM_LIBRARIES {XPM_CDC XPM_MEMORY} [current_project]
set_property default_lib xil_defaultlib [current_project]
set_property target_language Verilog [current_project]
set_property ip_output_repo g:/01Astar/Asatr64x64/top_astargnps/top_astargnps.cache/ip [current_project]
set_property ip_cache_permissions {read write} [current_project]
read_verilog -library xil_defaultlib {
  G:/01Astar/Asatr64x64/top_astargnps/top_astargnps.srcs/sources_1/new/astar_gnps.v
  G:/01Astar/Asatr64x64/top_astargnps/top_astargnps.srcs/sources_1/new/uart_rx.v
  G:/01Astar/Asatr64x64/top_astargnps/top_astargnps.srcs/sources_1/new/uart_tx.v
  G:/01Astar/Asatr64x64/top_astargnps/top_astargnps.srcs/sources_1/new/top_astargnps.v
}
read_ip -quiet G:/01Astar/Asatr64x64/top_astargnps/top_astargnps.srcs/sources_1/ip/openram12/openram12.xci
set_property used_in_implementation false [get_files -all g:/01Astar/Asatr64x64/top_astargnps/top_astargnps.srcs/sources_1/ip/openram12/openram12_ooc.xdc]

read_ip -quiet G:/01Astar/Asatr64x64/top_astargnps/top_astargnps.srcs/sources_1/ip/openram8/openram8.xci
set_property used_in_implementation false [get_files -all g:/01Astar/Asatr64x64/top_astargnps/top_astargnps.srcs/sources_1/ip/openram8/openram8_ooc.xdc]

read_ip -quiet G:/01Astar/Asatr64x64/top_astargnps/top_astargnps.srcs/sources_1/ip/closeram8/closeram8.xci
set_property used_in_implementation false [get_files -all g:/01Astar/Asatr64x64/top_astargnps/top_astargnps.srcs/sources_1/ip/closeram8/closeram8_ooc.xdc]

read_ip -quiet G:/01Astar/Asatr64x64/top_astargnps/top_astargnps.srcs/sources_1/ip/mapram1_1/mapram1.xci
set_property used_in_implementation false [get_files -all g:/01Astar/Asatr64x64/top_astargnps/top_astargnps.srcs/sources_1/ip/mapram1_1/mapram1_ooc.xdc]

read_ip -quiet G:/01Astar/Asatr64x64/top_astargnps/top_astargnps.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0.xci
set_property used_in_implementation false [get_files -all g:/01Astar/Asatr64x64/top_astargnps/top_astargnps.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0_board.xdc]
set_property used_in_implementation false [get_files -all g:/01Astar/Asatr64x64/top_astargnps/top_astargnps.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0.xdc]
set_property used_in_implementation false [get_files -all g:/01Astar/Asatr64x64/top_astargnps/top_astargnps.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0_ooc.xdc]

read_ip -quiet G:/01Astar/Asatr64x64/top_astargnps/top_astargnps.srcs/sources_1/ip/ila_astar/ila_astar.xci
set_property used_in_synthesis false [get_files -all g:/01Astar/Asatr64x64/top_astargnps/top_astargnps.srcs/sources_1/ip/ila_astar/ila_v6_2/constraints/ila_impl.xdc]
set_property used_in_implementation false [get_files -all g:/01Astar/Asatr64x64/top_astargnps/top_astargnps.srcs/sources_1/ip/ila_astar/ila_v6_2/constraints/ila_impl.xdc]
set_property used_in_implementation false [get_files -all g:/01Astar/Asatr64x64/top_astargnps/top_astargnps.srcs/sources_1/ip/ila_astar/ila_v6_2/constraints/ila.xdc]
set_property used_in_implementation false [get_files -all g:/01Astar/Asatr64x64/top_astargnps/top_astargnps.srcs/sources_1/ip/ila_astar/ila_astar_ooc.xdc]

# Mark all dcp files as not used in implementation to prevent them from being
# stitched into the results of this synthesis run. Any black boxes in the
# design are intentionally left as such for best results. Dcp files will be
# stitched into the design at a later time, either when this synthesis run is
# opened, or when it is stitched into a dependent implementation run.
foreach dcp [get_files -quiet -all -filter file_type=="Design\ Checkpoint"] {
  set_property used_in_implementation false $dcp
}
read_xdc G:/01Astar/Asatr64x64/top_astargnps/top_astargnps.srcs/constrs_1/new/io_plan.xdc
set_property used_in_implementation false [get_files G:/01Astar/Asatr64x64/top_astargnps/top_astargnps.srcs/constrs_1/new/io_plan.xdc]

read_xdc dont_touch.xdc
set_property used_in_implementation false [get_files dont_touch.xdc]
set_param ips.enableIPCacheLiteLoad 1
close [open __synthesis_is_running__ w]

synth_design -top top_astargnps -part xc7vx485tffg1761-2


# disable binary constraint mode for synth run checkpoints
set_param constraints.enableBinaryConstraints false
write_checkpoint -force -noxdef top_astargnps.dcp
create_report "synth_1_synth_report_utilization_0" "report_utilization -file top_astargnps_utilization_synth.rpt -pb top_astargnps_utilization_synth.pb"
file delete __synthesis_is_running__
close [open __synthesis_is_complete__ w]
