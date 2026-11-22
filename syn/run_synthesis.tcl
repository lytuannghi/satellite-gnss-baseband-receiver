# ==============================================================================
# Design Compiler Synthesis Script
# Design: satellite_gnss_receiver_top
# Target Library: TSMC 28nm HPC+ 0.9V Low-Power Logic Standard Cells
# ==============================================================================

set DESIGN_NAME "satellite_gnss_receiver_top"

set_app_var search_path ". ./rtl ./rtl/prn_gen ./rtl/nco ./rtl/correlator ./rtl/channel ./rtl/bus /tools/synopsys/libs"
set_app_var target_library "tcbn28hpcplus_ss0p9v125c.db"
set_app_var link_library "* tcbn28hpcplus_ss0p9v125c.db tcbn28hpcplus_ff0p99v0c.db"

# Analyze Verilog RTL Sources
analyze -format verilog {
    ./rtl/prn_gen/gold_code_generator.v
    ./rtl/nco/carrier_code_nco.v
    ./rtl/correlator/early_prompt_late_correlator.v
    ./rtl/channel/gnss_tracking_channel.v
    ./rtl/bus/gnss_regs_apb.v
    ./rtl/satellite_gnss_receiver_top.v
}

# Elaborate Top-Level Module
elaborate $DESIGN_NAME

# Apply Timing Constraints
source ./syn/gnss_timing.sdc

# Check Unresolved References and Constraints
check_design > ./syn/reports/check_design.rpt
check_timing > ./syn/reports/check_timing.rpt

# Compile and Map Logic
compile_ultra -gate_clock -scan

# Generate Reports
report_timing -delay_type max -max_paths 10 > ./syn/reports/timing_report.txt
report_timing -delay_type min -max_paths 10 >> ./syn/reports/timing_report.txt
report_area -hierarchy > ./syn/reports/area_report.txt
report_power -hierarchy > ./syn/reports/power_report.txt
report_qor > ./syn/reports/qor_report.txt

# Export Synthesized Gate-Level Netlist and Constraints
write -format verilog -hierarchy -output ./syn/netlist/satellite_gnss_receiver_top_syn.v
write_sdc ./syn/netlist/satellite_gnss_receiver_top_syn.sdc

puts "[INFO] Synthesis completed successfully for $DESIGN_NAME."
