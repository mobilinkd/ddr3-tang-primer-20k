# build_ddr3.tcl -- unattended headless gw_sh flow for primer-ddr3-controller.
#
# Targets the Tang Primer 20K (GW2A-LV18PG256C8/I7). Synthesises, places,
# routes, packs to a Gowin .fs bitstream.
#
# INVOCATION -- inside the fpga-tools container, via the gw wrapper:
#
#     tools/build.sh
#
# (tools/build.sh sets HOST_REPO_DIR, mounts the repo at that exact path,
#  launches tools/fpga-run.sh, and invokes `gw gw_sh -exit build_ddr3.tcl`
#  in the repo root.)
#
# Source list mirrors ddr3.gprj PLUS src/print.v because ddr3_top.v
# `include`s it (the Gowin GUI project file does not list it; the IDE
# auto-includes `\`include`d files but gw_sh does not -- so it MUST be
# here or elaboration breaks on undefined `tx`, `print_clk`, etc.).
#
# gowin_rpll_tmp.v is intentionally EXCLUDED -- orphan backup file, not
# referenced by any other file.
#
# Output: $REPO/build/ddr3/proj/ddr3/impl/pnr/ddr3.fs
#
# Success marker (last line): DDR3-BUILD-DONE

if {![info exists ::env(REPO)]} {
    # tools/build.sh always sets REPO, but this fallback lets the script be
    # run by hand for debugging.
    set REPO [file normalize [file join [pwd] ..]]
} else {
    set REPO $::env(REPO)
}

set projdir [file join $REPO build ddr3 proj]
file mkdir $projdir
cd $projdir

create_project -name ddr3 -dir $projdir \
    -pn GW2A-LV18PG256C8/I7 -device_version C -force

set src [file join $REPO src]
add_file -type verilog [file join $src ddr3_controller.v]
add_file -type verilog [file join $src ddr3_top.v]
add_file -type verilog [file join $src gowin_rpll gowin_rpll.v]
add_file -type verilog [file join $src uart_tx_V2.v]
add_file -type verilog [file join $src print.v]
add_file -type cst     [file join $src tang20k.cst]
add_file -type sdc     [file join $src ddr3.sdc]

set_option -top_module ddr3_top
# The .gprj says Verilog, but ddr3_top.v uses `include "print.v" and
# nested `define/`print macros; SystemVerilog-2017 elaborates these.
# (sv2017 is what the existing Gowin project picks.)
set_option -verilog_std sysv2017

run all

puts "DDR3-BUILD-DONE"