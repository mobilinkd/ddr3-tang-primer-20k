
DIR := ../src

controller.vvp: ddr3.v $(DIR)/ddr3_controller.v tb_controller.v prim_sim.v
	iverilog -o controller.vvp -D SIM -D den1024Mb -D sg25 -D x16 -g 2012 tb_controller.v ddr3.v $(DIR)/ddr3_controller.v prim_sim.v

controller: controller.vvp

run.controller: controller.vvp
	if not exist tmp mkdir tmp
	vvp controller.vvp

# Vanilla Micron DDR3 simulation
ddr3.vvp: ddr3.v 1024*.vh tb.v subtest.vh
	iverilog -o ddr3.vvp -D den1024Mb -D sg25 -D x16 -g 2012 tb.v ddr3.v

ddr3: ddr3.vvp

run.ddr3: ddr3.vvp
	if not exist tmp mkdir tmp
	vvp ddr3.vvp

# Misc
tag: *.v *.vh
	ctags -R *.v 1024*

clean:
	del *.vvp

# =====================================================================
# Self-contained flow, added 2026-09-28 (dave) for the 400 MB/s work.
#
# The targets above are upstream's, unchanged. They need Micron's
# ddr3.v / tb.v / subtest.vh / 1024*.vh and Gowin's prim_sim.v dropped
# into this directory by hand, as simulation/README.md describes. None of
# those are in the repo, so that flow cannot run from a clean checkout.
#
# The flow below needs nothing downloaded: it uses the behavioural DDR3
# model in this directory and the vendor prim_sim.v that already ships
# inside the fpga-sim container image. The measurement numbers come from
# THIS flow, and tools/run-sim.sh drives it.
#
# -DIVERILOG selects two simulation-only equivalences in src/print.v: the
# streaming concatenation `{>>{a}}` (unsupported by Icarus) and the
# duplicate `wire uart_txp` declaration (tolerated by Gowin, rejected by
# Icarus). Both are `ifdef-guarded, so the synthesized design is
# byte-identical to upstream. See the comments in src/print.v.
# =====================================================================

OUT := ../build/sim
SIM_IMAGE ?= localhost/fpga-sim

# Gowin's behavioural primitive models, inside the image.
PRIM_SIM_IN_IMAGE ?= /opt/gowin/IDE/simlib/gw2a/prim_sim.v

$(OUT)/prim_sim.v:
	mkdir -p $(OUT)
	podman run --rm $(SIM_IMAGE) bash -lc 'cat $(PRIM_SIM_IN_IMAGE)' > $@

$(OUT)/top.vvp: $(DIR)/ddr3_top.v $(DIR)/ddr3_controller.v \
                $(DIR)/uart_tx_V2.v $(DIR)/gowin_rpll/gowin_rpll.v \
                tb_top.v ddr3_x16_model.v $(OUT)/prim_sim.v
	mkdir -p $(OUT)
	iverilog -o $@ -DSIM -DIVERILOG -DDDR3_800 -g 2012 -I $(DIR) \
	    -s tb_top \
	    tb_top.v ddr3_x16_model.v \
	    $(DIR)/ddr3_top.v $(DIR)/ddr3_controller.v $(DIR)/uart_tx_V2.v \
	    $(DIR)/gowin_rpll/gowin_rpll.v $(OUT)/prim_sim.v

top: $(OUT)/top.vvp

run.top: $(OUT)/top.vvp
	vvp $(OUT)/top.vvp

clean.selfcontained:
	rm -rf $(OUT)

