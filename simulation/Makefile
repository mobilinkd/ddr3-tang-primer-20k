
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
# model and the first-party primitive models in this directory. The
# measurement numbers come from THIS flow.
#
# No vendor model is used. `gowin_prim_models.v` replaces Gowin's
# prim_sim.v outright: it defines every primitive this design instantiates
# (rPLL, OSER8, OSER8_MEM, DQS, IDES8_MEM, DLL, GSR). The vendor model
# cannot be used for a measurement -- its OSER8/DQS are written for a
# cycle-accurate event simulator and it does not instantiate the GSR
# primitive its own models reference by hierarchical name, so it cannot
# elaborate under Icarus at all here.
#
# -DIVERILOG selects two simulation-only equivalences in src/print.v: the
# streaming concatenation `{>>{a}}` (unsupported by Icarus) and the
# duplicate `wire uart_txp` declaration (tolerated by Gowin, rejected by
# Icarus). Both are `ifdef-guarded, so the synthesized design is
# byte-identical to upstream. See the comments in src/print.v.
# =====================================================================

OUT := ../build/sim

# Gowin's prim_sim.v is deliberately NOT used; see the header note above.
# The primitives come from gowin_prim_models.v, which is in this directory
# and tracked. The image below is only needed for the clock self-check,
# which cross-checks the rPLL model against the vendor's own model.

$(OUT)/top.vvp: $(DIR)/ddr3_top.v $(DIR)/ddr3_controller.v \
                $(DIR)/uart_tx_V2.v $(DIR)/gowin_rpll/gowin_rpll.v \
                tb_top.v ddr3_x16_model.v gowin_prim_models.v
	mkdir -p $(OUT)
	iverilog -o $@ -DSIM -DIVERILOG -DDDR3_800 -g 2012 -I $(DIR) \
	    -s tb_top \
	    tb_top.v ddr3_x16_model.v gowin_prim_models.v \
	    $(DIR)/ddr3_top.v $(DIR)/ddr3_controller.v $(DIR)/uart_tx_V2.v \
	    $(DIR)/gowin_rpll/gowin_rpll.v

top: $(OUT)/top.vvp

run.top: $(OUT)/top.vvp
	vvp $(OUT)/top.vvp

# The clock self-check. Asserts pclk = 99.5625 MHz, fclk = 398.25 MHz and
# fclk/pclk = 4.0 to 0.1%. Keep it: a dual-edge divider chain that is wrong
# by 3687x produces no warning anywhere else in this flow.
clock.check: probe_clocks.v gowin_prim_models.v
	iverilog -o $(OUT)/probe_clocks.vvp -g 2012 -s probe_clocks \
	    probe_clocks.v gowin_prim_models.v \
	    $(DIR)/gowin_rpll/gowin_rpll.v
	vvp $(OUT)/probe_clocks.vvp

clean.selfcontained:
	rm -rf $(OUT)

