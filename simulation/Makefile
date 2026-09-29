
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

# TB_BULK shrinks the measured region in WORDS (16 B words). It exists only
# to bring the datapath up quickly; a reported number must come from a run
# with TB_BULK unset, which is the full 64 Ki-word region. TB_MON_GAP sets the
# stall-monitor period in ps. The $display traffic in that monitor dominates
# Icarus wall time, so a long run sets it high.
TBFLAGS ?=

$(OUT)/top.vvp: $(DIR)/ddr3_top.v $(DIR)/ddr3_controller.v \
                $(DIR)/uart_tx_V2.v $(DIR)/gowin_rpll/gowin_rpll.v \
                tb_top.v ddr3_x16_model.v gowin_prim_models.v
	mkdir -p $(OUT)
	iverilog -o $@ -DSIM -DIVERILOG -DDDR3_800 -g 2012 -I $(DIR) $(TBFLAGS) \
	    -s tb_top \
	    tb_top.v ddr3_x16_model.v gowin_prim_models.v \
	    $(DIR)/ddr3_top.v $(DIR)/ddr3_controller.v $(DIR)/uart_tx_V2.v \
	    $(DIR)/gowin_rpll/gowin_rpll.v

top: $(OUT)/top.vvp

run.top: $(OUT)/top.vvp
	vvp $(OUT)/top.vvp

# ---------------------------------------------------------------------
# rb : the controller-level read-cadence bench.
#
# The `top` flow above is the measurement of record for the end-to-end DUT
# (it captures over the real UART and exercises the write path). But it
# simulates WIPE, WRITE_BLOCK and the legacy VERIFY_BLOCK first -- a fixed
# ~13 ms of simulated time -- and Icarus runs this design at a few thousand
# pclk per second, so an engine iteration costs many minutes. tb_rb.v drives
# the SAME ddr3_controller in the SAME fast_mode against the SAME DDR3 model
# and reports the SAME on-chip counters, in a run that takes seconds.
#
# It emits its snapshots in the UART wire format, so the rate is computed by
# tools/decode_uart.py -- never by the bench. That is why --only READ_BURST
# is passed: a controller-level run has no write phases, and the decoder
# would otherwise reject it for zero write deltas.
#
# WHAT THIS DOES AND DOES NOT PROVE. It measures the queue, the cadence, the
# address decode, the row tracker and the response bookkeeping. It does NOT
# prove the read data path works on silicon at 3 pclk/cmd, because
# gowin_prim_models.v drives DQSR90 = FCLK unconditionally and ignores both
# READ and HOLD: in simulation the DQS primitive is always clocked, so
# dqs_read cannot affect what comes back. The 128-bit data check in
# ddr3_top's READ_BURST phase is what exercises data, and the bench is what
# measures the cadence. Settling the primitive's behaviour is bench work.
$(OUT)/rb.vvp: $(DIR)/ddr3_controller.v $(DIR)/gowin_rpll/gowin_rpll.v \
                tb_rb.v ddr3_x16_model.v gowin_prim_models.v
	mkdir -p $(OUT)
	iverilog -o $@ -DSIM -DIVERILOG -DDDR3_800 -g 2012 -I $(DIR) $(TBFLAGS) \
	    -s tb_rb \
	    tb_rb.v ddr3_x16_model.v gowin_prim_models.v \
	    $(DIR)/ddr3_controller.v $(DIR)/gowin_rpll/gowin_rpll.v

rb: $(OUT)/rb.vvp

# run.rb also decodes, and writes the raw capture and the JSON next to it,
# so a reported number always has its evidence committed beside it.
run.rb: $(OUT)/rb.vvp ../tools/decode_uart.py
	mkdir -p ../evidence
	vvp $(OUT)/rb.vvp | tee ../evidence/sim-read-400mbs.log
	python3 ../tools/decode_uart.py ../evidence/sim-read-400mbs.log \
	    --only READ_BURST -o ../evidence/sim-read-400mbs.json

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

# ---------------------------------------------------------------------------
# tb_row: the row-field correctness bench.
#
# It is separate from `top` because it answers a different question: not
# "what is the rate" but "does the engine activate and read the row it was
# asked for". 6c9d94b shipped a q_row slice of [22:13] instead of [22:10],
# which aliased 8 consecutive rows onto one value. That bug is invisible to
# a cadence bench -- need_act stays self-consistent because row_open and A[0]
# are both loaded from the same q_row wire -- so the only way to see it is to
# compare returned data against the address that produced it. See the header
# of tb_row.v.
#
# NROWS is 16, deliberately > 8: with <= 8 rows the alias is invisible.
# ---------------------------------------------------------------------------
$(OUT)/row.vvp: $(DIR)/ddr3_controller.v tb_row.v ddr3_x16_model.v gowin_prim_models.v
	mkdir -p $(OUT)
	iverilog -o $@ -DSIM -DIVERILOG -DDDR3_800 -g 2012 -I $(DIR) \
	    -s tb_row \
	    tb_row.v ddr3_x16_model.v gowin_prim_models.v \
	    $(DIR)/ddr3_controller.v $(DIR)/gowin_rpll/gowin_rpll.v

row: $(OUT)/row.vvp
	vvp $(OUT)/row.vvp

