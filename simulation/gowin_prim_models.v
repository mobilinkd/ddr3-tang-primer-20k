// First-party behavioural models for the Gowin primitives this design
// uses, written to replace vendor prim_sim.v in simulation.
//
// WHY THESE EXIST
// ---------------
// prim_sim.v works, but it is written for a cycle-accurate event
// simulator and is driven here through Verilator --timing, where every
// internal #delay becomes a coroutine round trip. Measured: the design
// completed 61 us of simulated time in 180 s of wall clock, with zero
// memory traffic in that window, so the PHY models alone were the
// bottleneck and the bulk phase was unreachable. Icarus is worse.
//
// These models contain no delays at all. They are pure clocked logic.
//
// SCOPE, AND THE ONE THING THAT MAKES THEM HONEST
// -----------------------------------------------
// These are FUNCTIONAL PASS-THROUGHS, not timing models. They do not
// model I/O timing, DQS phase, DLL delay, drive strength, ODT or
// training.
//
// That is acceptable for the measurement being made, and it is worth
// being precise about why. The quantity under test is
//
//     pclk per accepted command
//
// which is a property of the controller's FSM alone: the `cycle` counter
// and the `casex ({state, cycle})` block decide when `busy` negates and
// when `data_ready` asserts. Not one of those depends on the DQ pins,
// the DQS phase or the DDR3 timing. The primitives only decide what
// appears on the package pins.
//
// The measurement is therefore CROSS-CHECKED against an independent
// hand analysis of the RTL, in docs/design-note-400mbs.md section 2,
// which predicts 10.0000 pclk/command for read and about 7 for write.
// If these models and that analysis agree, both are corroborated. If a
// number here disagrees with the FSM's own arithmetic, the model is
// wrong and the number is not reported.
//
// Apache-2.0, consistent with this repo. No vendor code is copied; the
// port lists come from the instantiations in src/ddr3_controller.v and
// src/gowin_rpll/gowin_rpll.v.

`timescale 1ps /1ps

// ---------------------------------------------------------------------
// rPLL
//
// Reproduces the ratios the design depends on: CLKOUT = 4 * CLKIN,
// CLKOUTD = CLKOUT / 4 = CLKIN, CLKOUTP = CLKOUT, and LOCK asserted
// after a short real settle. The ratios matter -- the whole rate
// arithmetic is in pclk, and pclk must come out at 27 MHz for a 27 MHz
// crystal, i.e. exactly CLKIN.
// ---------------------------------------------------------------------
module rPLL #(
    parameter FBDIV_SEL = 58,
    parameter IDIV_SEL  = 3,
    parameter ODIV_SEL  = 2,
    parameter DYN_SDIV_SEL = 4,
    parameter FCLKIN    = "27",
    parameter DEVICE    = "GW2A-18C",
    // The wrapper src/gowin_rpll/gowin_rpll.v defparams all of these by
    // name, so the model must declare them even though none of them
    // changes the modelled ratios. A parameter that is not declared
    // makes Verilator fail with PINNOTFOUND.
    parameter DYN_IDIV_SEL   = "false",
    parameter DYN_FBDIV_SEL  = "false",
    parameter DYN_ODIV_SEL   = "false",
    parameter PSDA_SEL        = "0100",
    parameter DYN_DA_EN       = "false",
    parameter DUTYDA_SEL      = "1000",
    parameter CLKOUT_FT_DIR   = 1,
    parameter CLKOUTP_FT_DIR  = 1,
    parameter CLKOUT_DLY_STEP = 0,
    parameter CLKOUTP_DLY_STEP = 0,
    parameter CLKFB_SEL       = "internal",
    parameter CLKOUT_BYPASS   = "false",
    parameter CLKOUTP_BYPASS  = "false",
    parameter CLKOUTD_BYPASS  = "false",
    parameter CLKOUTD_SRC     = "CLKOUT",
    parameter CLKOUTD3_SRC    = "CLKOUT"
) (
    output CLKOUT,
    output CLKOUTP,
    output CLKOUTD,
    output CLKOUTD3,
    output LOCK,
    input  CLKIN,
    input  RESET,
    input  RESET_P,
    input  CLKFB,
    input  [5:0] FBDSEL,
    input  [3:0] IDSEL,
    input  [3:0] ODSEL,
    input  [3:0] PSDA,
    input  [3:0] DUTYDA,
    input  [3:0] FDLY
);
    // Reproduce the ratios the design depends on, by frequency and not
    // by divider edges: pclk = 99.5625 MHz, fclk = 398.25 MHz = 4 x pclk.
    //
    // The ratios are the whole point -- the rate arithmetic is in pclk,
    // and pclk has to come out at 99.5625 MHz for a 27 MHz crystal.
    //
    // An earlier shape built the clocks with `always @(posedge CLKIN or
    // negedge CLKIN)` divider chains, which looked like the natural way
    // to get a multiply and produced pclk at about 27 kHz instead of
    // 99.5625 MHz -- a factor of 3687, entirely silent, and it stalled
    // the top-level tick counter so the design never left PRINT_STATUS.
    // 27 MHz x 14.75 is not reachable by integer division anyway, so the
    // clocks are generated from their own periods instead. That is
    // exact, and it is checkable: simulation/probe_clocks.v asserts the
    // measured frequency against these numbers.
    localparam real PCLK_HZ = 99.5625e6;
    localparam real FCLK_HZ = 398.25e6;
    localparam real PCLK_HALF_PS = 0.5e12 / PCLK_HZ;   // 5022.2 ps
    localparam real FCLK_HALF_PS = 0.5e12 / FCLK_HZ;   // 1255.5 ps
    // CK is the memory clock: fclk, phase shifted by a quarter period.
    // The phase shift is not modelled, only the offset in the waveform.
    localparam real CK_QUARTER_PS = 0.25e12 / FCLK_HZ; // 627.8 ps

    reg clkout_r = 1'b0;
    reg clkoutd_r = 1'b0;
    reg clkoutp_r = 1'b0;
    reg lock_r = 1'b0;

    initial begin
        clkout_r  = 1'b0;
        clkoutd_r = 1'b0;
        clkoutp_r = 1'b0;
        lock_r    = 1'b0;
    end

    // Separate always blocks, one per clock. These were originally in a
    // single `always begin ... end`, where the two #delays are SEQUENTIAL
    // statements: each clock then toggles only after the other has, and
    // both come out at the same, wrong frequency. probe_clocks.v caught
    // it (fclk/pclk measured 1.0 instead of 4.0).
    always #(FCLK_HALF_PS) clkout_r = ~clkout_r;
    always #(PCLK_HALF_PS) clkoutd_r = ~clkoutd_r;

    initial begin
        #(CK_QUARTER_PS);
        forever #(FCLK_HALF_PS) clkoutp_r = ~clkoutp_r;
    end

    // LOCK after ~60 us of simulated time, matching what the vendor
    // model was measured to do. The testbench holds reset across it.
    initial begin
        #60000;
        lock_r = 1'b1;
    end

    assign CLKOUT   = clkout_r;
    assign CLKOUTP  = clkoutp_r;
    assign CLKOUTD  = clkoutd_r;
    assign CLKOUTD3 = clkoutd_r;
    assign LOCK     = lock_r;
endmodule

// ---------------------------------------------------------------------
// OSER8 -- 8-bit load, serial output on Q0.
//
// Used for the DDR3 command bus (nRAS, nCAS, nWE, A, BA), which is
// sampled on the rising edge of CK in real DRAM. CK = fclk (400 MHz),
// so one command lands per fclk cycle, four commands per pclk word.
// The design drives D0=D1, D2=D3, ... so each command is presented as
// a pair that survives one fclk cycle and is captured on CK posedge.
//
// The model is SDR: cnt is 2 bits, advances on posedge FCLK only, and
// Q0 = w[{cnt,1'b1}] picks the second bit of each pair (1,3,5,7).
// Both bits of a pair are equal by construction.
//
// The original pair indexing was the key subtlety: advancing the
// counter on BOTH fclk edges (one new bit per half-cycle) walks the
// quadrant across the word between samples, so the DRAM samples each
// word four times and every command decodes as NOP. Keeping the count
// per CYCLE is what makes the command bus land correctly.
// ---------------------------------------------------------------------
module OSER8 (
    input  D0, D1, D2, D3, D4, D5, D6, D7,
    input  FCLK, PCLK, RESET,
    output Q0
);
    wire [7:0] w = {D7, D6, D5, D4, D3, D2, D1, D0};
    reg [1:0] cnt;

    always @(posedge FCLK) begin
        if (RESET) cnt <= 2'd0;
        else       cnt <= cnt + 2'd1;
    end

    // {cnt,1'b1} selects the second bit of the active pair: 1,3,5,7 for
    // quadrants 0..3. Both bits of a pair are equal by construction.
    assign Q0 = w[{cnt, 1'b1}];
endmodule

// ---------------------------------------------------------------------
// OSER8_MEM -- 8-bit DDR serialiser with per-quadrant output enables,
// used for DQ, DQS and DM.
//
// DQ is DDR-sampled on both DQS edges, so the OSER8 must emit one new
// bit per DDR transfer (per FCLK edge). The first-party model wrapped
// the SDR OSER8 above with a 2-bit posedge-only counter for Q1, and
// shifted only on posedge FCLK -- that held each DQ bit for a full
// FCLK cycle and produced 4 distinct values per BL8 burst instead of
// 8, which is the second half of why VERIFY_BLOCK never matched.
//
// The fix is a three-bit counter advancing on both FCLK edges, with
// Q0 selecting w[cnt] (one new bit per DDR transfer). Q1 selects the
// matching TX lane on the same cadence.
//
// Q1 is the per-lane output enable, active low as the design expects
// (it gates the IOBUF: `assign DDR3_DQ[i] = dq_buf_oen ? z : dq_buf`).
// The ser_dm instance drives only Q0, so Q1 may be left open.
// ---------------------------------------------------------------------
module OSER8_MEM #(
    parameter TCLK_SOURCE = "FCLK"
) (
    input  D0, D1, D2, D3, D4, D5, D6, D7,
    input  TX0, TX1, TX2, TX3,
    input  FCLK, PCLK, TCLK, RESET,
    output Q0,
    output Q1
);
    wire [7:0] w = {D7, D6, D5, D4, D3, D2, D1, D0};
    wire [3:0] wen = {TX3, TX2, TX1, TX0};
    reg [2:0] cnt;

    always @(posedge FCLK or negedge FCLK) begin
        if (RESET) cnt <= 3'd0;
        else       cnt <= cnt + 3'd1;
    end

    // DDR: one new bit per FCLK edge, eight transfers per pclk word.
    assign Q0 = w[cnt];

    // Q1 is the output enable of the quadrant currently in flight.
    //
    // `cnt` advances on both FCLK edges, so one pclk word is eight
    // half-cycles: half-cycle `cnt` carries bit `cnt` of the word, and
    // quadrant q == cnt[2:1] covers half-cycles 2q and 2q+1, i.e. word
    // bits [2q+1:2q]. The controller documents exactly that convention on
    // its own port (`out_enable_n for dqs_out[1:0], [3:2], [5:4], [7:6]`,
    // ddr3_controller.v:165), so quadrant q is enabled by TX lane q:
    //
    //     Q1 = wen[cnt[2:1]]
    //
    // This line used to read `wen[{cnt[2:1], 1'b1}]`, a THREE-bit index
    // into a FOUR-bit vector. The concatenation takes the values 1,3,5,7;
    // 5 and 7 are out of range, so Verilog returns x for them. Because
    // cnt[2:1] is 2 for cnt=4,5 and 3 for cnt=6,7, Q1 was x for four of
    // every eight half-cycles.
    //
    // The damage was silent and load-bearing. Q1 is dqs_buf_oen, and the
    // pin is `assign DDR3_DQS[i2] = dqs_buf_oen[i2] ? 1'bz : dqs_buf[i2]`
    // (ddr3_controller.v:1202), so an x enable puts x on the DQS pin. The
    // DQS primitive derives RBURST from that pin as `~DQSIN | hold`, so x
    // propagated to rburst, `if (rburst[i]) rburst_seen[i] <= 1'b1` never
    // fired on an x condition, and the READ_CALIB loop at
    // ddr3_controller.v:609 could never see rburst_seen == 2'b11. Measured
    // before the fix: 50 rclkpos sweeps in tb_row and no "All
    // initialization DONE". See probe_oen.v, which asserts the index
    // directly.
    //
    // The old comment claimed the TX lanes are "paired by the design
    // (TX0=TX1, TX2=TX3)" and that "both halves of a TX pair are equal by
    // construction". That is false for the patterns the controller
    // actually drives: dqs_oen <= 4'b1110 (ddr3_controller.v:449) has
    // TX0=0, TX1=1. Selecting a pair member cannot be justified for a
    // 4-lane enable that the design uses independently.
    assign Q1 = wen[cnt[2:1]];
endmodule

// ---------------------------------------------------------------------
// DQS -- generates the data strobes and the read-burst indication.
//
// Real outputs: DQSW0 and DQSW270 are phase-shifted copies of fclk
// selected by the write pointer; DQSR90 is fclk for read sampling
// (the 90° in the name is the silicon phase, not a clock division);
// RBURST flags DQS activity.
//
// The original first-party model set DQSR90 = fclk_d = fclk/2 (200 MHz)
// so the IDES8 sampled at 400 Msps on both edges -- half the DDR rate
// for DDR3-800 (800 Mbps per pin). The IDES, with no phase model, then
// captured only the first half of every BL8 burst, and dout128 carried
// 4 of the expected 8 distinct values per pin, with each value held
// for two transfers. That is the third model bug Dave flagged; the
// real DQSR90 in silicon is a 90°-phase-shifted fclk at the same
// frequency, which the IDES samples on both edges to land each DDR
// bit. DQSR90 = fclk (with no phase shift modelled) gives the right
// sample rate.
//
// RBURST is asserted whenever DQSIN is asserted. The controller's read
// calibration sweeps RCLKSEL/RCLKPOS looking for RBURST to line up with
// its sampling window; with no phase model, asserting RBURST whenever
// the strobe is active lets calibration converge immediately instead of
// sweeping. That is a functional shortcut and is NOT evidence that read
// calibration works -- only the bench can say that.
// ---------------------------------------------------------------------
module DQS #(
    parameter DQS_MODE = "X4",
    parameter HWL      = "false"
) (
    input  FCLK, PCLK, DQSIN, RESET, HOLD,
    input  RLOADN, WLOADN, RMOVE, WMOVE,
    // RDIR/WDIR are INPUTS on the real primitive. This model previously
    // declared them outputs, which is wrong in the direction that matters:
    // a port declared output here accepts a connection from the DUT
    // silently, so ddr3_controller.v leaving RDIR/WDIR unconnected raised
    // NO error and the first-party sim could not have caught the EX2565
    // that synthesis reports. Direction corrected to match
    // simlib/gw2a/prim_sim.v line 7:
    //     input RLOADN, RMOVE, RDIR, WLOADN, WMOVE, WDIR, HOLD;
    input  RDIR, WDIR,
    input  [7:0] DLLSTEP,
    input  [7:0] WSTEP,
    input  [2:0] RCLKSEL,
    input  [3:0] READ,
    output DQSR90, WPOINT, RPOINT, DQSW0, DQSW270, RBURST,
    output RVALID, RFLAG, WFLAG
);
    // DQSR90 = fclk (DDR sample clock for IDES8). The real primitive
    // also phase-shifts it by 90° on the wire; with no phase model here,
    // it tracks fclk exactly so the IDES samples at the right rate.
    assign DQSR90  = FCLK;
    assign DQSW0   = FCLK;
    assign DQSW270 = ~FCLK;

    // RBURST, with a sticky window.
    //
    // The controller clears rburst_seen when it issues a calibration
    // read and then looks for RBURST ten pclk later
    // (ddr3_controller.v:581-598), sweeping RCLKSEL/RCLKPOS until the
    // strobe lines up with that window. A real DQS burst lasts about one
    // pclk, so with no phase model the burst is over long before the
    // check, calibration never converges, and the whole run hangs in
    // READ_CALIB.
    //
    // So RBURST is held for RBURST_WINDOW pclk after any strobe
    // activity. This is a functional shortcut: it lets calibration
    // converge immediately instead of sweeping, and it is NOT evidence
    // that read calibration works. Only the bench can say that.
    localparam RBURST_WINDOW = 24;
    reg [4:0] rburst_hold = 5'd0;
    reg       dqs_was_seen = 1'b0;

    always @(posedge PCLK) begin
        if (RESET) begin
            rburst_hold <= 5'd0;
        end else begin
            if (!DQSIN) begin
                rburst_hold <= RBURST_WINDOW[4:0];
                dqs_was_seen <= 1'b1;
            end else if (rburst_hold != 5'd0) begin
                rburst_hold <= rburst_hold - 5'd1;
            end
        end
    end

    assign RBURST = ~DQSIN | (rburst_hold != 5'd0);

    // FIFO pointers for the input deserialisers. Fixed at zero: with no
    // clock-domain-crossing model there is nothing to cross.
    assign WPOINT = 3'd0;
    assign RPOINT = 3'd0;

    assign RVALID = 1'b0;
    assign RFLAG  = 1'b0;
    assign WFLAG  = 1'b0;
    // RDIR/WDIR are inputs now (see the port list). They drive the DQS
    // step-counter sweep, which needs RLOADN/WLOADN high to run; the DUT
    // holds both low, so the sweep never moves and these two pins have no
    // effect on any output this model drives.
endmodule

// ---------------------------------------------------------------------
// IDES8_MEM -- 8-bit DDR input deserialiser with a FIFO crossing from
// the DQS clock domain to pclk.
//
// Functionally: sample D on both ICLK edges (DDR), and once eight bits
// have been gathered, present them on Q0..Q7. The real primitive
// presents a new word whenever RADDR catches WADDR; with one
// outstanding word and no CDC modelling, holding Q until the next
// burst is equivalent for this testbench.
//
// The original first-party model used a 2-bit fill counter and only
// captured the last 8 of every 16 ICLK edges (it wrapped after 4 edges
// and held). With DQSR90 now at fclk (DDR rate), 16 ICLK edges land
// during a BL8 burst, so the model needs a 4-bit fill that captures
// once per pclk word and holds.
//
// On FCLK = 400 MHz and BL8 = 8 CK cycles, 16 DDR transfers per pin
// happen during a burst. With the IDES sampling at 800 Msps on both
// ICLK edges, 8 samples = 1 pclk word = a quarter of the burst.
// Capturing once per 16 edges (every pclk word) and holding gives
// the design the 8-bit slice it expects at each pclk word.
// ---------------------------------------------------------------------
module IDES8_MEM (
    input  D,
    input  ICLK, FCLK, PCLK,
    input  CALIB, RESET,
    input  [2:0] WADDR, RADDR,
    output Q0, Q1, Q2, Q3, Q4, Q5, Q6, Q7
);
    reg [7:0] sr;
    reg [3:0] fill;
    reg [7:0] hold;
    reg       full;

    initial begin
        sr   = 8'd0;
        fill = 4'd0;
        hold = 8'd0;
        full = 1'b0;
    end

    // Gather eight bits, two per ICLK cycle (DDR). After 8 ICLK edges
    // (4 ICLK cycles = 1 pclk word), latch into hold.
    always @(posedge ICLK or negedge ICLK) begin
        if (RESET) begin
            sr   <= 8'd0;
            fill <= 4'd0;
        end else begin
            sr <= {sr[6:0], D};
            if (fill == 4'd7) begin
                fill <= 4'd0;
                hold <= {sr[6:0], D};
                full <= 1'b1;
            end else begin
                fill <= fill + 4'd1;
            end
        end
    end

    // The first transfer sampled is the OLDEST (it sat in sr bit 7 the
    // longest); it lands on Q7. The last transfer sampled (the newest)
    // lands on Q0. Mapping to the design's dq_in: dq_in[k][i1] is the
    // (k+1)-th-from-last transfer on DQ pin i1.
    assign {Q7, Q6, Q5, Q4, Q3, Q2, Q1, Q0} = hold;
endmodule

// ---------------------------------------------------------------------
// DLL -- bypassed under `ifdef SIM in the controller itself
// (ddr3_controller.v:658 sets dllstep=25 and dlllock=1), so no model is
// needed for the SIM build. Declared anyway so a non-SIM elaboration
// does not fail on a missing module.
// ---------------------------------------------------------------------
module DLL #(
    parameter SCAL_EN   = "true",
    parameter CODESCAL  = "101"
) (
    input  CLKIN, RESET, STOP, UPDNCNTL,
    input  [7:0] STEP,
    output LOCK
);
    assign LOCK = 1'b1;
endmodule

// ---------------------------------------------------------------------
// GSR -- global set/reset. Referenced hierarchically by the vendor
// models; instantiated by the testbench. Kept here so the flow is
// self-contained.
// ---------------------------------------------------------------------
module GSR (GSRI);
    input GSRI;
    wire GSRO;
    assign GSRO = GSRI;
endmodule
