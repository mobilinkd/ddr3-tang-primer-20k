// ===========================================================================
// probe_discard.v -- are READ_BURST responses being LOST, or never ARRIVING?
//
// WHY THIS PROBE EXISTS
// ---------------------
// The full tb_top run stalls with rb_issued=8192 and rb_recvd=11. Two very
// different bugs produce that same pair of numbers:
//
//   (a) the read data path is dead and only 11 of 8192 responses come back;
//   (b) all 8192 come back and the phase DISCARDS ~8181 of them.
//
// The sampled monitor in tb_top cannot tell them apart, because rb_recvd is
// itself the counter that would be discarding them.
//
// ddr3_top.v ties the controller's rready to a constant 1'b1
// (ddr3_top.v:216, .rvalid(rvalid), .rready(1'b1)), so the engine pops every
// response the moment it is valid: f_pop = rvalid & rready. But rb_recvd is
// incremented ONLY in the `else if (rvalid)` arm of the READ_BURST case
// (ddr3_top.v:522-536), and that arm is reached only after the preceding
// `else if (rb_issued < RB_CMDS)` arm is false -- i.e. only once ALL 8192
// commands have been taken in. Every response that arrives while commands
// are still being issued is therefore retired by the engine and never
// counted, and never data-checked either.
//
// If that is the mechanism, rb_recvd converges to the pipeline TAIL, not to
// the response count. READ_LATENCY=8 and RESP_DEPTH=8 (src/ddr3_controller.v
// :683,:692), so the tail is on the order of 8-16 -- and the run reports 11.
//
// WHAT THIS PROBE DOES
// --------------------
// It replicates ddr3_top's READ_BURST bookkeeping exactly, and counts BOTH
// ways on the same run:
//
//   n_rvalid_wire : every rvalid pulse, counted in the testbench, every
//                   pclk. This is what the top SAW.
//   n_recvd_late  : responses counted the way ddr3_top counts them -- only
//                   once issuing is finished. This is rb_recvd.
//
// If n_rvalid_wire >> n_recvd_late, the responses are arriving and the top
// is throwing them away: a top-level accounting bug, not a controller,
// model, or DQS failure. The same run also reports the DQS read tap, which
// is Nic's hypothesis 1 -- dqs_read is sampled every pclk here rather than
// at the monitor's one arbitrary instant, so "never pulses" and "pulses but
// is missed by the sampler" are distinguishable.
//
// It also counts dout128 non-z cycles and rburst pulses, so a dead DQ path
// is visible as a distinct signature from a discarded-but-real one.
// ===========================================================================
`timescale 1ps/1ps

module probe_discard;

localparam ADDR_W   = 13 + 10 + 2;    // bank+row+col, as the controller wants
localparam N_CMDS   = 8192;           // same count as READ_BURST
localparam START    = 26'd0;

reg        pclk = 0, fclk = 0, ck = 0;
reg        resetn = 0;
always #5000 pclk = ~pclk;           // 100 MHz pclk
always #1250 fclk = ~fclk;           // 400 MHz fclk
always #1250 ck   = ~ck;             // 90-degree shifted, as on the board

// The queued read port, tied exactly as tb_rb ties it: fast_mode is low
// through init so the DUT's own read calibration gets its legacy READ
// pulses, and rready is tied high exactly as ddr3_top ties it.
reg              fast_mode = 0;
reg              cmd_valid = 0;
wire             cmd_ready;
reg  [ADDR_W-1:0] cmd_addr = START;
wire [127:0]     rdata;
wire             rvalid;
wire             rpop      = rvalid;      // rready tied high, as in ddr3_top
wire             accept;
wire [127:0]     dout128;
wire [15:0]      dout;
wire             data_ready, busy;
wire             write_level_done, read_calib_done;
wire [1:0]       rclkpos;
wire [2:0]       rclksel;
wire [7:0]       wstep;
wire [63:0]      debug;
wire [31:0]      ctl_cmds, ctl_pclk;
wire [23:0]      ctl_rf;

reg [31:0] pclk_count_local = 0;
reg        refresh_pulse = 0;
localparam REFI_PCLK = 780;          // 7.8 us at 100 MHz
always @(posedge pclk) begin
    pclk_count_local <= pclk_count_local + 1;
    refresh_pulse <= (pclk_count_local % REFI_PCLK == 0);
end

// ---- DDR3 pin nets (same as tb_rb) ----
wire [15:0] DDR3_DQ;
wire [1:0]  DDR3_DQS;
wire [13:0] DDR3_A;
wire [2:0]  DDR3_BA;
wire        DDR3_nCS, DDR3_nWE, DDR3_nRAS, DDR3_nCAS;
wire        DDR3_CK, DDR3_nRESET, DDR3_CKE, DDR3_ODT;
wire [1:0]  DDR3_DM;

wire [15:0] mem_dq_o;
wire        mem_dq_oen;      // active low
wire        mem_dqs_o;

ddr3_x16_model #(.COL_WIDTH(10), .ROW_WIDTH(13), .BANK_WIDTH(3)) u_mem (
    .ck     (DDR3_CK),
    .ncs    (DDR3_nCS),
    .nras   (DDR3_nRAS),
    .ncas   (DDR3_nCAS),
    .nwe    (DDR3_nWE),
    .a      (DDR3_A),
    .ba     (DDR3_BA),
    .dm     (DDR3_DM),
    .dq_i   (DDR3_DQ),
    .dq_o   (mem_dq_o),
    .dq_oen (mem_dq_oen),
    .dqs_o  (mem_dqs_o),
    .cke    (DDR3_CKE),
    .nreset (DDR3_nRESET)
);

// THE OVERRIDE UNDER DISCUSSION. Both benches contain these two lines, and
// they discard whatever the controller drives on the DQ/DQS pins:
//   assign DDR3_DQS = mem_dqs_o ? 2'b11 : 2'b00;
// so DQSIN at the DQS primitive is the MODEL's dqs_o, not the controller's
// OSER8_MEM output. Kept identical to tb_rb/tb_top here so this probe
// measures the same thing they do.
assign DDR3_DQ  = mem_dq_oen ? 16'hzzzz : mem_dq_o;
assign DDR3_DQS = mem_dqs_o ? 2'b11 : 2'b00;

ddr3_controller #(.ROW_WIDTH(13), .COL_WIDTH(10)) u_ddr3 (
    .pclk(pclk), .fclk(fclk), .ck(ck), .resetn(resetn),
    .rd(1'b0), .wr(1'b0), .refresh(refresh_pulse),
    .addr(26'd0),
    .din(16'd0), .dout128(dout128), .dout(dout),
    .data_ready(data_ready), .busy(busy), .accept(accept),
    .write_level_done(write_level_done), .wstep(wstep),
    .read_calib_done(read_calib_done), .rclkpos(rclkpos), .rclksel(rclksel),
    .debug(debug),
    .fast_mode(fast_mode), .cmd_valid(cmd_valid), .cmd_addr(cmd_addr),
    .cmd_ready(cmd_ready), .rdata(rdata), .rvalid(rvalid), .rready(rpop),
    .cmd_count_rd(ctl_cmds), .pclk_count(ctl_pclk), .refresh_count(ctl_rf),
    .DDR3_nRESET(DDR3_nRESET), .DDR3_DQ(DDR3_DQ), .DDR3_DQS(DDR3_DQS),
    .DDR3_A(DDR3_A), .DDR3_BA(DDR3_BA), .DDR3_nRAS(DDR3_nRAS),
    .DDR3_nCAS(DDR3_nCAS), .DDR3_nWE(DDR3_nWE), .DDR3_CK(DDR3_CK),
    .DDR3_CKE(DDR3_CKE), .DDR3_ODT(DDR3_ODT), .DDR3_DM(DDR3_DM)
);

// ---- wire-level counters, every pclk, independent of any bookkeeping ----
integer n_qpop = 0, n_fpush = 0, n_fpop = 0;
integer n_rvalid_wire = 0, n_dqsread = 0, n_rburst = 0;
integer n_dout_nz = 0, n_data_match = 0;
reg        issuing = 0;

always @(posedge pclk) begin
    if (u_ddr3.q_pop)                     n_qpop        = n_qpop + 1;
    if (u_ddr3.f_push)                    n_fpush       = n_fpush + 1;
    if (u_ddr3.f_pop)                     n_fpop        = n_fpop + 1;
    if (rvalid)                           n_rvalid_wire = n_rvalid_wire + 1;
    if (u_ddr3.dqs_read != 4'b0000)       n_dqsread     = n_dqsread + 1;
    if (u_ddr3.rburst[0] || u_ddr3.rburst[1]) n_rburst  = n_rburst + 1;
    if (u_ddr3.dout128 !== 128'hz)        n_dout_nz     = n_dout_nz + 1;
end

// ---- ddr3_top's counting rule, verbatim: only AFTER issuing completes ----
integer n_recvd_late = 0;
always @(posedge pclk) begin
    if (!issuing && rvalid) n_recvd_late = n_recvd_late + 1;
end

integer issued = 0;
initial begin
    $display("PROBE start");
    resetn = 0;
    repeat (40) @(posedge pclk);
    resetn = 1;

    // Same fixed wait as tb_rb: calibration does not converge in this
    // bench's clocking, so waiting on read_calib_done waits forever.
    cmd_valid = 0;
    repeat (4000) @(posedge pclk);
    repeat (200)  @(posedge pclk);
    fast_mode = 1'b1;
    repeat (100)  @(posedge pclk);
    $display("PROBE fast_mode asserted, rclkpos=%0d rd_calib_done=%b", rclkpos, read_calib_done);

    // Offer exactly as ddr3_top's READ_BURST arm does.
    cmd_valid = 1'b1;
    issuing   = 1'b1;
    while (issued < N_CMDS) begin
        @(posedge pclk);
        if (cmd_ready) begin
            cmd_addr <= cmd_addr + 26'd8;
            issued   = issued + 1;
        end
    end
    cmd_valid = 1'b0;
    $display("PROBE all %0d commands taken in at t=%0t", issued, $time);

    // Issuing is done: from here ddr3_top WOULD start counting. Drain long
    // enough for the tail to come back.
    issuing = 1'b0;
    repeat (4000) @(posedge pclk);

    $display("PROBE RESULT");
    $display("PROBE   commands offered/taken in = %0d", issued);
    $display("PROBE   q_pop    (issued to DRAM)  = %0d", n_qpop);
    $display("PROBE   f_push   (captured)        = %0d", n_fpush);
    $display("PROBE   f_pop    (retired)         = %0d", n_fpop);
    $display("PROBE   rvalid WIRE  (all)        = %0d", n_rvalid_wire);
    $display("PROBE   rvalid LATE  (ddr3_top's rule) = %0d", n_recvd_late);
    $display("PROBE   DISCARDED by the top      = %0d", n_rvalid_wire - n_recvd_late);
    $display("PROBE   dout128 non-z cycles      = %0d", n_dout_nz);
    $display("PROBE   dqs_read pulses          = %0d", n_dqsread);
    $display("PROBE   rburst pulses            = %0d", n_rburst);
    $display("PROBE   READ_LATENCY=%0d RESP_DEPTH=%0d", u_ddr3.READ_LATENCY, u_ddr3.RESP_DEPTH);

    if (n_dqsread == 0)
        $display("PROBE VERDICT dqs_read NEVER pulses -- the DQS read tap is not armed");
    else if (n_fpush > 100 && n_rvalid_wire > n_recvd_late + 100)
        $display("PROBE VERDICT responses ARRIVE and are RETIRED, but ddr3_top counts only the tail: TOP-LEVEL ACCOUNTING BUG, read path is alive");
    else if (n_fpush <= 100)
        $display("PROBE VERDICT responses are NOT being captured: read data path dead upstream of the FIFO");
    else
        $display("PROBE VERDICT inconclusive -- see the counters above");
    $finish;
end

initial begin
    #400000000000;    // 400 ms sim time
    $display("PROBE TIMEOUT q_pop=%0d f_push=%0d rvalid_wire=%0d", n_qpop, n_fpush, n_rvalid_wire);
    $finish;
end

endmodule
