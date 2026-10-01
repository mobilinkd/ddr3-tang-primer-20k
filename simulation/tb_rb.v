// ===========================================================================
// tb_rb.v -- focused cadence bench for the pipelined READ engine.
//
// WHY THIS EXISTS: the committed top-level flow (tb_top.v, the `top` and
// `run.top` Makefile targets) is the measurement of record, but it runs the
// WIPE, WRITE_BLOCK and legacy VERIFY_BLOCK phases first, which is a fixed
// ~12 ms of simulated time, and Icarus simulates this design at only a few
// thousand pclk per second of wall time. That makes every engine iteration
// a multi-minute turnaround.
//
// This bench drives the SAME ddr3_controller in the SAME fast_mode against
// the SAME behavioural DDR3 model, and measures ONLY the thing the 400 MB/s
// claim rests on: pclk between accepted read commands, and how many read
// commands come back. It does not compute a rate. tools/decode_uart.py
// does that, from the same on-chip counters, so a rate is never produced
// anywhere but the committed decoder.
//
// It does not verify data. Data correctness is the job of the top-level
// READ_BURST phase, which checks all 128 bits of every burst. This bench
// exists to answer "what is the cadence", which is answerable without it.
// ===========================================================================
`timescale 1ps/1ps

// Per-cycle engine tracing is off by default: $display dominates Icarus wall
// time, and a measurement run must not be slowed by its own diagnostics.
`ifdef TB_RB_TRACE
  `define RB_TRACE_ON 1
`else
  `define RB_TRACE_ON 0
`endif

module tb_rb;

localparam ADDR_W = 13 + 10 + 2;      // bank+row+col, as the controller wants
localparam N_CMDS = 8192;
localparam START  = 0;

reg        pclk = 0, fclk = 0, ck = 0;
reg        resetn = 0;
always #5000   pclk = ~pclk;         // 100 MHz
always #1250   fclk = ~fclk;         // 400 MHz
always #1250   ck   = ~ck;           // 90-degree shifted, as on the board

// The queued read port, tied the way the top level ties it in READ_BURST.
// fast_mode must be 0 through initialisation. The controller's dqs_read
// driver branches on it: in fast_mode it pulses from the engine's own tap,
// and in legacy mode from the FSM's read/calib cycle counter. Holding
// fast_mode high from time zero starves the DUT's own read calibration of
// READ pulses, rburst never arrives, and the calibration loops forever.
reg              fast_mode = 0;
reg              cmd_valid = 0;
wire             cmd_ready;
reg  [ADDR_W-1:0] cmd_addr = START;
wire [127:0]     rdata;
wire             rvalid;
wire             rpop      = rvalid;      // rready tied high
wire             accept;
wire [127:0]     dout128;
wire [15:0]      dout;
wire             data_ready, busy;
wire             write_level_done, read_calib_done;
wire [1:0]       rclkpos;    // set by the DUT's own read calibration
wire [2:0]       rclksel;
wire [7:0]       wstep;
wire [63:0]      debug;
wire [31:0]      ctl_cmds, ctl_pclk;   // on-chip counters (the measurement)

// Refresh generation. One REFI per 7.8 us (780 pclk at 100 MHz), the same
// period ddr3_top uses. The instance previously tied .refresh(1'b0), so the
// engine took no refreshes at all and the reported rate omitted refresh
// overhead -- a rate with no refreshes in it is not a rate the part can
// sustain. A 512-command run at 3 pclk/cmd spans ~15 us, so it must see
// about two refreshes to be representative.
reg [31:0] pclk_count_local = 0;
reg        refresh_pulse = 0;
localparam REFI_PCLK = 780;          // 7.8 us at 100 MHz
always @(posedge pclk) begin
    pclk_count_local <= pclk_count_local + 1;
    refresh_pulse <= (pclk_count_local % REFI_PCLK == 0);
end
wire [23:0]      ctl_rf;

// ---- DDR3 pin nets ----
// These must be real wires joined to the memory model. Leaving DDR3_DQ and
// DDR3_DQS unconnected makes the read calibration loop forever, because
// rburst comes back through the DQS pin and there is nothing to drive it.
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

assign DDR3_DQ  = mem_dq_oen ? 16'hzzzz : mem_dq_o;
assign DDR3_DQS = mem_dqs_o ? 2'bzz  : 2'b00;

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

// Host-side cross-check counters. These are NOT the measurement: the
// measurement is the on-chip ctl_pclk/ctl_cmds pair, read out below. These
// exist so the two can be compared, which is what would catch a counter
// that counts something other than what it claims.
reg [31:0] host_issued = 0, host_recvd = 0, host_pclk = 0;

reg [31:0] p0 = 0, c0 = 0, f0 = 0;
reg [31:0] p1 = 0, c1 = 0, f1 = 0;
reg        running = 0;

always @(posedge pclk) begin
    if (rpop) host_recvd <= host_recvd + 1;
    if (cmd_valid && cmd_ready) host_issued <= host_issued + 1;
    if (running) host_pclk <= host_pclk + 1;
end

initial begin
    resetn = 0;
    repeat (40) @(posedge pclk);
    resetn = 1;
    // Let the controller finish reset/CKE/config/write-level/read-calib.
    // fast_mode is tied high, so the engine itself is idle (nothing is
    // offered) while the legacy init sequence runs. The engine's dqs_read
    // tap is indexed by rclkpos, so the measurement must wait for the DUT's
    // own read calibration rather than assume a value: a wrong rclkpos shows
    // up as no read data at all, not as a quietly wrong rate.
    cmd_valid = 0;
    // A fixed wait, NOT a wait on read_calib_done. Two reasons:
    //  - gowin_prim_models.v drives DQSR90 = FCLK unconditionally and
    //    ignores READ and HOLD, so rclkpos has no effect on the simulated
    //    data path in this flow. The engine's dqs_read tap is indexed by
    //    rclkpos, but since READ is ignored, the tap value cannot change
    //    what this bench measures. Bench hardware is where that has to be
    //    settled.
    //  - calibration does not converge in this bench's clocking, and
    //    waiting for it means waiting forever.
    // The wait is still long enough for reset/CKE/config/write-leveling,
    //    which the engine's entry depends on.
    repeat (4000) @(posedge pclk);
    repeat (200) @(posedge pclk);
    fast_mode = 1'b1;
    repeat (100) @(posedge pclk);

    // Snapshot the on-chip counters, then run the measurement window.
    p0 = ctl_pclk; c0 = ctl_cmds; f0 = ctl_rf;
    host_pclk = 0; host_issued = 0; host_recvd = 0;
    running   = 1;
    $display("RB-BENCH start: p0=%0d c0=%0d rclkpos=%0d", p0, c0, rclkpos);
    // Emit the baseline in the SAME wire format the committed decoder reads
    // from the UART, so the rate for this bench is computed by
    // tools/decode_uart.py and not by a human. The decoder takes phase deltas
    // of four on-chip counters; this bench only needs slots 0 and 4.
    $display("UART|MEAS0 %08x %08x %08x %08x", p0, 32'd0, c0, f0);

    // Offer a command every cycle until the engine has taken N_CMDS of them.
    // This is the same discipline the top level uses: the address advances
    // only on cmd_ready, so a held request is never counted twice.
    // cmd_valid is asserted ONCE and held. It was previously raised inside
    // the loop, and the loop was bounded by a counter that incremented on
    // cmd_ready alone -- which is high whenever the engine is merely idle.
    // So the loop spun out without ever offering a command, and the engine
    // correctly reported zero. host_issued now counts cmd_valid & cmd_ready,
    // i.e. commands actually taken in, which is the number the cadence is
    // computed from.
    cmd_valid <= 1'b1;
    begin : pump
        while (host_issued < N_CMDS) begin
            @(posedge pclk);
            if (cmd_valid && cmd_ready) begin
                cmd_addr <= cmd_addr + 26'd8;   // 8 words = one BL8 burst
            end
        end
        cmd_valid <= 1'b0;
    end

    // Let the last bursts land.
    repeat (200) @(posedge pclk);
    running = 0;
    p1 = ctl_pclk; c1 = ctl_cmds; f1 = ctl_rf;

    $display("UART|MEAS1 %08x %08x %08x %08x", p0, 32'd0, c0, f0);
    $display("UART|MEAS2 %08x %08x %08x %08x", p0, 32'd0, c0, f0);
    $display("UART|MEAS3 %08x %08x %08x %08x", p0, 32'd0, c0, f0);
    $display("UART|MEAS4 %08x %08x %08x %08x", p1, 32'd0, c1, f1);
    $display("RB-BENCH on-chip: pclk=%0d cmds=%0d refreshes=%0d",
             p1 - p0, c1 - c0, f1 - f0);
    $display("RB-BENCH pclk_per_command=%.4f  (ISSUE_PCLK design point 3; 4 would be 398.25 MB/s and FAIL the 400 MB/s bar)",
             (p1 - p0) * 1.0 / (c1 - c0));
    $display("RB-BENCH host  : pclk=%0d issued=%0d recvd=%0d",
             host_pclk, host_issued, host_recvd);
    if ((c1 - c0) != host_issued)
        $display("RB-BENCH MISMATCH: on-chip cmd count != host count");
    else
        $display("RB-BENCH cmd counts agree");
    $display("RB-BENCH responses returned=%0d of %0d", host_recvd, host_issued);
    $display("RB-BENCH engine: fsm=%0d eng_ready=%b q_count=%0d f_count=%0d need_act=%b need_ref=%b row_valid=%b bank_open=%b row_open=%h bank_now=%h i_cycle=%0d s_cycle=%0d fast_mode=%b",
             u_ddr3.fsm, u_ddr3.eng_ready, u_ddr3.q_count, u_ddr3.f_count,
             u_ddr3.need_act, u_ddr3.need_ref, u_ddr3.row_valid,
             u_ddr3.bank_open, u_ddr3.row_open, u_ddr3.bank_now,
             u_ddr3.i_cycle, u_ddr3.s_cycle, fast_mode);
    $finish;
end

// Per-cycle trace of the first cycles after fast_mode rises. A summary at
// the end of the run cannot distinguish "never left S_IDLE" from "left and
// came back", and that distinction is the whole question here.
integer early = 0;
wire [24:0] dut_addr;
always @(posedge pclk) begin
    if (`RB_TRACE_ON && cmd_valid && early < 40) begin
        early = early + 1;
        $display("  E%0d fsm=%0d engready=%b q=%0d icyc=%0d s_cyc=%0d needact=%b rdy=%0b qpop=%0b rv=%0b v=%0b cmdv=%0b addr=%0d",
                 early, u_ddr3.fsm, u_ddr3.eng_ready, u_ddr3.q_count,
                 u_ddr3.i_cycle, u_ddr3.s_cycle, u_ddr3.need_act,
                 cmd_ready, u_ddr3.q_pop, u_ddr3.row_valid, cmd_valid, dut_addr);
    end
end

// Engine trace while commands are being offered.
integer tcount = 0;
always @(posedge pclk) begin
    if (running) begin
        tcount = tcount + 1;
        if (tcount % 2000 == 0)
            $display("   TR fsm=%0d q=%0d f=%0d icyc=%0d qpop=%0b rdy=%0b needact=%0b needref=%0b rdpipe=%b rowv=%0b banko=%0b s_cyc=%0d host=%0d recvd=%0d",
                     u_ddr3.fsm, u_ddr3.q_count, u_ddr3.f_count, u_ddr3.i_cycle,
                     u_ddr3.q_pop, cmd_ready, u_ddr3.need_act, u_ddr3.need_ref,
                     u_ddr3.rd_pipe, u_ddr3.row_valid, u_ddr3.bank_open,
                     u_ddr3.s_cycle, host_issued, host_recvd);
    end
end

// Backstop so a hung engine cannot spin forever.
initial begin
    #400_000_000;
    $display("RB-BENCH TIMEOUT");
    $finish;
end

endmodule
