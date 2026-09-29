// Focused unit TB for the pipelined read engine (fast_mode=1).
//
// Purpose: answer ONE question with direct evidence -- when a queued read
// is ISSUED (q_pop), does the response FIFO ever receive a push
// (f_push / f_count), and therefore does rvalid ever assert?
//
// This bypasses ddr3_top's long WRITE/READ phases entirely. It brings up
// the PLL and memory model exactly as tb_top does, asserts fast_mode, and
// then fills the command queue and watches the engine.
//
// It also probes the two candidate completion signals side by side:
//   - rd_cap   : the timer the current code uses (f_push = rd_cap & ~f_full)
//   - data_ready: the DUT's own legacy completion output
// so we can see, per issued read, WHICH one (if either) actually coincides
// with data on dout128.

`timescale 1ps / 1ps

module tb_engine;

    // ---- DDR3 pins (from the controller) ----
    wire        ddr_nrst, ddr_ck, cke, cs_n, ras_n, cas_n, we_n;
    wire [2:0]  ba;
    wire [12:0] a;
    wire [1:0]  dm;
    wire [15:0] dq;
    wire [1:0]  dqs;
    wire        odt;
    wire        dqs_n  = ~dqs;
    wire [1:0]  tdqs_n = 2'bzz;

    // ---- clocks ----
    reg pclk = 0, fclk = 0, ck = 0;
    reg resetn = 0;

    initial begin pclk = 0; forever #5004.4 pclk = ~pclk; end   // ~99.5625 MHz
    initial begin fclk = 0; forever #1251.1 fclk = ~fclk; end   // ~399.7 MHz
    initial begin ck   = 0; forever #1251.1 ck   = ~ck;   end

    // ---- controller-side signals ----
    reg  [25:0] addr    = 26'd0;
    reg         rd = 0, wr = 0, refresh = 0;
    reg  [15:0] din     = 16'd0;
    wire [15:0] dout;
    wire [127:0] dout128;
    wire        data_ready, busy, accept;

    wire [7:0]  wstep;
    wire [1:0]  rclkpos;
    wire [2:0]  rclksel;
    wire        wlevel_done, rcalib_done;

    // ---- queued read port ----
    reg         cmd_valid = 0;
    reg  [25:0] cmd_addr  = 26'd0;
    wire        cmd_ready;
    wire        rvalid;
    wire [127:0] rdata;
    wire [31:0] cmd_count_rd, pclk_count;
    wire [23:0] refresh_count;

    reg rready = 1;

    ddr3_controller #(.ROW_WIDTH(13), .COL_WIDTH(10)) u (
        .pclk(pclk), .fclk(fclk), .ck(ck), .resetn(resetn),
        .rd(rd), .wr(wr), .refresh(refresh), .addr(addr),
        .din(din), .dout(dout), .dout128(dout128),
        .data_ready(data_ready), .busy(busy), .accept(accept),
        .write_level_done(wlevel_done), .wstep(wstep),
        .read_calib_done(rcalib_done), .rclkpos(rclkpos), .rclksel(rclksel),
        .fast_mode(1'b1),
        .cmd_valid(cmd_valid), .cmd_addr(cmd_addr), .cmd_ready(cmd_ready),
        .rvalid(rvalid), .rready(rready), .rdata(rdata),
        .cmd_count_rd(cmd_count_rd), .pclk_count(pclk_count),
        .refresh_count(refresh_count),
        .DDR3_nRESET(ddr_nrst), .DDR3_CK(ddr_ck), .DDR3_CKE(cke),
        .DDR3_nCS(cs_n), .DDR3_nRAS(ras_n), .DDR3_nCAS(cas_n), .DDR3_nWE(we_n),
        .DDR3_A(a), .DDR3_BA(ba), .DDR3_DQ(dq), .DDR3_DQS(dqs), .DDR3_DM(dm),
        .DDR3_ODT(odt)
    );

    // The behavioural model takes the controller's DDR3_n* pins in the
    // active-low convention those pins are named for, and exposes a
    // separate dq_o/dq_oen pair for the read bus.
    wire [15:0] m_dq_o;
    wire        m_dq_oen;
    ddr3_x16_model sdram (
        .ck    (ddr_ck),
        .ncs   (cs_n),
        .nras  (ras_n),
        .ncas  (cas_n),
        .nwe   (we_n),
        .a     (a),
        .ba    (ba),
        .dm    (dm),
        .dq_i  (dq),
        .dq_o  (m_dq_o),
        .dq_oen(m_dq_oen),
        .dqs_o (dqs),
        .cke   (cke),
        .nreset(ddr_nrst)
    );

    // Join the model's read bus to the DUT's DQ net. The controller's own
    // IOBUF drives DDR3_DQ for writes and releases it (dq_buf_oen) for
    // reads, so the pin is already resolved; the model only adds its side.
    // Without this assign m_dq_o was dangling, DDR3_DQ stayed z through the
    // whole run, IDES8_MEM latched z, and dout128 was x for EVERY read --
    // an artifact of the harness, not of the read path. tb_top.v has always
    // had this line.
    assign dq = m_dq_oen ? 16'hzzzz : m_dq_o;

    // ---- counters (testbench-side observation only) ----
    integer n_qpop = 0, n_fpush = 0, n_rvalid_seen = 0, n_dataready = 0;
    integer n_dout128_chg = 0;
    reg [127:0] dout128_prev = 128'd0;

    // Track the two candidate "data arrived" indications separately, and
    // whether dout128 is actually non-zero/changing at that moment.
    always @(posedge pclk) begin
        if (!resetn) begin
            n_qpop <= 0; n_fpush <= 0; n_rvalid_seen <= 0; n_dataready <= 0;
        end else begin
            if (u.q_pop)                n_qpop        <= n_qpop + 1;
            if (u.f_push)               n_fpush       <= n_fpush + 1;
            if (rvalid)                 n_rvalid_seen <= n_rvalid_seen + 1;
            if (data_ready)             n_dataready   <= n_dataready + 1;
        end
    end

    always @(posedge pclk) begin
        if (resetn) begin
            if (dout128 !== dout128_prev) n_dout128_chg <= n_dout128_chg + 1;
            dout128_prev <= dout128;
        end
    end

    // ---- progress monitor ----
    integer cyc = 0;
    always @(posedge pclk) begin
        if (!resetn) cyc <= 0; else cyc <= cyc + 1;
        if (resetn && (cyc % 2000 == 0)) begin
            $display("MON t=%0t cyc=%0d fsm=%0d q=%0d f=%0d icyc=%0d qpop=%b fpush=%b rvalid=%b cmdrdy=%b needact=%b needref=%b rdpipe=%0d rowv=%b | qpop_n=%0d fpush_n=%0d rval_n=%0d dready_n=%0d dout_chg=%0d dout128=%032x",
                     $time, cyc, u.fsm, u.q_count, u.f_count, u.i_cycle,
                     u.q_pop, u.f_push, rvalid, cmd_ready, u.need_act,
                     u.need_ref, u.rd_pipe, u.row_valid,
                     n_qpop, n_fpush, n_rvalid_seen, n_dataready,
                     n_dout128_chg, dout128);
            $display("    XTRACE q_head=%0d q_tail=%0d qaddr0=%0d qrow=%0d qbnk=%0d row_open=%0d bank_open=%b bank_now=%0d row_valid=%b fsm=%0d s_cycle=%0d",
                     u.q_head, u.q_tail, u.q_addr[0], u.q_row, u.q_bnk,
                     u.row_open, u.bank_open, u.bank_now, u.row_valid,
                     u.fsm, u.s_cycle);
        end
    end

    // ---- the actual test: fill the queue, then watch ----
    initial begin
        $display("TB-ENGINE-START");
        #60000;                        // 60 us PLL lock (tb_top uses 300 us)
        resetn = 1;
        $display("TB-ENGINE-RESET-RELEASED t=%0t", $time);

        // Drive the queue with a run of same-row reads so no row miss
        // interleaves. Address = word address; +8 words per BL8 burst.
        // Drive AFTER the edge (the iverilog TB-scheduling trap).
        repeat (24) begin
            @(negedge pclk);
            cmd_valid = 1;
            cmd_addr  = 26'd0;
            @(negedge pclk);
        end
        @(negedge pclk);
        cmd_valid = 0;

        // Now just watch for a long window.
        repeat (40000) @(posedge pclk);

        $display("TB-ENGINE-SUMMARY");
        $display("  q_pop total   = %0d", n_qpop);
        $display("  f_push total  = %0d   <-- FIFO pushes", n_fpush);
        $display("  rvalid total  = %0d", n_rvalid_seen);
        $display("  data_ready tot= %0d   <-- legacy completion sig", n_dataready);
        $display("  dout128 chg   = %0d", n_dout128_chg);
        $display("  final: f_count=%0d q_count=%0d rd_pipe=%0d", u.f_count, u.q_count, u.rd_pipe);
        if (n_qpop > 0 && n_fpush == 0)
            $display("VERDICT: reads ARE issued but the response FIFO never receives a push");
        else if (n_qpop == 0)
            $display("VERDICT: no reads are issued at all (q_pop never fires)");
        else
            $display("VERDICT: reads issue and FIFO receives pushes; n_qpop=%0d n_fpush=%0d", n_qpop, n_fpush);
        $finish;
    end

endmodule
