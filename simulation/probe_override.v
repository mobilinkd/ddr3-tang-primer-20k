// ===========================================================================
// probe_override.v -- what does the DQS OVERRIDE cost, measured not guessed?
//
// THE QUESTION
// ------------
// Both benches contain:
//     assign DDR3_DQS = mem_dqs_o ? 2'b11 : 2'b00;
// (tb_rb.v:117, tb_top.v:115). DDR3_DQS is an `inout` on the controller
// (ddr3_controller.v:59) which the controller itself drives through
//     assign DDR3_DQS[i2] = dqs_buf_oen[i2] ? 1'bz : dqs_buf[i2];
// (ddr3_controller.v:1202). A second unconditional driver on the same net is
// a MULTIPLE-DRIVER conflict, not a clean override: while the controller
// drives, the two resolve to X. So what the benches actually see at the DQS
// primitive's DQSIN is the model's signal whenever the controller is driving
// too, and the controller's own DQS drive -- the thing the oen fix
// (bbfdfac) was about -- is never cleanly observable in either bench.
//
// The candidate replacement is a real bidirectional join, one line:
//     assign DDR3_DQS = mem_dqs_o ? 2'bzz : 2'b00;
// The model already encodes "released" as dqs_o = 1 (ddr3_x16_model.v:132,
// `dqs_o = reading ? 1'b0 : 1'b1`), so Z-when-released is the model's own
// intent, and the controller keeps its drive through its own oen.
//
// WHAT THIS PROBE ANSWERS
// -----------------------
// Whether removing the override is CHEAP -- i.e. whether the read path still
// returns data with the controller's DQS drive restored to the net. This runs
// the identical stimulus as probe_discard.v and differs ONLY in that one
// line, so the two runs are directly comparable:
//
//   probe_discard.v   f_push = 8192   (responses captured, override in place)
//   probe_override.v   f_push = ?      (responses captured, override removed)
//
// If f_push stays at 8192, the override was a fidelity gap and removing it
// costs one line -- and the read capture becomes real. If f_push collapses,
// the override is load-bearing in this flow and the gap stays documented
// rather than papered over.
// ===========================================================================
`timescale 1ps/1ps

module probe_override;

localparam ADDR_W = 13 + 10 + 2;
localparam N_CMDS = 8192;
localparam START  = 26'd0;

reg        pclk = 0, fclk = 0, ck = 0;
reg        resetn = 0;
always #5000 pclk = ~pclk;
always #1250 fclk = ~fclk;
always #1250 ck   = ~ck;

reg              fast_mode = 0;
reg              cmd_valid = 0;
wire             cmd_ready;
reg  [ADDR_W-1:0] cmd_addr = START;
wire [127:0]     rdata;
wire             rvalid;
wire             rpop      = rvalid;
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
localparam REFI_PCLK = 780;
always @(posedge pclk) begin
    pclk_count_local <= pclk_count_local + 1;
    refresh_pulse <= (pclk_count_local % REFI_PCLK == 0);
end

wire [15:0] DDR3_DQ;
wire [1:0]  DDR3_DQS;
wire [13:0] DDR3_A;
wire [2:0]  DDR3_BA;
wire        DDR3_nCS, DDR3_nWE, DDR3_nRAS, DDR3_nCAS;
wire        DDR3_CK, DDR3_nRESET, DDR3_CKE, DDR3_ODT;
wire [1:0]  DDR3_DM;

wire [15:0] mem_dq_o;
wire        mem_dq_oen;
wire        mem_dqs_o;

ddr3_x16_model #(.COL_WIDTH(10), .ROW_WIDTH(13), .BANK_WIDTH(3)) u_mem (
    .ck     (DDR3_CK), .ncs (DDR3_nCS), .nras(DDR3_nRAS), .ncas(DDR3_nCAS),
    .nwe    (DDR3_nWE), .a  (DDR3_A),    .ba (DDR3_BA),    .dm (DDR3_DM),
    .dq_i   (DDR3_DQ),  .dq_o(mem_dq_o), .dq_oen(mem_dq_oen),
    .dqs_o  (mem_dqs_o), .cke(DDR3_CKE), .nreset(DDR3_nRESET)
);

assign DDR3_DQ  = mem_dq_oen ? 16'hzzzz : mem_dq_o;

// THE ONE LINE THAT DIFFERS FROM probe_discard.v AND FROM BOTH BENCHES.
// The model releases DQS by driving dqs_o = 1 (ddr3_x16_model.v:132), so a
// released model is Z here and the controller's own drive -- gated by its
// own dqs_buf_oen at ddr3_controller.v:1202 -- reaches the DQS primitive.
assign DDR3_DQS = mem_dqs_o ? 2'bzz : 2'b00;

ddr3_controller #(.ROW_WIDTH(13), .COL_WIDTH(10)) u_ddr3 (
    .pclk(pclk), .fclk(fclk), .ck(ck), .resetn(resetn),
    .rd(1'b0), .wr(1'b0), .refresh(refresh_pulse), .addr(26'd0),
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

integer n_qpop = 0, n_fpush = 0, n_rvalid = 0, n_dqsread = 0, n_rburst = 0;
integer n_dqs_x = 0, n_dqs_z = 0, n_dqs_one = 0;
always @(posedge pclk) begin
    if (u_ddr3.q_pop)               n_qpop    = n_qpop + 1;
    if (u_ddr3.f_push)              n_fpush   = n_fpush + 1;
    if (rvalid)                     n_rvalid  = n_rvalid + 1;
    if (u_ddr3.dqs_read != 4'b0000) n_dqsread = n_dqsread + 1;
    if (u_ddr3.rburst[0] || u_ddr3.rburst[1]) n_rburst = n_rburst + 1;
    // What the DQS primitive actually sees on its DQSIN pin.
    if (DDR3_DQS[0] === 1'bx) n_dqs_x   = n_dqs_x + 1;
    if (DDR3_DQS[0] === 1'bz) n_dqs_z   = n_dqs_z + 1;
    if (DDR3_DQS[0] === 1'b1) n_dqs_one = n_dqs_one + 1;
end

integer issued = 0;
initial begin
    $display("PROBE-OVERRIDE start: DDR3_DQS driven by the MODEL only when released");
    resetn = 0;
    repeat (40) @(posedge pclk);
    resetn = 1;
    cmd_valid = 0;
    repeat (4000) @(posedge pclk);
    repeat (200)  @(posedge pclk);
    fast_mode = 1'b1;
    repeat (100)  @(posedge pclk);
    $display("PROBE-OVERRIDE fast_mode asserted rclkpos=%0d rd_calib_done=%b", rclkpos, read_calib_done);

    cmd_valid = 1'b1;
    while (issued < N_CMDS) begin
        @(posedge pclk);
        if (cmd_ready) begin
            cmd_addr <= cmd_addr + 26'd8;
            issued   = issued + 1;
        end
    end
    cmd_valid = 1'b0;
    $display("PROBE-OVERRIDE all %0d commands taken in at t=%0t", issued, $time);
    repeat (4000) @(posedge pclk);

    $display("PROBE-OVERRIDE RESULT");
    $display("PROBE-OVERRIDE   commands taken in = %0d", issued);
    $display("PROBE-OVERRIDE   q_pop   (issued)   = %0d", n_qpop);
    $display("PROBE-OVERRIDE   f_push  (captured) = %0d", n_fpush);
    $display("PROBE-OVERRIDE   rvalid            = %0d", n_rvalid);
    $display("PROBE-OVERRIDE   dqs_read pulses   = %0d", n_dqsread);
    $display("PROBE-OVERRIDE   rburst pulses     = %0d", n_rburst);
    $display("PROBE-OVERRIDE   DDR3_DQS[0] cycles: x=%0d z=%0d one=%0d", n_dqs_x, n_dqs_z, n_dqs_one);
    if (n_fpush >= N_CMDS)
        $display("PROBE-OVERRIDE VERDICT removing the override is CHEAP: all %0d responses still captured, so the read path does not depend on the model owning DQS", N_CMDS);
    else if (n_fpush > 100)
        $display("PROBE-OVERRIDE VERDICT PARTIAL: %0d of %0d captured -- the override IS load-bearing for part of the flow", n_fpush, N_CMDS);
    else
        $display("PROBE-OVERRIDE VERDICT the override is LOAD-BEARING: only %0d of %0d captured without it", n_fpush, N_CMDS);
    $finish;
end

initial begin
    #400000000000;
    $display("PROBE-OVERRIDE TIMEOUT q_pop=%0d f_push=%0d rvalid=%0d", n_qpop, n_fpush, n_rvalid);
    $finish;
end

endmodule
