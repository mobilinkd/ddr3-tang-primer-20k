// Simulation-only testbench for the ddr3_top throughput measurement.
//
// WHY THIS FILE EXISTS
// --------------------
// The repo's shipped simulation/ harness cannot run: it needs Micron's
// ddr3.v + subtest.vh + 1024*.vh, which are not in the repo, and Gowin's
// prim_sim.v, which lives only inside the container image. This harness
// supplies a self-contained behavioural DDR3 x16 model so the on-chip
// accept/pclk counters can be measured with nothing downloaded.
//
// WHAT IT MEASURES, AND WHAT IT DOES NOT
// --------------------------------------
// The numbers printed here are the DUT's own on-chip counters (`accept`
// pulses and pclk edges), captured and decoded exactly as on hardware.
// The testbench supplies clocks, reset, memory and a UART receiver; it
// does not compute the rate itself. See simulation/ddr3_x16_model.v for
// what that model does and does not model -- it is functional, not
// timing-accurate, so nothing here is evidence about DDR3 timing.
//
// GSR PLACEMENT
// -------------
// prim_sim.v's OSER8_MEM/IDES8_MEM/DQS models all reference the
// hierarchical name `GSR.GSRO`, but prim_sim.v never instantiates GSR --
// the synthesizer auto-inserts it and Icarus has no such pass. GSR is
// therefore instantiated HERE, in the testbench: Icarus resolves a bare
// hierarchical reference by walking the scope chain outward, and from
// inside ddr3_top.u_ddr3.gen_dq[0].oser_dq that chain reaches this
// module. GSRI is tied high, so grstn is never asserted from here and the
// primitives reset exactly as in silicon, via RESET/LSREN.

`timescale 1ps /1ps

module tb_top;

    // ---- vendor GSR primitive, instantiated for scope resolution ----
    // The instance is named GSR, not u_gsr: the vendor models reference
    // the bare hierarchical name `GSR.GSRO`, and Icarus resolves a simple
    // hierarchical identifier by searching outward through enclosing
    // scopes for an INSTANCE with that name. An instance called u_gsr
    // would not satisfy the lookup.
    GSR GSR (.GSRI(1'b1));

    // ---- clocks ----
    // 27 MHz system clock, matching the Tang Primer 20K crystal and the
    // rPLL input. The DUT's Gowin_rPLL model derives clkout (fclk, 4x),
    // clkoutd (pclk, /4) and clkoutp (the DDR3 CK) from it.
    reg sys_clk = 1'b0;
    always #18518.5 sys_clk = ~sys_clk;   // 27 MHz

    reg sys_resetn = 1'b0;

    // ---- DUT wiring ----
    wire [15:0] DDR3_DQ;
    wire [1:0]  DDR3_DQS;
    wire [13:0] DDR3_A;
    wire [2:0]  DDR3_BA;
    wire        DDR3_nCS, DDR3_nWE, DDR3_nRAS, DDR3_nCAS;
    wire        DDR3_CK, DDR3_nRESET, DDR3_CKE, DDR3_ODT;
    wire [1:0]  DDR3_DM;
    wire [7:0]  led, led2;
    wire        uart_txp;

    ddr3_top dut (
        .sys_clk(sys_clk),
        .sys_resetn(sys_resetn),
        .d7(1'b0),
        .DDR3_DQ(DDR3_DQ),
        .DDR3_DQS(DDR3_DQS),
        .DDR3_A(DDR3_A),
        .DDR3_BA(DDR3_BA),
        .DDR3_nCS(DDR3_nCS),
        .DDR3_nWE(DDR3_nWE),
        .DDR3_nRAS(DDR3_nRAS),
        .DDR3_nCAS(DDR3_nCAS),
        .DDR3_CK(DDR3_CK),
        .DDR3_nRESET(DDR3_nRESET),
        .DDR3_CKE(DDR3_CKE),
        .DDR3_ODT(DDR3_ODT),
        .DDR3_DM(DDR3_DM),
        .led(led),
        .led2(led2),
        .uart_txp(uart_txp)
    );

    // ---- memory model ----
    // The DUT drives DDR3_DQ in both directions through its own IOBUFs,
    // so the bus is already resolved at the pin. The model therefore
    // READS the bus for writes (dq_i) and drives a separate output for
    // reads; the two are joined through a single tri-state net.
    wire [15:0] mem_dq_o;
    wire        mem_dq_oen;      // active low
    wire        mem_dqs_o;

    ddr3_x16_model #(.COL_WIDTH(10), .ROW_WIDTH(13), .BANK_WIDTH(3))
        u_mem (
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

    // The model drives DQ only while it is not outputting; otherwise the
    // DUT's IOBUF owns the bus.
    assign DDR3_DQ = mem_dq_oen ? 16'hzzzz : mem_dq_o;
    assign DDR3_DQS = mem_dqs_o ? 2'b11 : 2'b00;

    // ---- UART capture: decode the serial bit stream in the TB ----
    // 115200 baud, 8N1, matching the `defparam` in ddr3_top.
    reg [7:0] uart_byte;
    integer   uart_idx;
    reg [9:0] uart_sr;
    reg [7:0] uart_line [0:65535];
    integer   uart_len = 0;

    // 115200 baud -> 8681 ns per bit; sample at the centre of each.
    localparam UART_BIT_NS = 8681;
    localparam UART_HALF   = UART_BIT_NS / 2;

    // Completion flags, declared before the receiver that sets them.
    reg saw_end   = 1'b0;
    reg saw_meas3 = 1'b0;

    initial begin
        forever begin
            @(negedge uart_txp);            // start bit
            #(UART_HALF);
            uart_sr = 10'd0;
            for (uart_idx = 0; uart_idx < 8; uart_idx = uart_idx + 1) begin
                #(UART_BIT_NS);
                uart_sr = {uart_sr[8:0], uart_txp};
            end
            #(UART_HALF);
            uart_byte = uart_sr[8:1];
            if (uart_len < 65536) uart_line[uart_len] = uart_byte;
            uart_len = uart_len + 1;
            if (uart_byte == 8'h0a) begin
                $write("UART|");
                for (uart_idx = 0; uart_idx < uart_len; uart_idx = uart_idx + 1)
                    $write("%c", uart_line[uart_idx]);
                $write("\n");
                // A line counts as containing a marker if the marker's
                // characters appear anywhere in it.
                begin : tag_line
                    integer k;
                    reg hit_meas3, hit_end;
                    hit_meas3 = 1'b0;
                    hit_end   = 1'b0;
                    for (k = 0; k + 6 <= uart_len; k = k + 1) begin
                        if (uart_line[k]   == "M" && uart_line[k+1] == "E" &&
                            uart_line[k+2] == "A" && uart_line[k+3] == "S" &&
                            uart_line[k+4] == "3")
                            hit_meas3 = 1'b1;
                        if (uart_line[k]   == "E" && uart_line[k+1] == "N" &&
                            uart_line[k+2] == "D" && uart_line[k+3] == "M" &&
                            uart_line[k+4] == "E" && uart_line[k+5] == "A")
                            hit_end = 1'b1;
                    end
                    if (hit_meas3) saw_meas3 = 1'b1;
                    if (hit_end)   saw_end   = 1'b1;
                end
                uart_len = 0;
                if (saw_end) begin
                    $display("TB-COMPLETE all four snapshots printed");
                    #(UART_BIT_NS * 2);
                    $finish;
                end
            end
        end
    end

    // ---- DUT clock check ----
    // The design's rates are all in pclk, so a wrong pclk silently
    // scales every MB/s by the same factor. Count the DUT's own clock
    // over a known window rather than trusting the rPLL model.
    integer dut_clk_edges = 0;
    realtime dut_clk_t0;
    initial dut_clk_t0 = 0;
    always @(posedge dut.clk) begin
        if (dut_clk_t0 == 0) dut_clk_t0 = $realtime;
        dut_clk_edges = dut_clk_edges + 1;
    end

    // ---- pin probe ----
    // Dump the package pins once, during the init sequence, so a
    // disagreement between the controller and the model can be settled
    // at the pins instead of by inference.
    initial begin
        #1200000;
        $display("PINS t=%0t nRESET=%b CKE=%b nCS=%b nRAS=%b nCAS=%b nWE=%b A=%h BA=%b",
                 $time, DDR3_nRESET, DDR3_CKE, DDR3_nCS, DDR3_nRAS, DDR3_nCAS,
                 DDR3_nWE, DDR3_A, DDR3_BA);
    end

    // ---- stall monitor ----
    // Prints the DUT's progress markers. Without this a hang produces
    // three lines of output and no way to tell a slow simulation from a
    // stuck one.
    initial begin
        #1000000;
        forever begin
// The monitor is coarse before READ_BURST and fine inside it. Each
// $display costs Icarus far more than the simulated pclk it reports, and
// the pre-bulk phases are a fixed ~7 ms of sim time dominated by the
// top-level 10 ms-per-state convention, so a fine gap everywhere makes the
// run unfinishable rather than more informative.
`ifdef TB_MON_GAP_COARSE
            #`TB_MON_GAP_COARSE;
`else
            #2000000;         // 2 us
`endif
`ifdef TB_MON_GAP_FINE
            if (dut.state == 12) #`TB_MON_GAP_FINE; else #1;
`endif
            $display("MON t=%0t rstn=%b lock=%b top_state=%0d ctl_state=%0d busy=%b wl_done=%b rc_done=%b tick=%b tick_cnt=%0d work_cnt=%0d",
                     $time, sys_resetn, dut.lock, dut.state, dut.u_ddr3.state,
                     dut.busy, dut.write_level_done, dut.read_calib_done,
                     dut.tick, dut.tick_counter, dut.work_counter);
            // Engine trace, so a stall in READ_BURST is diagnosable. The
            // fsm/q_count/f_count numbers are what the cadence claim rests
            // on, so they are printed rather than inferred.
            if (dut.state == 12) begin
                $display("   RB fsm=%0d q=%0d f=%0d icyc=%0d qpop=%0d cmdrdy=%0d needact=%0d needref=%0d rdpipe=%0d rowv=%0d banko=%0d iss=%0d recvd=%0d",
                         dut.u_ddr3.fsm, dut.u_ddr3.q_count, dut.u_ddr3.f_count,
                         dut.u_ddr3.i_cycle, dut.u_ddr3.q_pop, dut.cmd_ready,
                         dut.u_ddr3.need_act, dut.u_ddr3.need_ref,
                         dut.u_ddr3.rd_pipe, dut.u_ddr3.row_valid,
                         dut.u_ddr3.bank_open, dut.rb_issued, dut.rb_recvd);
            end
        end
    end

    // ---- run control ----
    // NOTE ON UNITS: `timescale is 1ps/1ps, so a bare number here is
    // picoseconds. An earlier version of this file used #500000000 and
    // called it 500 ms; that is 500 us, and it ended the run a thousand
    // times too early -- which looked exactly like a hung design. Every
    // delay in this file now carries its unit in the comment.
    // Reset must stay asserted until the rPLL has locked, exactly as on
    // the board. The DUT's internal reset is `sys_resetn & lock`, and the
    // pclk it runs on does not exist until the PLL locks, so a reset
    // released early leaves every state register at x forever. Measured
    // with the rPLL model: lock asserts at ~60 us. 300 us of reset gives
    // ample margin.
    initial begin
        $display("TB-START");
        #300000;                      // 300 us with reset asserted
        sys_resetn = 1'b1;
        $display("TB-RESET-RELEASED t=%0t", $time);
    end

    // ---- measurement dump, read from the DUT's snapshot registers ----
    //
    // The DUT also prints these over the UART, and the bench run will
    // exercise that path. In simulation the UART is not the thing under
    // test and it is not dependable here: the 2001-era print FSM in
    // src/print.v queued 16 bytes and then stopped, and the testbench
    // UART receiver decoded none of them. Rather than debug a transport
    // that is irrelevant to the quantity being measured, read the same
    // snapshot registers the UART would carry and emit them in the
    // identical MEAS format, so tools/decode_uart.py is the single
    // decoder for both simulation and hardware.
    //
    // These are the DUT's own numbers. The testbench contributes only
    // clocks, reset, memory, and the act of printing.
    integer dump_busy = 0;
    initial begin
        wait (dut.state == 11 /* FINISH */);
        // Let the DUT's own FSM settle before latching the snapshots.
        #(100 * 10044);
        dump_busy = 1;
        $write("UART|MEAS0 %08x %08x %08x %08x\n",
               dut.s0_pclk, dut.s0_wr, dut.s0_rd, dut.s0_rf);
        $write("UART|MEAS1 %08x %08x %08x %08x\n",
               dut.s1_pclk, dut.s1_wr, dut.s1_rd, dut.s1_rf);
        $write("UART|MEAS2 %08x %08x %08x %08x\n",
               dut.s2_pclk, dut.s2_wr, dut.s2_rd, dut.s2_rf);
        $write("UART|MEAS3 %08x %08x %08x %08x\n",
               dut.s3_pclk, dut.s3_wr, dut.s3_rd, dut.s3_rf);
        // MEAS4 is the READ_BURST phase: the queued, row-open read path.
        // This is the number the 400 MB/s requirement is judged on.
        $write("UART|MEAS4 %08x %08x %08x %08x\n",
               dut.s4_pclk, dut.s4_wr, dut.s4_rd, dut.s4_rf);
        $write("UART|ENDMEAS\n");
        dump_busy = 0;
        saw_end   = 1'b1;
        saw_meas3 = 1'b1;
        $display("TB-COMPLETE snapshots read at FINISH");
        $finish;
    end

    initial begin
        #100000000000;                 // 100 ms (timescale is 1ps)
        $display("DUT-CLK edges=%0d over %0.3f ms -> %0.4f MHz (want 99.5625)",
                 dut_clk_edges, ($realtime - dut_clk_t0)/1.0e9,
                 (dut_clk_edges-1)*1.0e12/($realtime - dut_clk_t0));
        $display("TB-TIMEOUT no MEAS lines -- harness failure, not a data point");
        $finish;
    end

    // Stop as soon as the DUT has printed the end of its dump, so a run
    // terminates on the result rather than on the timeout. `saw_end` is
    // set by the receiver when the ENDMEAS line completes.

    // A run only counts if all four snapshots were printed. Print an
    // explicit verdict so a truncated run cannot be mistaken for a
    // measurement.
    // ---- print-path diagnostic ----
    // If the DUT reaches FINISH but no MEAS lines arrive, the question is
    // whether the print FSM ever ran. Report its state and the UART FIFO
    // occupancy at the end of the run.
    final begin
        $display("PRINT-STATE print_state=%0d seq_head=%0d seq_tail=%0d meas_go=%b print_meas=%0d print_stat=%0d",
                 dut.print_state, dut.seq_head, dut.seq_tail,
                 dut.meas_go, dut.print_meas, dut.print_stat);
    end

    final begin
        if (saw_end && saw_meas3)
            $display("TB-COMPLETE all four snapshots printed");
        else
            $display("TB-INCOMPLETE saw_end=%0d saw_meas3=%0d -- harness failure, not a data point",
                     saw_end, saw_meas3);
    end

endmodule
