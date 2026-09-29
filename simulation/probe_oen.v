// Unit probe for the OSER8_MEM output-enable index.
//
// Why this exists: gowin_prim_models.v:240 was
//     assign Q1 = wen[{cnt[2:1], 1'b1}];
// `wen` is 4 bits, so the 3-bit concatenation {cnt[2:1],1'b1} takes the
// values 1,3,5,7. Indices 5 and 7 are out of range for a 4-bit vector and
// return x, so Q1 -- and therefore dqs_buf_oen, and therefore the DDR3_DQS
// pin -- is x for four of every eight FCLK half-cycles. Read calibration
// keys off RBURST, which is derived from that pin, so it never converges.
//
// This probe is deliberately tiny and drives ONE instance with a known
// per-quadrant enable pattern, so the expected Q1 sequence is known by
// construction. It asserts, it does not print and pass.
//
// TX convention under test: TX0..TX3 are the four quadrant enables, and
// quadrant q covers the two DDR transfers 2q and 2q+1 of the pclk word,
// i.e. q == cnt[2:1] for the 3-bit both-edge counter. So the correct
// enable for the quadrant in flight is wen[cnt[2:1]], with indices 0..3.
//
// The stimulus below is one-hot-per-quadrant (each quadrant a different
// value) rather than the paired pattern the controller happens to use, so
// the probe fails if the index is merely "in range" but wired to the
// wrong quadrant.

`timescale 1ps/1ps

module probe_oen;

    reg        fclk = 1'b0;
    reg        pclk = 1'b0;
    reg        rst_n = 1'b0;
    reg  [7:0] d   = 8'h00;
    reg  [3:0] tx  = 4'b0000;
    wire       q0, q1;

    // fclk = 4x pclk, DDR-style: cnt advances on BOTH edges.
    always #1256.5  fclk = ~fclk;      // 398.25 MHz
    always #5026.5  pclk = ~pclk;      //  99.5625 MHz

    OSER8_MEM dut (
        .D0(d[0]), .D1(d[1]), .D2(d[2]), .D3(d[3]),
        .D4(d[4]), .D5(d[5]), .D6(d[6]), .D7(d[7]),
        .TX0(tx[0]), .TX1(tx[1]), .TX2(tx[2]), .TX3(tx[3]),
        .FCLK(fclk), .PCLK(pclk), .TCLK(fclk), .RESET(~rst_n),
        .Q0(q0), .Q1(q1)
    );

    integer errors = 0;
    integer n_x    = 0;
    integer n_chk  = 0;

    // Each quadrant gets a distinct value, so a wrong-quadrant index is
    // caught, not just an out-of-range one.
    localparam [3:0] QPAT = 4'b1001;

    initial begin
        $display("OEN-PROBE expecting Q1 == wen[cnt[2:1]], QPAT=%b", QPAT);
        #20000 rst_n = 1'b1;           // release reset
        tx = QPAT;
        #40000;
        rst_n = 1'b0;
        $display("OEN-PROBE PASS (no x, no quadrant mismatch)");
        $finish;
    end

    // Sample on every fclk edge, one delta after the edge, so the
    // non-blocking counter update has landed.
    always @(posedge fclk or negedge fclk) begin
        if (rst_n) begin
            #1;
            n_chk = n_chk + 1;
            if (^q1 === 1'bx) begin
                n_x = n_x + 1;
                if (n_x <= 8)
                    $display("OEN-PROBE FAIL q1 is x at t=%0t (cnt=%0d)", $realtime, dut.cnt);
            end else if (q1 !== QPAT[dut.cnt[2:1]]) begin
                errors = errors + 1;
                if (errors <= 8)
                    $display("OEN-PROBE FAIL q1=%b want %b at t=%0t (cnt=%0d)",
                             q1, QPAT[dut.cnt[2:1]], $realtime, dut.cnt);
            end
        end
    end

    // Bound the run so a hang is a report, not a timeout kill.
    initial begin
        #200000;
        $display("OEN-PROBE samples=%0d x_samples=%0d mismatches=%0d", n_chk, n_x, errors);
        if (n_x != 0) begin
            $display("OEN-PROBE FAILED: %0d of %0d samples were x", n_x, n_chk);
            $finish;
        end
        if (errors != 0) begin
            $display("OEN-PROBE FAILED: %0d quadrant mismatches", errors);
            $finish;
        end
        $display("OEN-PROBE PASS");
        $finish;
    end

endmodule
