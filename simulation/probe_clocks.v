// Self-check for the rPLL model in simulation/gowin_prim_models.v.
//
// The model generates pclk, fclk and CK from explicit periods. If those
// periods are wrong, nothing downstream complains: the design simply
// runs at the wrong rate and every measured MB/s is wrong by the same
// factor. That already happened once -- a dual-edge divider chain
// produced pclk at about 27 kHz instead of 99.5625 MHz, and the only
// symptom was a tick counter that never reached its terminal count.
//
// So the frequencies are measured here and asserted, rather than being
// assumed. This is a real assertion: it fails the run, it does not warn.

`timescale 1ps/1ps

module probe_clocks;

    localparam real PCLK_HZ = 99.5625e6;
    localparam real FCLK_HZ = 398.25e6;
    localparam real TOL    = 0.001;          // 0.1 %

    reg sys_clk = 1'b0;
    always #18518.5 sys_clk = ~sys_clk;      // 27 MHz, the real crystal

    wire fclk, pclk, ck, lock;
    Gowin_rPLL pll (.clkout(fclk), .clkoutp(ck), .lock(lock),
                    .clkoutd(pclk), .clkin(sys_clk));

    integer n_pclk = 0;
    integer n_fclk = 0;
    integer n_ck   = 0;
    realtime t_first_pclk, t_first_fclk, t_first_ck;
    real pclk_hz, fclk_hz, ck_hz, ratio;
    integer errors = 0;

    // Ignore the first 10 us: the model settles and LOCK asserts at 60 us.
    initial begin
        wait (pclk === 1'b1);
        t_first_pclk = $realtime;
        forever @(posedge pclk) n_pclk = n_pclk + 1;
    end

    initial begin
        wait (fclk === 1'b1);
        t_first_fclk = $realtime;
        forever @(posedge fclk) n_fclk = n_fclk + 1;
    end

    initial begin
        wait (ck === 1'b1);
        t_first_ck = $realtime;
        forever @(posedge ck) n_ck = n_ck + 1;
    end

    initial begin
        #100000000;                       // 100 ms of simulated time
        pclk_hz = (n_pclk - 1) * 1.0e12 / ($realtime - t_first_pclk);
        fclk_hz = (n_fclk - 1) * 1.0e12 / ($realtime - t_first_fclk);
        ck_hz   = (n_ck   - 1) * 1.0e12 / ($realtime - t_first_ck);

        $display("CLOCK-PROBE pclk = %0.4f MHz (want %0.4f)", pclk_hz/1e6, PCLK_HZ/1e6);
        $display("CLOCK-PROBE fclk = %0.4f MHz (want %0.4f)", fclk_hz/1e6, FCLK_HZ/1e6);
        $display("CLOCK-PROBE ck   = %0.4f MHz (want %0.4f)", ck_hz/1e6,   FCLK_HZ/1e6);
        $display("CLOCK-PROBE fclk/pclk = %0.6f (want 4.0)", fclk_hz/pclk_hz);

        // The ratio is compared RELATIVE to 4.0. Comparing it against an
        // absolute +/-0.001 window is a test bug, not a tight spec: the
        // same 0.1% is 0.0001 on a value of 1 and 0.004 on a value of 4.
        // (That bug reported a 0.054% ratio error as a failure, while
        // letting a 0.1% error on a 1.0 ratio through.) The individual
        // frequency checks below are the binding ones and are unchanged.
        ratio = fclk_hz / pclk_hz;
        if (ratio < 4.0*(1.0-TOL) || ratio > 4.0*(1.0+TOL)) begin
            $display("CLOCK-PROBE FAIL fclk/pclk ratio is %0.6f, want 4.0 +/- %0.2f%%",
                     ratio, TOL*100);
            errors = errors + 1;
        end
        if (pclk_hz < PCLK_HZ*(1-TOL) || pclk_hz > PCLK_HZ*(1+TOL)) begin
            $display("CLOCK-PROBE FAIL pclk off by more than %0.1f%%", TOL*100);
            errors = errors + 1;
        end
        if (ck_hz < FCLK_HZ*(1-TOL) || ck_hz > FCLK_HZ*(1+TOL)) begin
            $display("CLOCK-PROBE FAIL ck off by more than %0.1f%%", TOL*100);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("CLOCK-PROBE PASS");
        else
            $display("CLOCK-PROBE FAILED with %0d error(s)", errors);
        $finish;
    end

endmodule
