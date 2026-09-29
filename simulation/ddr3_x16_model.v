// Behavioural DDR3 x16 SDRAM model, written for this repo.
//
// WHY THIS EXISTS
// ---------------
// The shipped simulation/ harness needs Micron's ddr3.v, subtest.vh and
// 1024*.vh, none of which are in the repo, plus Gowin's prim_sim.v, which
// lives only inside the container image. AGENTS.md rule 2 requires the
// tree to be self-contained and every source tracked, so this is a
// from-scratch model rather than a download.
//
// SCOPE, STATED PLAINLY
// ---------------------
// This model is a FUNCTIONAL model, not a timing-accurate one. It
// implements the command decode the controller actually issues --
// ACT, PRE, REF, MRS, ZQCL, BL8 read, BC4+DM write, NOP -- and it
// honours burst length, DM masking, row/bank/column address mapping and
// CKE/nRESET. It does NOT model tRCD, tRP, tRAS, tWR, tCCD, tREFI, tRFC,
// ODT, write levelling, read calibration, or timing-violation reporting.
//
// CONSEQUENCE, WHICH MATTERS: a rate measured against THIS model is a
// rate measured on the DUT's FSM, not on the DRAM. That is exactly the
// quantity under test (how many commands per pclk the controller
// accepts) and it is the same quantity the on-chip counters report on
// hardware. It is NOT evidence that the DDR3 timing rules are met. That
// needs silicon or a timing-accurate model; this file is not one.
//
// DDR RATE ON THE DQ BUS
// ----------------------
// DDR3 transfers two bits per pin per CK cycle: one on the rising edge
// and one on the falling edge of CK. With FCLK = 400 MHz (CK = fclk)
// that's 800 Mbps per pin. The original first-party model drove dq_o
// once per posedge ck and held it for a full CK cycle, so the IDES8
// saw the same bit twice for two DDR transfers and only the first half
// of every burst made it across the deserialiser. The fix drives a new
// 16-bit dq_o on BOTH edges of CK, and advances the beat counter on
// both edges too, so the burst produces 8 distinct values per pin
// spread across 4 CK cycles -- one per DDR transfer. The IDES8, also
// sampled at DDR rate (see simulation/gowin_prim_models.v), captures
// the full burst.
//
// Command decode and the BL/DM behaviour follow the DDR3 JEDEC command
// truth table and this controller's own use of it.
// Apache-2.0, consistent with this repo.

`timescale 1ps /1ps

module ddr3_x16_model #(
    parameter COL_WIDTH  = 10,
    parameter ROW_WIDTH  = 13,
    parameter BANK_WIDTH = 3
) (
    input              ck,
    input              ncs,
    input              nras,
    input              ncas,
    input              nwe,
    input      [13:0]  a,
    input      [2:0]   ba,
    input      [1:0]   dm,
    input      [15:0]  dq_i,
    output reg  [15:0] dq_o,
    output             dq_oen,
    output             dqs_o,
    input              cke,
    input              nreset
);

    // ---- command encoding, as driven by ddr3_controller.v ----
    localparam CMD_NOP          = 3'b111;
    localparam CMD_SetModeReg   = 3'b000;
    localparam CMD_AutoRefresh  = 3'b001;
    localparam CMD_PreCharge    = 3'b010;
    localparam CMD_BankActivate = 3'b011;
    localparam CMD_Write        = 3'b100;
    localparam CMD_Read         = 3'b101;
    localparam CMD_ZQCL         = 3'b110;

    localparam ROWS  = 1 << ROW_WIDTH;    // 8192 rows per bank
    localparam COLS  = 1 << COL_WIDTH;    // 1024 words per row
    localparam BANKS = 1 << BANK_WIDTH;   // 8 banks
    localparam WORDS = ROWS * COLS * BANKS;

    reg [15:0] mem [0:WORDS-1];

    // ---- row/bank state ----
    reg [ROW_WIDTH-1:0] open_row [0:BANKS-1];
    reg [BANKS-1:0]     row_open;

    // ---- read burst in progress ----
    // Beat counter advances on BOTH edges of CK so 8 beats span 4 CK
    // cycles (8 DDR transfers = BL8). reading is held for rd_len
    // transfers.
    reg                reading = 1'b0;
    reg [COL_WIDTH-1:0]  rd_col;
    reg [ROW_WIDTH-1:0]  rd_row;
    reg [BANK_WIDTH-1:0] rd_bank;
    reg [4:0]            rd_beat;
    reg [4:0]            rd_len;

    // ---- write burst in progress ----
    reg                 writing = 1'b0;
    reg [COL_WIDTH-1:0]  wr_col;
    reg [ROW_WIDTH-1:0]  wr_row;
    reg [BANK_WIDTH-1:0] wr_bank;
    reg [4:0]            wr_beat;
    reg [4:0]            wr_len;
    reg [1:0]            wr_dm;

    integer refresh_count = 0;
    integer rfp_count     = 0;   // refresh with a bank still open

    // ---- write levelling feedback (STUB) ----
    // The controller's WRITE_LEVELING state loops on
    //     if (~DDR3_DQ[0] || ~DDR3_DQ[8]) wstep <= wstep + 1;   // retry
    // until the DRAM returns the levelling feedback on those two DQ
    // bits (ddr3_controller.v:527). Real DRAM asserts them in response
    // to the DQS test pulse. This model has no eye and no DQS sampling,
    // so without a stub the loop never terminates and the simulation
    // hangs.
    //
    // THIS STUB IS NOT EVIDENCE ABOUT WRITE LEVELLING. It exists only so
    // the controller proceeds past init to the part actually under test
    // -- command acceptance rate. Whether wstep is correct on silicon is
    // a separate question, answered only on the bench.
    reg wl_mode     = 1'b0;
    reg wl_feedback = 1'b0;

    // dq_oen is active low (out_enable_n), matching how the controller
    // drives its own dq_oen. dqs_o is released (high) except during a
    // read burst.
    assign dq_oen = ~(reading | writing | (wl_mode & wl_feedback));
    assign dqs_o  = reading ? 1'b0 : 1'b1;

    integer i;
    initial begin
        row_open = {BANKS{1'b0}};
        for (i = 0; i < BANKS; i = i + 1)
            open_row[i] = {ROW_WIDTH{1'b0}};
        // NOTE: `mem` is deliberately NOT initialised here. It has
        // ROWS*COLS*BANKS = 67,108,864 entries, and a time-0 loop over all
        // of them costs minutes of wall clock before the simulation has
        // simulated a single nanosecond. Leaving it as x is safe: the DUT
        // writes before it reads (WIPE, then WRITE_BLOCK, then
        // VERIFY_BLOCK), so every location the verify touches was written
        // first. Locations outside the test are never read and never
        // appear in a comparison.
    end

    function integer mem_addr;
        input [BANK_WIDTH-1:0] b;
        input [ROW_WIDTH-1:0]   r;
        input [COL_WIDTH-1:0]   c;
        begin
            mem_addr = (b * ROWS + r) * COLS + c;
        end
    endfunction

    // ---- command decode ----
    // Every command this controller issues is launched on an OSER8 aligned
    // to the rising edge of CK and is valid for one CK cycle, so
    // sampling the command bus on posedge CK is sufficient and makes the
    // model's beat timing match the controller's own view of time.
    integer trace_count = 0;
    always @(posedge ck) begin
        if (!nreset) begin
            row_open = {BANKS{1'b0}};
            reading  = 1'b0;
            writing  = 1'b0;
        end else if (cke && ncs === 1'b0) begin
            // Trace the first commands so a stuck run says which command
            // it last decoded, instead of only that it is stuck.
            if (trace_count < 400) begin
                $display("DDR3-CMD t=%0t {nRAS,nCAS,nWE}=%b ba=%0d a=%h",
                         $time, {nras, ncas, nwe}, ba, a);
                trace_count = trace_count + 1;
            end
            case ({nras, ncas, nwe})
                CMD_BankActivate: begin
                    open_row[ba] = a[ROW_WIDTH-1:0];
                    row_open[ba] = 1'b1;
                end

                CMD_PreCharge: begin
                    if (a[10]) begin
                        row_open = {BANKS{1'b0}};    // precharge-all
                    end else begin
                        row_open[ba] = 1'b0;
                    end
                end

                CMD_AutoRefresh: begin
                    // DDR3 requires every bank precharged before REF.
                    // This is a live correctness question once row-open
                    // lands, so flag it rather than silently tolerate it.
                    if (row_open != {BANKS{1'b0}}) begin
                        rfp_count = rfp_count + 1;
                        if (rfp_count <= 4)
                            $display("DDR3-MODEL-WARN t=%0t AutoRefresh with %0d bank(s) still open",
                                     $time, $countones(row_open));
                    end
                    row_open      = {BANKS{1'b0}};
                    refresh_count = refresh_count + 1;
                end

                CMD_Read: begin
                    // MR0.M_BL = 2'b01, so A[12] selects BL8 on the fly
                    // (ddr3_controller.v:196). A[10] is auto-precharge.
                    rd_len  = a[12] ? 5'd8 : 5'd4;
                    rd_bank = ba;
                    rd_row  = open_row[ba];
                    rd_col  = a[COL_WIDTH-1:0];
                    rd_beat = 5'd0;
                    reading = 1'b1;
                    if (a[10]) row_open[ba] = 1'b0;   // auto-precharge
                end

                CMD_Write: begin
                    // A BC4 write burst always starts on a 4-word boundary
                    // regardless of the exact address (see the comment at
                    // ddr3_controller.v:418); DM then masks the word.
                    wr_len  = a[12] ? 5'd8 : 5'd4;
                    wr_bank = ba;
                    wr_row  = open_row[ba];
                    wr_col  = a[COL_WIDTH-1:0] & ((wr_len == 5'd8) ? 10'd7 : 10'd3);
                    wr_beat = 5'd0;
                    writing = 1'b1;
                    if (a[10]) row_open[ba] = 1'b0;   // auto-precharge
                end

                CMD_SetModeReg: begin
                    // MR1[7] puts the part into write-levelling mode.
                    // A[7] on a BA=0 MRS is MR1 here; the controller
                    // drives `MR1 | 8'b1000_0100` at
                    // ddr3_controller.v:502 to enter it.
                    wl_mode = a[7];
                    if (wl_mode) begin
                        wl_feedback = 1'b1;   // STUB, see the note above
                        dq_o        = 16'h0101; // DQ[0] and DQ[8] high
                    end
                end

                // CMD_ZQCL and CMD_NOP have no effect on the data
                // contents in this functional model.
                default: ;
            endcase
        end
    end

    // ---- DDR data path ----
    // Two transfers per CK cycle: one on posedge ck, one on negedge ck.
    // The IDES8 (DQSR90 = fclk in simulation/gowin_prim_models.v) samples
    // on both ICLK edges, so eight distinct values per BL8 burst are
    // needed to match what the IDES captures. Driving dq_o on both edges
    // and advancing rd_beat on both edges produces those 8 distinct
    // values across 4 CK cycles.
    always @(posedge ck) begin
        if (reading) begin
            dq_o <= mem[mem_addr(rd_bank, rd_row, rd_col + rd_beat[3:0])];
            if (rd_beat == rd_len - 1)
                reading <= 1'b0;
            rd_beat <= rd_beat + 5'd1;
        end
        if (writing) begin
            wr_dm <= dm;
            if (wr_len == 5'd4) begin
                if (!wr_dm[0])
                    mem[mem_addr(wr_bank, wr_row, {wr_col[COL_WIDTH-3:0], 2'b00})] <= dq_i[15:8];
                if (!wr_dm[1])
                    mem[mem_addr(wr_bank, wr_row, {wr_col[COL_WIDTH-3:0], 2'b10})] <= dq_i[7:0];
            end else begin
                mem[mem_addr(wr_bank, wr_row, wr_col + wr_beat[3:0])] <= dq_i;
            end
            if (wr_beat == wr_len - 1)
                writing <= 1'b0;
            wr_beat <= wr_beat + 5'd1;
        end
    end

    always @(negedge ck) begin
        if (reading) begin
            dq_o <= mem[mem_addr(rd_bank, rd_row, rd_col + rd_beat[3:0])];
            if (rd_beat == rd_len - 1)
                reading <= 1'b0;
            rd_beat <= rd_beat + 5'd1;
        end
        if (writing) begin
            wr_dm <= dm;
            if (wr_len == 5'd4) begin
                if (!wr_dm[0])
                    mem[mem_addr(wr_bank, wr_row, {wr_col[COL_WIDTH-3:0], 2'b00})] <= dq_i[15:8];
                if (!wr_dm[1])
                    mem[mem_addr(wr_bank, wr_row, {wr_col[COL_WIDTH-3:0], 2'b10})] <= dq_i[7:0];
            end else begin
                mem[mem_addr(wr_bank, wr_row, wr_col + wr_beat[3:0])] <= dq_i;
            end
            if (wr_beat == wr_len - 1)
                writing <= 1'b0;
            wr_beat <= wr_beat + 5'd1;
        end
    end

    // ---- periodic self-report, so a run always ends with evidence ----
    integer report_cycle = 0;
    always @(posedge ck) begin
        if (cke) begin
            report_cycle = report_cycle + 1;
            if (report_cycle % 20000 == 0)
                $display("DDR3-MODEL t=%0t refreshes=%0d rfp_violations=%0d",
                         $time, refresh_count, rfp_count);
        end
    end

endmodule
