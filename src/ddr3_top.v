//
// DDR3 test top level @ 100Mhz
//

`timescale 1ps /1ps

module ddr3_top
(
    input sys_clk,
    input sys_resetn,

    input d7,

	inout  [15:0] DDR3_DQ,   // 16 bit bidirectional data bus
	inout  [1:0] DDR3_DQS,   // DQ strobe for high and low bytes
	output [13:0] DDR3_A,    // 14 bit multiplexed address bus
	output [2:0] DDR3_BA,    // 3 banks
	output DDR3_nCS,  // a single chip select
	output DDR3_nWE,  // write enable
	output DDR3_nRAS, // row address select
	output DDR3_nCAS, // columns address select
	output DDR3_CK,
	output DDR3_nRESET,
	output DDR3_CKE,
    output DDR3_ODT,
    output [1:0] DDR3_DM,

    output [7:0] led,
    output [7:0] led2,

    output uart_txp
);

reg start = 1'b1;      
`ifdef D7_TO_START
    // switch d7 on to start running
    always @(posedge clk) begin
        if (d7) start <= 1;
        if (~sys_resetn) start <= 0;
    end
`endif

reg rd, wr, refresh;
reg [25:0] addr;
reg [15:0] din;
wire [127:0] dout128;
wire [15:0] dout;

localparam FREQ=99_800_000;

localparam [25:0] START_ADDR = 26'h0;
`ifdef SIM
// Simulation bulk size. A rate here is a difference of two on-chip
// counters, so it does not depend on the bulk size -- but wall-clock does.
// 8M commands at ~7 pclk each is ~58M pclk cycles, which is hours of
// Icarus. 64 Ki words is 65,536 commands, still a delta large enough to
// pin pclk/command exactly, and it runs in seconds. The full-size numbers
// come from the bench, not from here.
localparam [25:0] TOTAL_SIZE = 64*1024;
`else
localparam [25:0] TOTAL_SIZE = 8*1024*1024;       // Test 8MB
`endif
//localparam [25:0] TOTAL_SIZE = 32*1024*1024;       // Test 64MB

Gowin_rPLL pll(
    .clkout(clk_x4),    // 398.25 Mhz
    .clkoutp(clk_ck),   // 90-degree shifted
    .lock(lock),        
    .clkoutd(clk),      // 99.56 Mhz
    .clkin(sys_clk)     // 27 Mhz
);

wire [7:0] wstep;
reg [1:0] rclkpos;
reg [2:0] rclksel;
wire [63:0] debug;

// ============================================================
// MEASUREMENT INSTRUMENTATION -- dave, 2026-09-28, step 2 of feat/400mbs
//
// Purpose: settle the command-count denominator that the README
// caveat calls the unresolved 1.0132. The published rates were
// inferred from the LEVEL signals rd/wr, which are held high for the
// whole busy window and therefore count hold-cycles, not commands.
// `accept` (src/ddr3_controller.v:100, driven at :344) is the only
// quantity that is exactly one per command actually taken in.
//
// The controller's FUNCTIONAL RTL is unchanged by this commit: no FSM
// state, no command timing and no DDR3 pin behaviour is altered. The
// only edits to src/ddr3_controller.v are three `ifdef IVERILOG guards
// that work around Icarus-only elaboration limits (a forward reference
// to `state`, a duplicate `wire uart_txp`); under synthesis the
// preprocessor discards them and the file reduces to upstream's bytes
// plus the `accept` port.
//
// Everything functional added here is at the top level, and every phase
// boundary is snapshotted ON-CHIP; the numbers are only pushed out over
// the UART at the very end, so printing never perturbs the phase being
// measured.
// ============================================================
wire accept;
wire [31:0] u_cmd_wr, u_cmd_rd, u_pclk;
wire [23:0] u_rf;

// Live counters, all incremented on the same posedge clk.
reg [31:0] m_pclk = 32'd0;
reg [31:0] m_wr   = 32'd0;
reg [31:0] m_rd   = 32'd0;
// NOTE: m_rf counts refreshes ISSUED by this top level, not refreshes
// ACCEPTED by the controller. The controller takes a refresh in IDLE
// with no accept pulse of its own, so "issued" is the strongest claim
// the available observation point supports. Refresh is ~1% of the
// traffic, so this cannot decide the rate -- but it is labelled, not
// quietly rounded.
reg [23:0] m_rf   = 24'd0;

// rd/wr delayed one pclk. The controller registers `accept` on the
// posedge at which it samples rd/wr, so the direction belonging to an
// accept visible at posedge N is the rd/wr that was high at N-1.
reg rd_d, wr_d;

// Phase-end snapshots of the live counters, so a rate is a delta of
// two on-chip numbers and the UART is never inside the measurement.
// Flat registers, not a variable-indexed array: no RAM inference, no
// read-port inference, nothing for the tool to be clever with.
reg [31:0] s0_pclk, s0_wr, s0_rd;  reg [23:0] s0_rf;   // baseline: entry to WIPE
reg [31:0] s1_pclk, s1_wr, s1_rd;  reg [23:0] s1_rf;   // end of WIPE
reg [31:0] s2_pclk, s2_wr, s2_rd;  reg [23:0] s2_rf;   // end of WRITE_BLOCK
reg [31:0] s3_pclk, s3_wr, s3_rd;  reg [23:0] s3_rf;   // end of VERIFY_BLOCK

// Flat snapshot task: the caller's phase boundary picks the slot.
task meas_snap;
    input [1:0] ph;
    begin
        case (ph)
            2'd0: begin s0_pclk <= m_pclk; s0_wr <= m_wr; s0_rd <= m_rd; s0_rf <= m_rf; end
            2'd1: begin s1_pclk <= m_pclk; s1_wr <= m_wr; s1_rd <= m_rd; s1_rf <= m_rf; end
            2'd2: begin s2_pclk <= m_pclk; s2_wr <= m_wr; s2_rd <= m_rd; s2_rf <= m_rf; end
            2'd3: begin s3_pclk <= m_pclk; s3_wr <= m_wr; s3_rd <= m_rd; s3_rf <= m_rf; end
        endcase
    end
endtask

ddr3_controller #(.ROW_WIDTH(13), .COL_WIDTH(10)) u_ddr3 (
    .pclk(clk), .fclk(clk_x4), .ck(clk_ck), .resetn(sys_resetn & lock),
	.addr(addr), .rd(rd), .wr(wr), .refresh(refresh),
	.din(din), .dout128(dout128), .dout(dout), .data_ready(data_ready), .busy(busy),
	.accept(accept),
    .write_level_done(write_level_done), .wstep(wstep),       // write leveling status
    .read_calib_done(read_calib_done), .rclkpos(rclkpos), .rclksel(rclksel),        // read calibration status
    .debug(debug),

    .DDR3_nRESET(DDR3_nRESET),
    .DDR3_DQ(DDR3_DQ),      // 16 bit bidirectional data bus
    .DDR3_DQS(DDR3_DQS),    // DQ strobes
    .DDR3_A(DDR3_A),        // 13 bit multiplexed address bus
    .DDR3_BA(DDR3_BA),      // two banks
    .DDR3_nCS(DDR3_nCS),    // a single chip select
    .DDR3_nWE(DDR3_nWE),    // write enable
    .DDR3_nRAS(DDR3_nRAS),  // row address select
    .DDR3_nCAS(DDR3_nCAS),  // columns address select
    .DDR3_CK(DDR3_CK),
    .DDR3_CKE(DDR3_CKE),
    .DDR3_ODT(DDR3_ODT),
    .DDR3_DM(DDR3_DM)
);

localparam INIT = 0;
localparam PRINT_STATUS = 1;
localparam WRITE1 = 2;
localparam WRITE2 = 3;
localparam WRITE3 = 4;
localparam READ_START = 5;
localparam READ = 6;
localparam READ_DONE = 7;
localparam WRITE_BLOCK = 8;
localparam VERIFY_BLOCK = 9;
localparam WIPE = 10;
localparam FINISH = 11;

// ============================================================
// QUEUED DATAPATH (dave, 2026-09-28) -- the 400 MB/s consumer.
//
// WIPE and WRITE_BLOCK now offer one 16 B command per cycle through
// cmd_valid/cmd_data instead of one 16-bit word per busy-wait cycle.
// Each command is a full BL8 burst, so the controller's queue is fed at
// 1 cmd/pclk (1593 MB/s of headroom) and drained at 1 cmd / ISSUE_PCLK.
//
// DATA IS NEVER CONSTANT (AGENTS.md 7). Every one of the 8 words in a
// burst is a different function of the address, so nothing in the 128-bit
// payload can be const-folded and the BL8 write datapath is genuinely
// exercised. VERIFY_BLOCK compares all 128 bits, not the low byte.
// ============================================================
reg         fast_mode;        // 1 during the bulk phases only
reg         cmd_valid;
reg  [25:0] cmd_addr;
reg         cmd_is_write;
reg  [127:0] cmd_data;
wire        cmd_ready;
wire [127:0] rdata;
wire        rvalid;
reg         rready;
wire [2:0]  rbeat;
wire        rbeat_last;

// One 16 B payload whose eight words are eight different functions of the
// command address. Changing in every word and every command.
function [127:0] gen_pattern;
    input [25:0] a;
    reg [15:0] w0, w1, w2, w3, w4, w5, w6, w7;
    begin
        w0 = a[15:0] ^ {6'b0, a[25:16]} ^ 16'd59;
        w1 = w0 ^ 16'h00A5;
        w2 = w0 ^ 16'h5A3C;
        w3 = w0 ^ 16'hC33C;
        w4 = w0 ^ 16'h1F0F;
        w5 = w0 ^ 16'h7E3D;
        w6 = w0 ^ 16'hB4E2;
        w7 = w0 ^ 16'h2D6B;
        gen_pattern = {w7, w6, w5, w4, w3, w2, w1, w0};
    end
endfunction

// And the same pattern regenerated on the verify side, independently.
function [127:0] expect_pattern;
    input [25:0] a;
    reg [15:0] w0, w1, w2, w3, w4, w5, w6, w7;
    begin
        w0 = a[15:0] ^ {6'b0, a[25:16]} ^ 16'd59;
        w1 = w0 ^ 16'h00A5;
        w2 = w0 ^ 16'h5A3C;
        w3 = w0 ^ 16'hC33C;
        w4 = w0 ^ 16'h1F0F;
        w5 = w0 ^ 16'h7E3D;
        w6 = w0 ^ 16'hB4E2;
        w7 = w0 ^ 16'h2D6B;
        expect_pattern = {w7, w6, w5, w4, w3, w2, w1, w0};
    end
endfunction

reg [25:0] vaddr;
reg [7:0] state, end_state;
reg [7:0] work_counter; // 10ms per state to give UART time to print one line of message
reg [7:0] latency_write1, latency_write2, latency_read;

reg error_bit;

reg refresh_needed;
reg refresh_executed;   // pulse from main FSM

// 7.8us refresh
reg [11:0] refresh_time;
localparam REFRESH_COUNT=FREQ/1000/1000*7813/1000;       // one refresh every 781 cycles for 100Mhz

always @(posedge clk) begin
    if (state) begin
        refresh_time <= refresh_time == (REFRESH_COUNT*2-2) ? (REFRESH_COUNT*2-2) : refresh_time + 1;
        if (refresh_time == REFRESH_COUNT) 
            refresh_needed <= 1;
        if (refresh_executed) begin
            refresh_time <= refresh_time - REFRESH_COUNT;
            refresh_needed <= 0;
        end
        if (~sys_resetn) begin
            refresh_time <= 0;
            refresh_needed <= 0;
        end
    end
end

reg refresh_cycle;
reg [23:0] refresh_count;
reg [24:0] refresh_addr;

reg [63:0] debug_dq_in_buf [15:0];
reg [3:0] debug_cycle;
reg [19:0] tick_counter;        // 0.01s max
reg tick;
reg result_to_print;            // pulse for print control to print a line of result
reg [15:0] expected, actual;
reg [127:0] actual128;
reg [25:0] addr_read;
reg wlevel_feedback;
reg wlevel_done = 0;
reg rlevel_done = 0;
reg [7:0] read_level_cnt;

// LED module in right-bottom PMOD
assign led = ~{state[3:0], busy, error_bit, read_calib_done, write_level_done}; 
assign led2 = ~wstep;       // for write leveling
//assign led2 = ~{read_calib_done, 2'b0, rclkpos[1:0], rclksel[2:0]};   // for read calib

typedef logic [7:0] BYTE;
typedef logic [25:0] ADDR;

// The counter always block lives here, after the declarations it reads
// (`refresh_executed` is declared at the top of this module, well above
// the instrumentation block). Verilog requires declaration before use;
// the Gowin synthesizer does not, Icarus does.

always @(posedge clk) begin
    rd_d <= rd;
    wr_d <= wr;
    m_pclk <= m_pclk + 32'd1;
    if (accept) begin
        if (rd_d) m_rd <= m_rd + 32'd1;
        if (wr_d) m_wr <= m_wr + 32'd1;
    end
    if (refresh_executed) m_rf <= m_rf + 24'd1;
    if (~sys_resetn) begin
        m_pclk <= 32'd0;
        m_wr   <= 32'd0;
        m_rd   <= 32'd0;
        m_rf   <= 24'd0;
    end
end

always @(posedge clk) begin
    wr <= 0; rd <= 0; refresh <= 0; refresh_executed <= 0;
    cmd_valid <= 1'b0;
    // The queued datapath owns the engine only during the bulk phases. The
    // single-word self-tests above must keep the legacy rd/wr path, which is
    // also the check that the legacy path still works after this change.
    fast_mode <= (state == WIPE) || (state == WRITE_BLOCK) || (state == VERIFY_BLOCK);
    work_counter <= work_counter + 1;
    tick_counter <= tick_counter == 0 ? 0 : tick_counter - 20'd1;
    tick <= tick_counter == 20'd1;

    case (state)
        // wait for busy==0 (controller initialization done)
        INIT: if (lock && sys_resetn && !busy && start) begin
            state <= PRINT_STATUS;
            tick_counter <= 20'd100_000;
        end
        PRINT_STATUS: if (tick) begin
            tick_counter <= 20'd100_000;
            work_counter <= 0;
            addr = START_ADDR;
            state <= WRITE1;
        end

        // Part 1 - single write/read test
        WRITE1: if (tick) begin 
            wr <= 1'b1;
            addr <= 26'h0000;
            din <= 16'h1122;
            work_counter <= 0;
            state <= WRITE2;      /* WRITE2 */
            tick_counter <= 20'd100_000;        // 1ms
        end
        WRITE2: if (tick) begin 
            wr <= 1'b1;
            addr <= 26'h0001;
            din <= 16'h3344;
            work_counter <= 0;
            state <= WRITE3;      /* WRITE2 */
            tick_counter <= 20'd100_000;        // 1ms
        end
        WRITE3: if (tick) begin
            if (busy) error_bit <= 1;
            // record write latency and issue another write command
            latency_write1 <= work_counter[7:0]; 
            wr <= 1'b1;
            addr <= 26'h0002;
            din <= 16'h5566;
            state <= READ_START;
            work_counter <= 0;
            debug_cycle <= 0;
            tick_counter <= 20'd100_000;        // wait 1ms
        end

        READ_START: if (tick) begin
            addr[15:0] <= 16'h0000;
            tick_counter <= 20'd100_000;        // wait 1ms
            state <= READ;
        end
        READ: begin
            result_to_print <= 0;
            if (tick) begin
                // issue one read command every tick
                if (addr[15:0] == 16'h0003) begin
                    tick_counter <= 20'd200_000;    // wait 2ms
                    state <= READ_DONE;
                end else begin
                    rd <= 1'b1;
                    tick_counter <= 20'd200_000;    // wait 2ms
                end
            end else if (data_ready) begin
                actual <= dout;
                actual128 <= dout128;
                addr_read <= addr;
                result_to_print <= 1'b1;
                addr[15:0] <= addr[15:0] + 16'd1;
            end
        end
        READ_DONE: begin
            meas_snap(2'd0);      // baseline: counters at the head of WIPE
            state <= WIPE;
            work_counter <= 0;
            addr <= START_ADDR;
        end

        // Part 2 - bulk write/read test
        // WIPE: offered one 16 B command per cycle, no busy-wait. The address
        // advances by 8 WORD addresses per command because a BL8 burst
        // covers 8 words, and the low 3 bits are always 0 as BL8 requires.
        WIPE: begin
            if (addr == ADDR'(START_ADDR + TOTAL_SIZE)) begin
                cmd_valid <= 1'b0;
                meas_snap(2'd1);  // end of WIPE
                work_counter <= 0;
                addr <= START_ADDR;
                state <= WRITE_BLOCK;
            end else if (!refresh_needed) begin
                cmd_valid    <= 1'b1;
                cmd_is_write <= 1'b1;
                cmd_addr     <= addr;
                cmd_data     <= 128'd0;
                if (cmd_ready)
                    addr <= addr + 26'd8;
            end else begin
                // Let the queue drain, then let the controller refresh.
                cmd_valid <= 1'b0;
                if (!busy) begin
                    refresh <= 1'b1;
                    refresh_executed <= 1'b1;
                    refresh_cycle <= 1'b1;
                    refresh_count <= refresh_count + 1;
                    refresh_addr <= addr;
                end
            end
        end

        // WRITE_BLOCK: the measured write phase. Changing 16 B payloads,
        // one command per cycle, gated only by cmd_ready.
        WRITE_BLOCK: begin
            if (addr == ADDR'(START_ADDR + TOTAL_SIZE)) begin
                cmd_valid <= 1'b0;
                meas_snap(2'd2);  // end of WRITE_BLOCK
                state <= VERIFY_BLOCK;
                work_counter <= 0;
                addr <= START_ADDR;
                vaddr <= START_ADDR;   // arm the verify-side address
            end else if (!refresh_needed) begin
                cmd_valid    <= 1'b1;
                cmd_is_write <= 1'b1;
                cmd_addr     <= addr;
                cmd_data     <= gen_pattern(addr);
                if (cmd_ready)
                    addr <= addr + 26'd8;
            end else begin
                cmd_valid <= 1'b0;
                if (!busy) begin
                    refresh <= 1'b1;
                    refresh_executed <= 1'b1;
                    refresh_cycle <= 1'b1;
                    refresh_count <= refresh_count + 1;
                    refresh_addr <= addr;
                end
            end
        end

        // VERIFY_BLOCK: the measured read phase. Offers a read command per
        // cycle and retires each 16 B response as it arrives, comparing ALL
        // 128 bits against the independently regenerated pattern.
        VERIFY_BLOCK: begin
            rready <= 1'b1;
            if (rvalid) begin
                actual128 <= rdata;
                if (rdata !== expect_pattern(vaddr)) begin
                    $display("VERIFY MISMATCH addr=%h got=%h want=%h",
                             vaddr, rdata, expect_pattern(vaddr));
                    error_bit <= 1'b1;
                    end_state <= state;
                    state <= FINISH;
                end
                vaddr <= vaddr + 26'd8;
            end
            if (addr == ADDR'(START_ADDR + TOTAL_SIZE)) begin
                cmd_valid <= 1'b0;
                meas_snap(2'd3);  // end of VERIFY_BLOCK
                end_state <= state;
                state <= FINISH;
            end else if (!refresh_needed) begin
                cmd_valid    <= 1'b1;
                cmd_is_write <= 1'b0;
                cmd_addr     <= addr;
                if (cmd_ready)
                    addr <= addr + 26'd8;
            end else begin
                cmd_valid <= 1'b0;
                if (!busy) begin
                    refresh <= 1'b1;
                    refresh_executed <= 1'b1;
                    refresh_cycle <= 1'b1;
                    refresh_count <= refresh_count + 1;
                    refresh_addr <= addr;
                end
            end
        end




    endcase

    if (~sys_resetn) begin
        error_bit <= 1'b0;
        tick <= 1'b0;
        tick_counter <= 20'd100_000;        // wait 1ms for everything to initialize
        latency_write1 <= 0; latency_write2 <= 0; latency_read <= 0;
        refresh_count <= 0;
        vaddr <= START_ADDR;
        state <= INIT;
    end
end


`include "print.v"
defparam tx.uart_freq=115200;
defparam tx.clk_freq=FREQ;
assign print_clk = clk;
// Drive the output port `uart_txp` from the UART instance's wire `txp`
// (declared in print.v). The original `assign txp = uart_txp;` tried to
// drive the local wire from the OUTPUT PORT, which is illegal in SV (you
// cannot read an output port) and left `uart_txp` undriven.
assign uart_txp = txp;

reg[3:0] state_0;
reg[3:0] state_1;
reg[3:0] state_old;
wire[3:0] state_new = state_1;

reg [7:0] print_counters = 0, print_counters_p;
reg [7:0] print_stat = 0, print_stat_p;

// Measurement dump, chained AFTER print_stat so the two printers never
// both call int_print on the same idle cycle (the task drops a request
// that arrives while print_state != IDLE, and two in one cycle would
// overwrite each other's print_buffer). One item per idle cycle, so
// the whole sequence is paced by the UART itself, not by a timer.
reg       meas_go = 0;
reg [7:0] print_meas = 0, print_meas_p;
localparam MEAS_LAST = 8'd35;

typedef logic [3:0] NIB;

always@(posedge clk)begin
    state_1<=state_0;
    state_0<=state;

    if(state_0==state_1) begin //stable value
        state_old<=state_new;

        if(state_old!=state_new)begin//state changes
            if(state_new==INIT)`print("Initializing SDRAM\n",STR);
          
            if (state_new==PRINT_STATUS) begin
                if (write_level_done && read_calib_done) 
                    `print("Write leveling and read calib successful. \n\n{WSTEP[7:0], rclkpos[3:0], rclksel[3:0]}=", STR);
                else
                    `print("Write leveling or read calibration failed. \n\n{WSTEP[7:0], rclkpos[3:0], rclksel[3:0]}=", STR);
            end

            if(state_new==WRITE1)
                    `print({wstep, NIB'(rclkpos), NIB'(rclksel)}, 2);

            if (state_new==WRITE2) `print("\n\n1 - Single write/read tests:\n", STR);
          
            if(state_new==FINISH) begin
                if(error_bit)
                    `print("\n\n2 - Bulk write/read tests: ERROR. See below for actual dout.\n",STR);
                else
                    `print("\n\n2 - Bulk write/read tests: SUCCESS.\n",STR);
                print_stat <= 1;
                meas_go   <= 1;   // arm the measurement dump, chained after print_stat
            end      
        end
    end

    if (result_to_print) print_counters <= 1'b1;        // trigger result printing

    print_counters_p <= print_counters;
    if (print_counters != 0 && print_counters == print_counters_p && print_state == PRINT_IDLE_STATE) begin
        case (print_counters)
        8'd1: `print("\n", STR);
        8'd2: `print(addr_read[15:0], 2);
        8'd3: `print("=", STR);
        8'd4: `print(actual, 2);
//        8'd4: `print(actual128[127:0], 16);      // print everything for debug
        endcase
        print_counters <= print_counters == 8'd255 ? 0 : print_counters + 1;
    end

    print_stat_p <= print_stat;
    if (print_stat != 0 && print_stat == print_stat_p && print_state == PRINT_IDLE_STATE) begin
        case (print_stat)
        8'd1: `print("\nFinal address=", STR);
        8'd2: `print({6'b0, addr[25:0]}, 4);
        8'd3: `print("\nError=", STR);
        8'd4: `print({7'b0, error_bit}, 1);
        8'd5: `print("\nExpected=", STR);
        8'd6: `print(expected[15:0], 2);
        8'd7: `print("\nActual=", STR);
        8'd8: `print(actual[15:0], 2);
//        8'd10: `print(actual128, 16);
        8'd17: `print("\nRefresh counts=", STR);
        8'd18: `print({8'b0, refresh_count}, 4);
        8'd19: `print("\nLast refresh address=", STR);
        8'd20: `print(refresh_addr[23:0], 3);
        8'd255: `print("\n\n", STR);
        endcase
        print_stat <= print_stat == 8'd255 ? 0 : print_stat + 1;
    end

    // ---- measurement dump ----
    // Runs only after print_stat has wrapped to 0, so it cannot collide
    // with the status printer above. Each line is
    //     MEAS<n> <pclk> <cmd_wr> <cmd_rd> <refresh>
    // as hex, fixed width, so the decoder can parse without heuristics.
    // Consecutive snapshots make each phase a difference of two numbers.
    print_meas_p <= print_meas;
    if (meas_go && print_stat == 0 && print_state == PRINT_IDLE_STATE &&
        (print_meas == 0 || print_meas == print_meas_p)) begin
        case (print_meas)
            8'd0:  `print("\nMEAS0 ", STR);
            8'd1:  `print(s0_pclk, 4);
            8'd2:  `print(" ", STR);
            8'd3:  `print(s0_wr, 4);
            8'd4:  `print(" ", STR);
            8'd5:  `print(s0_rd, 4);
            8'd6:  `print(" ", STR);
            8'd7:  `print({8'b0, s0_rf}, 4);
            8'd8:  `print("\nMEAS1 ", STR);
            8'd9:  `print(s1_pclk, 4);
            8'd10: `print(" ", STR);
            8'd11: `print(s1_wr, 4);
            8'd12: `print(" ", STR);
            8'd13: `print(s1_rd, 4);
            8'd14: `print(" ", STR);
            8'd15: `print({8'b0, s1_rf}, 4);
            8'd16: `print("\nMEAS2 ", STR);
            8'd17: `print(s2_pclk, 4);
            8'd18: `print(" ", STR);
            8'd19: `print(s2_wr, 4);
            8'd20: `print(" ", STR);
            8'd21: `print(s2_rd, 4);
            8'd22: `print(" ", STR);
            8'd23: `print({8'b0, s2_rf}, 4);
            8'd24: `print("\nMEAS3 ", STR);
            8'd25: `print(s3_pclk, 4);
            8'd26: `print(" ", STR);
            8'd27: `print(s3_wr, 4);
            8'd28: `print(" ", STR);
            8'd29: `print(s3_rd, 4);
            8'd30: `print(" ", STR);
            8'd31: `print({8'b0, s3_rf}, 4);
            8'd35: `print("\nENDMEAS", STR);
        endcase
        if (print_meas == MEAS_LAST) begin
            print_meas <= 0;
            meas_go    <= 0;
        end else begin
            print_meas <= print_meas + 1;
        end
    end
end


endmodule

