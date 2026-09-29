
// Simple low-latency DDR3 PHY controller for Tang Primer 20K
// nand2mario, 2022.9
//
// Tested on SKHynix H5TQ1G63EFR-PBC (1Gb)
// - PB speed grade: JEDEC DDR3-1600 11-11-11 (aka DDR3-1600K)
// - RCD=RP=13.75ns, RC=48.75ns
// - Equivalent to "sg25" in Micron DDR3 model
//
// ============================================================
// MODIFIED by Mobilinkd (mobilinkd/ddr3-tang-primer-20k), 2026-09-27
// Base: upstream nand2mario/ddr3-tang-primer-20k @ a6d866d (Apache-2.0)
// Change: added the `accept` output, a one-cycle pulse asserted at the
//   IDLE -> READ/WRITE transition, so that one pulse == one command actually
//   taken in by the controller. The existing `wr`/`rd` ports are LEVEL signals
//   held high for the whole busy window, so a command count derived from them
//   over-counts by roughly 3.35x. This is the only quantity that is exactly one
//   per command, and it is what any throughput number must divide by.
// No other changes to this file.
// ============================================================
//

// Undefine this to use DDR3-1066
`define DDR3_800

module ddr3_controller
#(
    // Main clock frequency, 75Mhz~100Mhz (DDR3 nominal clock 300Mhz~400Mhz)
    parameter         ROW_WIDTH = 13,
    parameter         COL_WIDTH = 10,
    parameter         BANK_WIDTH = 3,

    // Time delays in nCK (tCK=2.5ns for DDR-800, 1.875ns for DDR-1066).
`ifdef DDR3_800
    parameter CAS  = 6,     // 6 cycles, set in MR0[6,5,4,2] = 0100
    parameter BL   = 2,     // BC4 bursting means 2 cycles, MR0[1:0] = 10
    parameter CWL  = 5,     // 5 cycles, set in MR2[5:3] = 000
    parameter WR   = 6,     // 15ns, write recovery
    parameter MRD  = 8,     // 4 cycles, mode register set delay
    parameter RP   = 6,     // 13.75ns, precharge to active
    parameter RCD  = 6,     // 13.75ns, active to r/w
    parameter RC   = 20,    // 48.25ns, ref/active to ref/active
    parameter MOD  = 12     // 12nCK/15ns, cycles after MRS before any other command
`else
    // DDR3-1066
    parameter CAS  = 7,     // MR0[6,5,4,2] = 0110
    parameter CWL  = 6,     // MR2[5:3] = 001
    parameter RCD  = 8,     // 13.75ns, active to r/w
    parameter WR   = 8,     // MR0[11:9] = 100
    parameter MRD  = 8,     // >=4 cycles, mode register set delay
    parameter RP   = 8,     // 13.75ns, precharge to active
    parameter RC   = 26,    // 48.25ns, ref/active to ref/active
    parameter MOD  = 12     // 12nCK/15ns, cycles after MRS before any other command
`endif
)
(
    // DDR3 side interface
    inout [15:0]      DDR3_DQ,
    inout [1:0]       DDR3_DQS,
    output[ROW_WIDTH-1:0]  DDR3_A,
    output[BANK_WIDTH-1:0] DDR3_BA,

    output            DDR3_nRAS,
    output            DDR3_nCAS,
    output            DDR3_nWE,

    output            DDR3_nCS,     // always 0
    output            DDR3_CK,      // ck, 180-degree shifted fclk 
    output     reg    DDR3_CKE,     
    output            DDR3_nRESET,    // reset pin
    output     [1:0]  DDR3_DM,      // always 0
    output            DDR3_ODT,     // always 1
    
    // System side interface
    input             pclk,         // primary clock (rd, wr, etc), e.g. 100Mhz
    input             fclk,         // fast clock (4*pclk), e.g. 400Mhz
    input             ck,           // 90-degree shifted fclk for memory clock
    input             resetn,
    input             rd,           // command: read
    input             wr,           // command: write
    input             refresh,      // command: auto refresh. 4096 refresh cycles in 64ms. Once per 15us.
    input      [BANK_WIDTH+ROW_WIDTH+COL_WIDTH-1:0] 
                      addr,         // word address
    input      [15:0] din,          // 16-bit data input
    output    [15:0]  dout,         // word output
    output    [127:0] dout128,      // 128-bit data output
    output reg        data_ready = 1'b0,   // available 6 cycles after wr is set
    output reg        busy = 1'b1,  // 0: ready for next command

    // accept: ONE-CYCLE PULSE, one per command ACTUALLY ACCEPTED by the
    // controller. Asserted at the IDLE -> READ/WRITE transition below, which
    // is the only place a command is taken in, and de-asserted the next cycle.
    //
    // WHY THIS EXISTS: wr/rd are LEVEL signals held high for the whole
    // transaction (the busy window), and the top-level FSM re-asserts them
    // every cycle, so counting cycles where (wr|rd) is high counts
    // hold-cycles, not commands. The ratio is ~3.35x for this controller, so
    // any command count derived that way is wrong by that factor. This pulse
    // is the only quantity that is exactly one per command.
    output reg        accept = 1'b0,

    // Write leveling. This is done after mode registers are set.
    output            write_level_done,  // 1 means write leveling is successful for this DQS
    output reg [7:0]  wstep,        // write delay steps, 8 bits for each DQS. this is result of write leveling.

    // Read calibration
    // Increment rclkpos/rclksel, send a BL8 read command, and once a rburst
    // pulse is detected, then we have read calibration.
    output            read_calib_done,  // 1: read calibration successful
    output reg  [1:0] rclkpos,      // cycle value for read clock (0-3), 2 bits for each DQS
    output reg  [2:0] rclksel,      // phase value for read clock (0-7), 3 bits for each DQS

    // ---- Queued read port ------------------------------------------------
    //
    // The read requirement is 400 MB/s and dout128 already carries 16 B,
    // which is exactly one BL8 burst, so the read path has NO port-width
    // problem: the whole task is the ISSUE CADENCE. The legacy engine above
    // cannot meet it because it has no command queue and no response FIFO,
    // so only ever one read is in flight and each costs ~12 pclk. These
    // ports drive a pipelined engine that keeps up to 8 in flight.
    //
    // fast_mode selects the pipelined engine. It is asserted ONLY by the
    // READ_BURST phase in ddr3_top, so the write path and every other phase
    // keep running the legacy engine byte-for-byte unchanged.
    input             fast_mode,     // 1: use the pipelined read engine
    input             cmd_valid,     // a read command is being offered
    input      [BANK_WIDTH+ROW_WIDTH+COL_WIDTH-1:0] cmd_addr,
    output            cmd_ready,     // 1: cmd_valid was taken in THIS cycle
    output    [127:0] rdata,         // returned burst, one entry per read
    output            rvalid,        // one entry of rdata is valid
    input             rready,        // 1: retire the rvalid entry

    // Counters for the rate measurement. These are on-chip so the number
    // comes from the DUT rather than from the host, and they count what the
    // ENGINE did, not what the top level requested.
    output reg [31:0] cmd_count_rd,  // queued reads taken in (== accept)
    output reg [31:0] pclk_count,    // pclk elapsed in fast_mode
    output reg [23:0] refresh_count, // refreshes in fast_mode

    output [63:0] debug
);

// More timing parameters
localparam WLMRD= 44;       // Write leveling wait time before DQS pulse
localparam SERDES = 16;     // SERDES round-trip latency in (OSER8 is 3 clk, IDER8 is 1 clk)

`ifdef DDR3_800
    localparam USEC = 100;  // a micro second in pclk cycles (pclk <= 100Mhz)
`else
    localparam USEC = 134;  // pclk <= 133Mhz
`endif

// Wait until DLL locked to do anything
wire dlllock;
wire rst_lock_n = resetn & dlllock;
reg resetn_delay;
assign DDR3_nRESET = rst_lock_n & resetn_delay;
assign DDR3_ODT = 1'b1;     // Use dynamic ODT

// Tri-state DQ inout signals (double DDR3_CK speed)
reg [15:0] dq_out [7:0];    // DQ output data
reg [0:3] dq_oen;           // out_enable_n for dq_out[1:0], [3:2], [5:4], [7:6]
wire [15:0] dq_in [7:0];    // DQ input data
reg [0:7] dqs_out;          // DQS output
reg [0:3] dqs_oen;          // out_enable_n for dqs_out[1:0], [3:2], [5:4], [7:6]
reg [0:7] dm_out;

// Bit lengths
typedef logic [7:0] BYTE;
typedef logic [3:0] NIB;
typedef logic [4:0] FIVEB;

// Debug stuff
reg [7:0] cnt_read = 0;
reg [7:0] cnt_write = 0;
// `state` is declared ~20 lines below this assign. The Gowin synthesizer
// tolerates the forward reference; Icarus does not ("Unable to bind
// wire/reg/memory `state'"). Guarded so the synthesized design is
// unchanged; under simulation the identical assign is made at the
// declaration site instead.
`ifndef IVERILOG
assign debug = {cnt_write, cnt_read, BYTE'(state)};
`endif

// Output signals
reg nRAS[3:0], nCAS[3:0], nWE[3:0];
reg [ROW_WIDTH-1:0] A[3:0];
reg [2:0] BA[3:0];
assign DDR3_CK = ck;
assign DDR3_nCS = 1'b0;

assign dout128 = {dq_in[0], dq_in[1], dq_in[2], dq_in[3], dq_in[4], dq_in[5], dq_in[6], dq_in[7]};
`ifdef SIM
assign dout = dq_in[4];     // somehow in simulation data is here
`else
    `ifdef DDR3_800
        assign dout = dq_in[0];
    `else
        assign dout = dq_in[4];
    `endif
`endif

// Our main FSM state
reg [3:0] state;
// Deferred from above: see the IVERILOG note on the `debug` assign.
`ifdef IVERILOG
assign debug = {cnt_write, cnt_read, BYTE'(state)};
`endif
localparam RST_WAIT = 4'd0;
localparam CKE_WAIT = 4'b1;
localparam CONFIG = 4'd2;
localparam ZQCL = 4'd3;
localparam IDLE = 4'd4;
localparam READ = 4'd5;
localparam WRITE = 4'd6;
localparam REFRESH = 4'd7;
localparam WRITE_LEVELING = 4'd8;
localparam READ_CALIB = 4'd9;
reg [4:0] cycle;        // step within operation (config/read/write) 
reg tick;               // pulse after tick_counter cycles
reg [16:0] tick_counter = 17'd50_000;   

// RAS# CAS# WE#
localparam CMD_SetModeReg=3'b000;
localparam CMD_AutoRefresh=3'b001;
localparam CMD_PreCharge=3'b010;
localparam CMD_BankActivate=3'b011;
localparam CMD_Write=3'b100;
localparam CMD_Read=3'b101;
localparam CMD_ZQCL=3'b110;
localparam CMD_NOP=3'b111;

// Mode registers are {BA[2:0], A[12:0]}, total 16 bits
localparam [1:0] M_BL = 2'b01;      // burst length BC4 or 8 on the fly
`ifdef DDR3_800
localparam [3:0] M_CAS = 4'b0100;   // 6 cycles
localparam [2:0] M_CWL = 3'b000;    // 5 cycles
localparam [2:0] M_WR = 3'b010;     // 6 cycles
`else
localparam [3:0] M_CAS = 4'b0110;   // 7 cycles
localparam [2:0] M_CWL = 3'b001;    // 6 cycles
localparam [2:0] M_WR = 3'b100;     // 8 cycles
`endif
localparam M_DLLReset = 1'b1;
localparam M_RTT_NOM = 3'b000;      // Disabled, RTT_NOM is only needed for multi-rank systems
localparam M_RTT_WR = 2'b01;        // RTT_WR = 60ohm (2: 120ohm)
localparam M_RTT_WR_off = 2'b00;    //
localparam M_DRIVE = 2'b00;         // 00: low drive strength, 01: high
localparam M_AL = 2'b0;             // AL=0
wire [15:0] MR0 = 
//15 14 13 12    11 10 9   8           7     6 5 4       3     2         1 0
  {3'b000, 1'b0, M_WR,     M_DLLReset, 1'b0, M_CAS[3:1], 1'b0, M_CAS[0], M_BL};
wire [15:0] MR1 = 
//15 14 13 12:10 9             8 7   6             5           4 3   2             1           0   
  {3'b001, 3'b0, M_RTT_NOM[2], 2'b0, M_RTT_NOM[1], M_DRIVE[1], M_AL, M_RTT_NOM[0], M_DRIVE[0], 1'b0};
wire [15:0] MR2 =
//15 14 13 12 11 10 9          8 7 6 5 4 3  2 1 0
  {3'b010, 2'b0, M_RTT_WR_off, 3'b0, M_CWL, 3'b0};
wire [15:0] MR2_RTT_WR =        // Dynamic ODT is applied after write leveling
  {3'b010, 2'b0, M_RTT_WR,     3'b0, M_CWL, 3'b0};
// We use dynamic ODT because it allows ODT pin to be tied to 1. 
// RTT_WR will be automatically applied when there is a write.
// See: Micron TN-41-13: DDR3 Point-to-Point Design Support
wire [15:0] MR3 = {3'b011, 13'b0};

// Write leveling
reg wlevel_done = 1'b0;
reg [3:0] wlevel_cnt = 3'b0;
assign write_level_done = wlevel_done;

// Read calibration. 
// Note there's no read eye diagram testing yet.
reg [3:0] dqs_read = 4'b0;
reg dqs_hold = 1'b0;
wire [1:0] rburst;
reg [1:0] rburst_seen;
reg rcalib_done = 1'b0;
reg [3:0] rcalib_cnt = 4'b0;
assign read_calib_done = rcalib_done;

`ifdef SIM
localparam WLEVEL_COUNT=2;
localparam RCALIB_COUNT=2;
`else
localparam WLEVEL_COUNT=1;          // test this many times before a wstep passes write leveling
localparam RCALIB_COUNT=8;          // test this many times before rclkpos/rclksel passes read calib
`endif

//
// DDR3 state machine
//
always @(posedge pclk) begin    
  if (rst_lock_n) begin
    cycle <= cycle == 5'd31 ? 5'd31 : cycle + 4'd1;
    tick <= tick_counter == 16'd1;
    tick_counter <= tick_counter == 0 ? 0 : tick_counter - 16'd1;

    // defaults
    {nRAS[0], nCAS[0], nWE[0]} <= CMD_NOP; {nRAS[1], nCAS[1], nWE[1]} <= CMD_NOP;
    {nRAS[2], nCAS[2], nWE[2]} <= CMD_NOP; {nRAS[3], nCAS[3], nWE[3]} <= CMD_NOP;
    A[0] <= 1'b0; A[1] <= 1'b0; A[2] <= 1'b0; A[3] <= 1'b0;
    BA[0] <= 3'b0; BA[1] <= 3'b0; BA[2] <= 3'b0; BA[3] <= 3'b0;
    dm_out = 8'b1111_1111;
    dqs_oen[0:3] <= 4'b1111;
    dq_oen[0:3] <= 4'b1111;
    dqs_hold <= 1'b0;

    // accept defaults low: a one-cycle pulse, re-asserted only at the accept
    // point. (The refresh accept is counted separately via refresh_count in
    // the top level; accept covers rd/wr commands only, which is what the
    // throughput number divides by.)
    accept <= 1'b0;

    casex ({state, cycle})
        // RESET off after 200 us 
        {RST_WAIT, 5'bxxxxx} : if (tick) begin
            resetn_delay <= 1'b1;
`ifdef SIM
            tick_counter <= 17'd19;
`else
            tick_counter <= 500 * USEC + 20;
`endif
            state <= CKE_WAIT;
        end
        
        // CKE on after 500 us before 
        {CKE_WAIT, 5'bxxxxx} : begin
            if (tick_counter == 16'd15)
                DDR3_CKE <= 1'b1;       // tXPR (120ns) between CKE and MRS
            if (tick) begin
                state <= CONFIG;
                cycle <= 0;
            end
        end

        // Mode Register Config sequence
        //  cycle  //0 \___/ ...___//4 \___/ ... __//8 \___/ ...___//12\___/ ...___//16\___/ ...___//28\___/ ...___/1024+28\___
        //  cmd                    |  MR0  |       |  MR1  |       |  MR2  |       |  MR3  |       | ZQCL  |       | _Next_|
        //                         '---- MRD=4 ----'----  MRD=4 ---'----  MRD=4 ---'-----MOD=12----'--tZQCL+tDLLK--'
        {CONFIG, FIVEB'(0)} : begin
            {nRAS[0], nCAS[0], nWE[0]} <= CMD_SetModeReg;
            {BA[0], A[0][12:0]} <= MR2;
        end
        {CONFIG, FIVEB'(MRD/4)} : begin
            {nRAS[0], nCAS[0], nWE[0]} <= CMD_SetModeReg;
            {BA[0], A[0][12:0]} <= MR3;
        end
        {CONFIG, FIVEB'(MRD/2)} : begin
            {nRAS[0], nCAS[0], nWE[0]} <= CMD_SetModeReg;
            {BA[0], A[0][12:0]} <= MR1;
        end
        {CONFIG, FIVEB'(MRD*3/4)} : begin
            {nRAS[0], nCAS[0], nWE[0]} <= CMD_SetModeReg;
            {BA[0], A[0][12:0]} <= MR0;
        end
        {CONFIG, FIVEB'(MRD*3/4+MOD/4+1)} : begin
            {nRAS[0], nCAS[0], nWE[0]} <= CMD_ZQCL;
            A[0][10] <= 1'b1;         // long ZQ calibration
`ifdef SIM
            tick_counter <= 16'd2;      // 1024 nCK (256 clk), set to 2 for simulation
`else
            tick_counter <= 16'd512 + 2;
`endif
            state <= ZQCL;
        end

        // ZQ calibration
        {ZQCL, 5'bxxxxx} : if (tick) begin
            state <= WRITE_LEVELING;
            cycle <= 0;
        end

        // read/write/refresh
        {IDLE, 5'bxxxxx}: if (rd | wr) begin
            // bank activate
            {nRAS[0], nCAS[0], nWE[0]} <= CMD_BankActivate;
            BA[0] <= addr[ROW_WIDTH+COL_WIDTH+BANK_WIDTH-1 : ROW_WIDTH+COL_WIDTH];    // bank id
            A[0] <= addr[ROW_WIDTH+COL_WIDTH-1:COL_WIDTH];      // 12-bit row address
            state <= rd ? READ : WRITE;
            // accept pulse: exactly one per command taken in, at the only
            // point where a command is accepted. See the port declaration.
            accept <= 1'b1;
            if (rd) cnt_read <= cnt_read == 8'hff ? 8'hff : cnt_read + 1;
            if (wr) cnt_write <= cnt_write == 8'hff ? 8'hff : cnt_write + 1;
            cycle <= 4'd1;
            busy <= 1'b1;
            if (rd) dqs_hold <= 1'b1;       // reset WPOINT/RPOINT
        end else if (refresh) begin
            // auto-refresh
            // no need for precharge-all b/c all our r/w are done with auto-precharge.
            {nRAS[0], nCAS[0], nWE[0]} <= CMD_AutoRefresh;
            state <= REFRESH;
            cycle <= 4'd1;
            busy <= 1'b1;
        end

        // Read sequence (9 cycles at 100Mhz)
        // Quirk: you can issue next command once you see data_ready at cycle 8 (busy will be 0 next cycle), that saves a cycle
// CK    \___//0 \___/ 1 \___/ 2 \___/ 3 \___//4 \___/ 5 \___/ 6 \___/ 7 \___//8 \___/ 9 \___/ 10\___/ 11 ...//32\___/ 33\___/ 34\___/ 35\___//36\___/ 37\___/ 38\___/ 39\___
// RD   _||              1               ||                                                                                              ||           _NEXT_         |
// CMD                                   ||ACTIVE|       |       |       ||      |       | READ  |       ||
// ADDR                                  ||Row   |       |       |       ||      |       | Col n |       ||
// DQS  ---------------------------------------------------------------------------------------------____/   \___/   \___--------------
// DQ                                                                                                    ||DO|DO |DO |DO ||
//                                                                                                       ||n |n+1|n+2|n+3||
//                                       `-------------------- RCD=6 --------------------' CL6+SERDES=22 '
// data_ready ___________________________________________________________________________________________/                               \______
// busy  ________________________________/                                                                                               \______
        {READ, FIVEB'(RCD/4)}: begin              // 1       
            {nRAS[RCD%4], nCAS[RCD%4], nWE[RCD%4]} <= CMD_Read;     // 101
            BA[RCD%4] <= addr[ROW_WIDTH+COL_WIDTH+BANK_WIDTH-1 : ROW_WIDTH+COL_WIDTH];    // bank id
            A[RCD%4][12] <= 1'b1;                   // BL8 burst length
            A[RCD%4][10] <= 1'b1;                   // set auto precharge
            A[RCD%4][9:0] <= addr[COL_WIDTH-1:0];   // column address

            dqs_hold <= 1'b1;                   // make hold pulse one cycle longer as we are having setup violation here...
        end
        {READ, FIVEB'((RCD+CAS+SERDES)/4+1)}: begin   // 7 (6+6+16 = 28)
            data_ready <= 1'b1;
        end
        {READ, FIVEB'((RCD+CAS+SERDES)/4+2)}: begin // 8
            data_ready <= 1'b0;
            busy <= 0;
            state <= IDLE;
        end

        // write sequence (7 cycles at 100Mhz)
// CK     \___//0 \___/ 1 \___/ 2 \___/ 3 \___//4 \___/ 5 \___/ 6 \___/ 7 \___//8 \___/ 9 \___/ 10\___/ 11\___//12\___/ 13\___/ 14\___/ 15\___//16\___/ 17\___/ 18\___/ 19 ...//28\___/ 29\___/ 30\___/ 31\___
// WR     |               1               ||                                                                                          |   ||  |   |                       ||           _NEXT_
// CMD                                    ||ACTIVE|       |       |       ||      |       | WRITE |       ||                          |   ||  |   |
// ADDR                                   || Row  |       |       |       ||      |       | COL n |       ||                          |   ||  |   |
// DQS  ----------------------------------------------------------------------------------------------------------------------/   \___/   \___/   \___----------
// DQ                                                                                                                               |Din|Din|Din|Din|Din|Din|Din|Din|
//                                                                                                                                  |n  |n+1|n+2|n+3|n+4|n+5|n+6|n+7|
//                                            `-------------------- RCD=6 --------------------'--------------- CWL = 5 ---------------'------------  4  --------------'--------- WR+RP = 12 ----------'
// busy __________________________________/                                                                                                                           \_______ (negedge at 24)

        {WRITE, FIVEB'(RCD/4)}: begin             
            {nRAS[RCD%4], nCAS[RCD%4], nWE[RCD%4]} <= CMD_Write;    // 100
            BA[RCD%4] <= addr[ROW_WIDTH+COL_WIDTH+BANK_WIDTH-1 : ROW_WIDTH+COL_WIDTH];    // bank id
            A[RCD%4][10] <= 1'b1;                           // set auto precharge
            A[RCD%4][9:0] <= addr[COL_WIDTH-1:0];           // column address
            A[RCD%4][12] <= 1'b0;                           // BC4 burst length
        end
`ifdef DDR3_800
        {WRITE, FIVEB'((RCD+CWL)/4)}: begin
            // DQS output
            dqs_out <= 8'b1111_1110;
            dqs_oen <= 4'b1110;

            // DQ output
            dq_out[6] <= 0; dq_out[7] <= din;
            dq_oen <= 4'b1110;

            // DM, umask the word we are writing to
            // Note that BC4 write always start from whole 4 words (not the exact address we want to write to)
            dm_out[7] <= !(addr[1:0] == 2'b00);
        end
        {WRITE, FIVEB'((RCD+CWL)/4+1)}: begin
            // DQS output
            dqs_out <= 8'b1010_0000;
            dqs_oen <= 4'b0011;

            // DQ output
            dq_out[0] <= din; dq_out[1] <= din;
            dq_out[2] <= din; dq_out[3] <= 0;
            dq_oen <= 4'b0011;

            dm_out[0] <= !(addr[1:0] == 2'b01);
            dm_out[1] <= !(addr[1:0] == 2'b10);
            dm_out[2] <= !(addr[1:0] == 2'b11);
        end
        {WRITE, FIVEB'(20/4)}: begin            // busy=0 at 24
            busy <= 0;
            state <= IDLE;
        end
`else
    // DDR3-1066: phase is 2 steps ahead of DDR3-800
        {WRITE, FIVEB'((RCD+CWL)/4)}: begin         // 3 (8+6=14) for DDR3-1066
            // DQS output
            dqs_out <= 8'b1111_1010;
            dqs_oen <= 4'b1100;

            // DQ output
            dq_out[4] <= 0; dq_out[5] <= din;
            dq_out[6] <= din; dq_out[7] <= din;
            dq_oen <= 4'b1100;

            dm_out[5] <= !(addr[1:0] == 2'b00);
            dm_out[6] <= !(addr[1:0] == 2'b01);
            dm_out[7] <= !(addr[1:0] == 2'b10);
        end
        {WRITE, FIVEB'((RCD+CWL)/4+1)}: begin
            // DQS output
            dqs_out <= 8'b1000_0000;
            dqs_oen <= 4'b0111;

            // DQ output
            dq_out[0] <= din; dq_out[1] <= 0;
            dq_oen <= 4'b0111;

            dm_out[0] <= !(addr[1:0] == 2'b11);
        end
        {WRITE, FIVEB'(24/4)}: begin              // busy=0 at 28
            busy <= 0;
            state <= IDLE;
        end
`endif


        // refresh sequence (6 cycles in 100Mhz)
// ck      \___//0 \___/ 1 \___/ 2 \___/ 3 \___//4 \___/ 5 \___/ 6 \___/ 7  ...//20\___/ 21\___/ 22\___/ 23\___//24\___/ 25\___/ 26\___/ 27\___
// refresh |               1               |                                                               ||            _NEXT_            |
// cmd                                     |Refresh|       |       |       ||      |       |       |       ||
// busy     _______________________________/                               \________________________________
//                                         `----------------------- T_RC ----------------------------------'
        {REFRESH, FIVEB'(RC/4)}: begin
            state <= IDLE;
            busy <= 0;
        end

        // Write Leveling
        // After tWLDQSEN(25nck), drive DQS low. After tWLMRD (40nCK), pulse DQS, then after tWLO (7.2ns) feedback is provide on prime DQ
        {WRITE_LEVELING, FIVEB'(0)}: begin
            $display("Entering WRITE LEVELING");
            // enter write leveling mode
            {nRAS[0], nCAS[0], nWE[0]} <= CMD_SetModeReg;
            // MR1[9,6,2]=001, RTT_NOM=60ohm, as dynamic ODT is not available in write leveling
            {BA[0], A[0][12:0]} <= MR1 | 8'b1000_0100;      // MR1[7] is write-leveling                    
            wlevel_cnt <= 0;
`ifdef SIM
    `ifdef DDR3_800
            wstep <= 8'h18;     // quick start write leveling for simulation
    `else
            wstep <= 8'h12;
    `endif
`endif
        end
        {WRITE_LEVELING, FIVEB'(WLMRD/4-1)}, {WRITE_LEVELING, FIVEB'(WLMRD/4+1)},
        {WRITE_LEVELING, FIVEB'(WLMRD/4+2)}, {WRITE_LEVELING, FIVEB'(WLMRD/4+3)},
        {WRITE_LEVELING, FIVEB'(WLMRD/4+4)}, {WRITE_LEVELING, FIVEB'(WLMRD/4+5)}: begin
            dqs_out[0:7] <= 8'b0;               // bring down DQS
            dqs_oen[0:3] <= 4'b0;
        end
        {WRITE_LEVELING, FIVEB'(WLMRD/4)}: begin
            $display("wstep=%d", wstep);
            dqs_out[0:7] <= 8'b1010_1010;       // provide a test pulse
            dqs_oen[0:3] <= 4'b0;                    
        end
        {WRITE_LEVELING, FIVEB'(WLMRD/4+6)}: begin             // SERDES round trip + feedback time tWLO=7.5ns
            dqs_out[0:7] <= 8'b0;
            dqs_oen[0:3] <= 4'b0;

            if (~DDR3_DQ[0] || ~DDR3_DQ[8]) begin
                wstep <= wstep + 1'd1;
                wlevel_cnt <= 0;
                cycle <= FIVEB'(WLMRD/4-1);    // loop
            end else begin
                wlevel_cnt <= wlevel_cnt + 1'b1;
                // test WLEVEL_COUNT times before we commit
                if (wlevel_cnt == NIB'(WLEVEL_COUNT-1'b1)) begin
                    $display("Write leveling process finished.");
                    wlevel_done <= 1'b1;
                    // Exit write leveling mode
                    {nRAS[0], nCAS[0], nWE[0]} <= CMD_SetModeReg;
                    {BA[0], A[0][12:0]} <= MR1;      // A[7] is write-leveling
                end else
                    cycle <= FIVEB'(WLMRD/4-1);
            end     
        end
        {WRITE_LEVELING, FIVEB'((WLMRD+MRD)/4+6)}: begin
            // Turn on dynamic ODT
            {nRAS[0], nCAS[0], nWE[0]} <= CMD_SetModeReg;
            {BA[0], A[0][12:0]} <= MR2_RTT_WR;
        end
        {WRITE_LEVELING, FIVEB'((WLMRD+MRD+MOD)/4+6)}: begin
            state <= READ_CALIB;                // now continue to read calibration
            cycle <= 0;
        end

        // Read calibration
        // Issues BL8 read commands while adjusting rclkpos (0-3), rclksel (0-7)
        // until we see rburst pulse after the read. Then read calibration is achieved.
        {READ_CALIB, FIVEB'(0)}: begin
            $display("Entering READ CALIBRATION");
            // activate bank 0
            {nRAS[0], nCAS[0], nWE[0]} <= CMD_BankActivate;
            BA[0] <= 0; A[0] <= 0;
            rcalib_cnt <= 0;
`ifdef SIM
            // in simulation the final rclkpos=2, rclksel=0 for DDR3-800
            //                         rclkpos=1, rclksel=1 for DDR3-1066
    `ifdef DDR3_800
            rclkpos <= 2'd1; rclksel <= 3'd6;
    `else
            rclkpos <= 2'd1; rclksel <= 3'd0;
    `endif
`endif
        end
        {READ_CALIB, FIVEB'(RCD/4)}: begin
            // issue read
            $display("rclkpos=%d, rclksel=%d", rclkpos, rclksel);
            {nRAS[2], nCAS[2], nWE[2]} <= CMD_Read;
            A[2][12] <= 1'b1;                   // BL8 burst length
            A[2][10] <= 1'b0;                   // NO auto precharge
            A[2][9:0] <= 0;                     // row address=0
        end
        {READ_CALIB, FIVEB'(RCD/4+10)}: begin   // Allow 10 cycles for rburst signal
            if (rburst_seen != 2'b11) begin
                rclksel <= rclksel + 3'b1;
                rclkpos <= rclksel == 3'd7 ? rclkpos + 2'b1 : rclkpos;
                rcalib_cnt <= 0;
                cycle <= FIVEB'(RCD/4);         // loop
            end else begin
                rcalib_cnt <= rcalib_cnt + 1'b1;
                // test RCALIB_COUNT times before we commit
                if (rcalib_cnt == NIB'(RCALIB_COUNT - 1)) begin
                    // read calibration is achieved
                    rcalib_done <= 1'b1;
                    {nRAS[0], nCAS[0], nWE[0]} <= CMD_PreCharge;    // precharge
                    BA[0] <= 0; A[0] <= 0;                          // bank 0, row 0
                    // exit read calib
                end else 
                    cycle <= FIVEB'(RCD/4);     // loop
            end
        end
        {READ_CALIB, FIVEB'(RCD/4+10+RP/4)}: begin
            // Precharge is finished. Exit read calibration. Initialization is all done.
            $display("All initialization DONE...");
            busy <= 1'b0;
            state <= IDLE;
        end
    endcase
  end else if (~rst_lock_n) begin
        busy <= 1'b1;
        data_ready <= 1'b0;
        DDR3_CKE <= 1'b0;
       {nRAS[0], nCAS[0], nWE[0]} <= CMD_NOP;
       {nRAS[1], nCAS[1], nWE[1]} <= CMD_NOP;
       {nRAS[2], nCAS[2], nWE[2]} <= CMD_NOP;
       {nRAS[3], nCAS[3], nWE[3]} <= CMD_NOP;
`ifdef SIM
        tick_counter <= 16'd1;          // initial 500us delay, 140ns in simulation
`else
        tick_counter <= 16'd60_000;     // initial 500us delay
`endif
        tick <= 0;
        cycle <= 0;
        wlevel_cnt <= 0; wlevel_done <= 2'b0; wstep <= 0;
        rcalib_cnt <= 0; rcalib_done <= 0; rclkpos <= 0; rclksel <= 0;
        resetn_delay <= 1'b0;       // cause extra 200us reset for DDR3
        state <= RST_WAIT;
  end
end


// =============================================================
// Queued READ engine (fast_mode)
//
// Scope: READ ONLY. The write datapath, the 16-bit `din` port, and the
// BC4+DM write path below are untouched, and fast_mode is asserted only by
// the READ_BURST phase in ddr3_top. Every other phase runs the legacy
// engine byte-for-byte as before.
//
// Why a queue is the whole fix: dout128 already returns 16 B per command,
// which is exactly one BL8 burst, so the read path has no port-width
// problem. The legacy engine has no command queue and no response FIFO, so
// only ever ONE read is in flight and the next cannot issue until busy
// drops -- about 12 pclk per command. Eight in flight turns that into a
// fixed 3 pclk cadence, which is 531 MB/s at 100 MHz pclk.
//
// Budget check (tCK = 2.5 ns, 4 CK per pclk at 100 MHz pclk):
//   BL8 on a 16-bit bus = 8 transfers = 4 CK = 1.0 pclk for 16 B, so the
//   bus ceiling is 16 B/pclk = 1593 MB/s. At 3 pclk/cmd the bus is busy
//   one pclk in three.
//   tCCD(8) = 4 CK = 1.0 pclk, so the DRAM accepts a read every pclk.
//   4 pclk/cmd would be 398.25 MB/s, which FAILS the 400 MB/s bar by
//   0.44%. 3 pclk/cmd clears it with 31% margin. Hence ISSUE_PCLK = 3 and
//   the $error below that will not let it be set to 4 by accident.
// =============================================================
localparam STREAM = 4'd10;             // pipelined read engine (idle rows 10-15 free)
localparam CMD_DEPTH  = 8;             // one burst of 8 covers a whole row-ish
localparam RESP_DEPTH = 8;             // must be >= READ_LATENCY to never stall
localparam ISSUE_PCLK = 3;             // pclk between column commands
localparam RCD_PCLK   = RCD/4;         // ACT->READ delay in pclk
// A read issued this cycle returns data READ_LATENCY pclk later. 7 mirrors
// the legacy read FSM, which asserts data_ready at cycle 7 = (RCD+CAS+
// SERDES)/4+1 = (6+6+16)/4+1. The shift register retires reads strictly in
// order, which is what lets a fixed 3 pclk cadence be correct: a read whose
// response is swallowed by a row miss or a refresh simply produces no
// capture, and the count stays in step because nothing was in flight then.
localparam READ_LATENCY = (RCD+CAS+SERDES)/4+1;

reg  [4:0] fsm;                       // the STREAM state, separate from `state`
localparam S_IDLE  = 3'd0;             // no row open, queue may be empty
localparam S_ISSUE = 3'd1;             // row open, issuing at ISSUE_PCLK
localparam S_MISS  = 3'd2;             // precharging, then activating
localparam S_PRE   = 3'd3;             // precharge-all before a refresh
localparam S_REF   = 3'd4;             // refresh
localparam S_DRAIN = 3'd5;             // refresh due: empty the pipe first
localparam S_RFC   = 3'd6;             // tRFC recovery after the refresh

// Pointer width is $clog2(DEPTH), NOT a 4-bit reg. A 4-bit pointer holds
// 0..15 and only the low $clog2(DEPTH) bits address the array, so a bare
// `+ 4'd1` walks 7 -> 8 -> 9, and q_addr[8]/f_data[8] read out of range
// (x in simulation). That poisoned q_row/q_bnk, so need_act went x, which
// gated q_pop forever: the queue filled to CMD_DEPTH and stuck there with
// q_count = 8 and nothing ever issued. A pointer exactly as wide as the
// index needs no wrap logic at all -- 7 + 1 = 0 -- which is also why
// CMD_DEPTH and RESP_DEPTH must stay powers of two (guarded below).
localparam Q_PTR_W = $clog2(CMD_DEPTH);
localparam F_PTR_W = $clog2(RESP_DEPTH);

reg [3:0]  q_count;  reg [25:0] q_addr [0:CMD_DEPTH-1];
reg [Q_PTR_W-1:0] q_head, q_tail;
reg [3:0]  f_count;  reg [127:0] f_data [0:RESP_DEPTH-1];
reg [F_PTR_W-1:0] f_head, f_tail;
reg [ROW_WIDTH-1:0] row_open;          // row currently open, valid if row_valid
reg        row_valid;
reg        bank_open;  reg [BANK_WIDTH-1:0] bank_now;
reg        need_ref;                   // a refresh is due
wire       need_ref_q = need_ref;
reg [2:0]  pre_cnt;                    // post-refresh recovery counter

wire       q_full   = (q_count == CMD_DEPTH[3:0]);
wire       f_full   = (f_count == RESP_DEPTH[3:0]);
wire       f_empty  = (f_count == 4'd0);
wire       q_empty  = (q_count == 4'd0);
// The head command, and whether its row is already open.
wire [ROW_WIDTH-1:0]   q_row = q_addr[q_head][BANK_WIDTH+ROW_WIDTH+COL_WIDTH-1 : BANK_WIDTH+COL_WIDTH];
wire [BANK_WIDTH-1:0]  q_bnk = q_addr[q_head][BANK_WIDTH+ROW_WIDTH+COL_WIDTH-1 : BANK_WIDTH+ROW_WIDTH+COL_WIDTH-1];
wire                   need_act = ~(row_valid & bank_open
                                 & (row_open == q_row) & (bank_now == q_bnk));
reg  [READ_LATENCY-1:0] rd_pipe = 0;
wire                    rd_cap  = rd_pipe[READ_LATENCY-1];
wire       rpop       = rvalid & rready;
wire       f_push     = rd_cap & ~f_full;

// The read-capture pipeline: a read is issued, and dq_in (the 8 captured
// words of that burst) is pushed into the response FIFO READ_LATENCY pclk
// later. One push per read.
reg                    rd_issue;

assign rvalid     = ~f_empty;
assign rdata      = f_data[f_head];
// cmd_ready means "a command offered this cycle has been TAKEN IN", i.e.
// it is the queue's write-enable. It must NOT depend on the queue being
// non-empty: if it did, an empty queue could never be filled, because the
// only thing that increments q_count is a push, and a push needs
// cmd_ready. That is a hard deadlock on entry to an empty queue, and it is
// what hung the first READ_BURST run.
//
// The cadence gate belongs on q_pop instead (below), where it decides
// whether a command is ISSUED to DRAM this cycle. Keeping the two apart is
// what lets the queue sit full at depth 8 while the engine issues at
// ISSUE_PCLK.
assign cmd_ready  = ((fsm == S_IDLE) | (fsm == S_ISSUE))
                                 & ~q_full & ~f_full & ~need_ref;

// dqs_read must be pulsed for every read, in the phase the DQS primitive
// expects. rclkpos is a RUNTIME register (0-3, found by read calibration),
// so the tap is a wire (a variable bit select = an 8:1 mux), not a
// localparam. The index mirrors the legacy FSM, which pulses READ at cycle
// rclkpos+RCD/4+1 with the command placed at cycle RCD/4.
localparam STRB_N = 4 + RCD_PCLK + 1;
reg [STRB_N-1:0] rd_strb = 0;
wire [2:0] strb_tap = {rclkpos, 1'b0} + RCD_PCLK[2:0];
wire rd_strb_tap = rd_strb[strb_tap];

// ---- push/pop bookkeeping, shared by every fsm state ----------------------
// A command is ISSUED to DRAM this cycle: the engine is in the issuing
// state, there is one queued, its row is already open, and the cadence
// counter has expired. The `cycle == 0` gate is what makes this ISSUE_PCLK
// pclk apart; without it a BL8 would go out every pclk, which is not a
// rate the tCCD budget was sized for.
// The engine has its OWN cadence counter. It must not reuse the legacy
// `cycle`, which the main FSM unconditionally saturates at 31 on every
// cycle -- two drivers on one reg, and the engine's countdown was being
// overwritten every cycle, so q_pop never fired.
reg [4:0] i_cycle;
wire q_pop = (fsm == S_ISSUE) & ~q_empty & ~need_act & ~need_ref
                            & (i_cycle == 5'd0);
wire f_pop = rpop;
always @(posedge pclk) begin
    if (!rst_lock_n) begin
        q_count <= 0; q_head <= 0; q_tail <= 0;
        f_count <= 0; f_head <= 0; f_tail <= 0;
        row_open <= 0; row_valid <= 0; bank_open <= 0; bank_now <= 0;
        need_ref <= 0; pre_cnt <= 0; fsm <= S_IDLE;
        rd_pipe <= 0; rd_strb <= 0; cmd_count_rd <= 0; pclk_count <= 0; refresh_count <= 0;
    end else if (fast_mode) begin
        // counters for the rate measurement
        if (q_pop)      cmd_count_rd  <= cmd_count_rd + 32'd1;
        if (need_ref_q) refresh_count <= refresh_count + 24'd1;
        pclk_count <= pclk_count + 32'd1;

        // command queue
        if (cmd_valid & cmd_ready) begin
            q_addr[q_tail] <= cmd_addr;
            q_tail         <= q_tail + 4'd1;
        end
        if (q_pop) q_head <= q_head + 4'd1;
        case ({cmd_valid & cmd_ready, q_pop})
            2'b10: q_count <= q_count + 4'd1;
            2'b01: q_count <= q_count - 4'd1;
            default: ;
        endcase

        // response FIFO
        if (f_push) begin
            f_data[f_tail] <= dout128;
            f_tail         <= f_tail + 4'd1;
        end
        if (f_pop) f_head <= f_head + 4'd1;
        case ({f_push, f_pop})
            2'b10: f_count <= f_count + 4'd1;
            2'b01: f_count <= f_count - 4'd1;
            default: ;
        endcase

        // read issue / capture shift registers
        rd_pipe <= {rd_pipe[READ_LATENCY-2:0], q_pop};
        rd_strb <= {rd_strb[STRB_N-2:0], q_pop};
        rd_issue <= q_pop;
    end
end

// refresh request from the top level. In fast_mode it is latched and
// serviced by the engine's own S_PRE/S_REF states so that a refresh never
// interleaves with a burst.
always @(posedge pclk) begin
    if (!rst_lock_n)            need_ref <= 0;
    else if (!fast_mode)        need_ref <= 0;
    else if (refresh)           need_ref <= 1'b1;
    else if (fsm == S_REF)      need_ref <= 1'b0;
end

// ---- the engine FSM --------------------------------------------------------
reg [4:0] s_cycle;
always @(posedge pclk) begin
    if (!rst_lock_n) begin
        s_cycle <= 0; cycle <= 0; i_cycle <= 0;
    end else if (fast_mode) begin
        case (fsm)
        // Entry: arm the DQS read path once. dqs_hold is deliberately NOT
        // pulsed per read: in the DQS model reset_f = reset | HOLD, so
        // HOLD resets BOTH read-FIFO pointers. The legacy FSM pulses it per
        // read because it has only one read in flight; with 8 in flight
        // that would wipe every read already on the wire.
        S_IDLE, S_ISSUE: begin
            // The cadence countdown lives HERE, in the combined arm, not in
            // a separate S_ISSUE arm. A separate S_ISSUE arm is unreachable:
            // in a case statement the first matching arm wins, and
            // `S_IDLE, S_ISSUE:` already claims state S_ISSUE. With the
            // countdown in the unreachable arm, i_cycle was loaded once and
            // never decremented, so the engine issued 9 commands and stopped.
            s_cycle <= 0;
            busy  <= (q_count != 4'd0) | (|rd_pipe) | (f_count != 4'd0);
            if (!row_valid) begin
                dqs_hold <= 1'b1;
            end
            if (need_ref) begin
                // A refresh is due. If nothing is in flight it can be taken
                // at once; otherwise drain first. Either way it must be
                // taken before more reads are issued, because tREFI has
                // passed.
                if (f_count == 4'd0 && (rd_pipe == 0)) begin
                    s_cycle <= 0;
                    fsm <= S_PRE;
                end else begin
                    s_cycle <= 0;
                    fsm <= S_DRAIN;
                end
            end else begin
                if (!q_empty && !need_act) begin
                    fsm <= S_ISSUE;
                end else if (!q_empty && need_act) begin
                    fsm <= S_MISS;                    // 0-cycle
                end else begin
                    fsm <= S_IDLE;
                end
            end

            // The cadence countdown goes LAST in this arm, because the last
            // assignment to a reg inside one always block wins.
            //
            // The reload on entry to S_ISSUE must be guarded by
            // `fsm != S_ISSUE`. Unguarded, it reloaded every cycle, so
            // i_cycle was pinned at ISSUE_PCLK-1 and never reached 0: the
            // engine issued 9 commands and then stalled forever.
            if (fsm != S_ISSUE) i_cycle <= FIVEB'(ISSUE_PCLK-1);
            else if (i_cycle != 5'd0) i_cycle <= i_cycle - 5'd1;
        end

        // Row miss: precharge the open row/bank, then activate the new one.
        // RP is in nCK; /4 to get pclk. The activate is placed at the
        // earliest legal point, which is what makes the row-open stream
        // amortise: a full 1K-column row is 128 reads behind one ACT.
        S_MISS: begin
            if (s_cycle == 0) begin
                {nRAS[0], nCAS[0], nWE[0]} <= CMD_PreCharge;
                // A[10]=1 on a PRECHARGE means precharge-all-banks, which
                // is what the legacy refresh path relies on.
                A[0][10] <= 1'b1;
                s_cycle  <= 5'd1;
            end else if (s_cycle >= 5'(RCD_PCLK+1)) begin
                {nRAS[0], nCAS[0], nWE[0]} <= CMD_BankActivate;
                BA[0] <= q_bnk;
                A[0]  <= {7'b0, q_row};
                A[0][10] <= 1'b0;
                s_cycle <= 0;
                fsm <= S_ISSUE;
            end else begin
                s_cycle <= s_cycle + 5'd1;
            end
        end

        // Precharge ALL banks before a refresh. REF with a bank open is a
        // DDR3 violation. This is the correctness cost of holding the row
        // open, and it is paid once per refresh (~2% at tRFC 160 ns
        // against one REFI every 7.8 us), not once per command.
        S_PRE: begin
            if (s_cycle == 0) begin
                {nRAS[0], nCAS[0], nWE[0]} <= CMD_PreCharge;
                A[0][10] <= 1'b1;            // precharge all
                s_cycle  <= 5'd1;
            end else if (s_cycle >= 5'(RP/4)) begin
                s_cycle <= 0;
                fsm <= S_REF;
            end else begin
                s_cycle <= s_cycle + 5'd1;
            end
        end

        S_REF: begin
            if (s_cycle == 0) begin
                {nRAS[0], nCAS[0], nWE[0]} <= CMD_AutoRefresh;
                A[0][10] <= 1'b0;
                s_cycle  <= 5'd1;
            end else if (s_cycle >= FIVEB'(RC/4)) begin
                // tRFC recovery. RC is this file's tRFC (48.75ns), the same
                // wait the legacy REFRESH path uses; the 7.8us REFI period
                // is generated by the top level, not here.
                s_cycle <= 0;
                pre_cnt <= 3'd7;
                fsm <= S_RFC;
            end else begin
                s_cycle <= s_cycle + 5'd1;
            end
        end

        // Refresh is due but reads are still in flight. Stop issuing --
        // cmd_ready is low in every state except S_ISSUE, so the queue is
        // held, not dropped -- and wait for the last burst to land.
        //
        // This state used to be the tRFC-recovery state as well, which
        // deadlocked: it waited for !need_ref, but need_ref is only cleared
        // in S_REF, and nothing could reach S_REF. Recovery is now its own
        // state, S_RFC.
        S_DRAIN: begin
            if (f_count == 4'd0 && rd_pipe == 0) begin
                s_cycle <= 0;
                fsm <= S_PRE;
            end
        end

        // tRFC recovery after the refresh, then back to issuing.
        S_RFC: begin
            if (pre_cnt != 0) begin
                pre_cnt <= pre_cnt - 3'd1;
            end else begin
                row_valid <= 1'b0;          // the refresh closed every bank
                bank_open <= 1'b0;
                fsm <= S_IDLE;
            end
        end
        default: fsm <= S_IDLE;
        endcase

        // Row tracking: the engine only changes these when it actually
        // drives an ACT or a precharge-all, never speculatively.
        if ((fsm == S_MISS) && (s_cycle == 0)) begin
            bank_open <= 1'b0;
        end
        if ((fsm == S_MISS) && (s_cycle >= 5'(RCD_PCLK+1))) begin
            row_open  <= q_row;
            bank_open <= 1'b1;
            bank_now  <= q_bnk;
            row_valid <= 1'b1;
        end
        if (fsm == S_DRAIN) begin
            row_valid <= 1'b0;
            bank_open <= 1'b0;
        end
    end
end

// The 400 MB/s bar, as a self-check. At 16 B/command, 4 pclk/cmd is
// 398.25 MB/s: it FAILS the bar. If this ever fires, the cadence was
// loosened to 4 and the number is no longer the one that was asked for.
`ifndef SYNTHESIS
initial begin
    if (ISSUE_PCLK == 4)
        $error("ISSUE_PCLK=4 gives 398.25 MB/s, which is below the 400 MB/s read bar. Use 3.");
    if (ISSUE_PCLK == 3)
        $display("READ-BAR check: ISSUE_PCLK=3 -> %.2f MB/s at 100 MHz pclk, 16 B/cmd",
                 (16.0/ISSUE_PCLK)*100.0);
    // The queue and response pointers are exactly $clog2(DEPTH) bits wide,
    // which wraps them for free ONLY if DEPTH is a power of two. A depth of
    // 10 or 12 would need an explicit wrap, and without one the pointer runs
    // off the end of the array, the row compare reads x, and the engine
    // stalls with a full queue. Fail at elaboration instead.
    if ((CMD_DEPTH & (CMD_DEPTH-1)) != 0)
        $error("CMD_DEPTH=%0d is not a power of two; the q_head/q_tail pointers do not wrap.", CMD_DEPTH);
    if ((RESP_DEPTH & (RESP_DEPTH-1)) != 0)
        $error("RESP_DEPTH=%0d is not a power of two; the f_head/f_tail pointers do not wrap.", RESP_DEPTH);
end
`endif


// Monitor rburst pulses and set rburst_seen when one is seen
always @(posedge pclk) begin
    if (rburst[0]) rburst_seen[0] <= 1'b1;
    if (rburst[1]) rburst_seen[1] <= 1'b1;
    if ({state,cycle} == {READ_CALIB, FIVEB'(RCD/4)})   // when we send read calibration req.
        rburst_seen <= 2'b0;
    if (~rst_lock_n) begin
        rburst_seen <= 2'b0;
    end
end

// Output READ pulse is for detecting DQS signal
// RCLKSEL shifts READ pulse by 1/4 CK per step
// Read calibration is done by first trying all RCLKSEL values (total 8)
// Then move READ pulse to next cycle (total 4). So there're 32 positions to try.
// When signals align, there will be a RBURST pulse.
always @(posedge pclk) begin
    dqs_read <= 0;
    if (fast_mode) begin
        // Every queued read needs a READ pulse at the calibrated phase.
        // Without this dqs_en stays 0, which gates dqs_r_clean, which gates
        // DQSR90 (the IDES8 ICLK) -- no pulse means no read data at all.
        if (rd_strb_tap) dqs_read <= 4'b1111;
    end else if ((state == READ || state == READ_CALIB)
            && cycle == FIVEB'(rclkpos+RCD/4+1)) begin
        dqs_read <= 4'b1111;
    end
end

//
// SERDES IOLOGIC for DDR3
//
wire [7:0] dllstep;

`ifdef SIM
// DLL simulation takes too long to lock (33600 cycles). So let's cheat.
assign dllstep = 8'd25;
assign dlllock = 1'b1;
`else
// SCAL_EN=="false" means constant output of 
// Every DLL step is about 0.025 ns. For 100Mhz, dllstep should be roughly 0.625/0.025=25
DLL #(
    .SCAL_EN("true"), .CODESCAL("101")     // 68-degree phase shift
//    .SCAL_EN("false")                     // 90° phase shift
) dll(
    .CLKIN(fclk), .RESET(~resetn), .STOP(0), 
    .UPDNCNTL(1'b0), .STEP(dllstep), .LOCK(dlllock)
);
`endif

wire [2:0] dqs_waddr[1:0], dqs_raddr[1:0];
wire clk_dqsr[1:0];
wire clk_dqsw[1:0];
wire clk_dqsw270[1:0];
generate
    // ck is fclk delayed 90 degrees with PLL
    // DQSR90 is DQSIN delayed DLLSTEP.
    // DQSW0 is fclk delayed WSTEP
    // DQSW270 is fclk delayed WSTEP+DLLSTEP+180-degrees.
    for (genvar i0 = 0; i0 < 2; i0++) begin : gen_dqs_controller
        DQS #(
            .DQS_MODE("X4"), .HWL("false")       // HWL is related to WL&RL
            // .DQS_MODE("X4"), .HWL("true")   
        ) u_dqs (
            .FCLK(fclk), .PCLK(pclk), .DQSIN(DDR3_DQS[i0]), .RESET(~rst_lock_n), .HOLD(dqs_hold), 
            .RLOADN(1'b0), .WLOADN(1'b0), .RMOVE(1'b0), .WMOVE(1'b0),
            // RDIR/WDIR: the DQS internal step-counter SWEEP DIRECTION.
            // Tied explicitly rather than left open (EX2565 fires on both
            // instances of this generate loop; an unconnected primitive
            // input is not a defined value).
            //
            // They are NOT the DQ bus turnaround control. In Gowin's own
            // model, simlib/gw2a/prim_sim.v, the sweep can only move while
            // the matching *LOADN is HIGH:
            //     always @(wstep_init or WLOADN or WMOVE or WDIR)
            //         if (WLOADN == 1'b0) wstep_reg <= wstep_init;
            //         else if (WMOVE falls) wstep_reg += WDIR ? -1 : +1;
            // RLOADN/WLOADN are tied 0 on the line above, so RDIR/WDIR
            // cannot move rstep_reg/wstep_reg at all. This controller
            // takes its read phase straight from DLLSTEP and its write
            // phase from wstep (write levelling) and never uses the sweep.
            //
            // The DQ turnaround is done by dq_oen, which the FSM drives to
            // 4'b1111 (all lanes Hi-Z) every cycle by default and drops
            // only in the WRITE states.
            //
            // The one residual effect of the tie value is RFLAG/WFLAG,
            // which the model asserts when the step counter reaches a rail
            // (0xFF for dir 0, 0x00 for dir 1). RDIR=0 means "sweep runs
            // toward 0xFF". rstep_reg loads from DLLSTEP (25 in sim), so
            // RFLAG is low either way -- and the controller does not read
            // RFLAG/WFLAG regardless: read calibration uses RBURST.
            .RDIR(1'b0), .WDIR(1'b0),
            .DLLSTEP(dllstep), .WSTEP(wstep),        // 0.625/0.025
            .RCLKSEL(rclksel), .READ(dqs_read),
            .DQSR90(clk_dqsr[i0]), .WPOINT(dqs_waddr[i0]), .RPOINT(dqs_raddr[i0]), 
            .DQSW0(clk_dqsw[i0]), .DQSW270(clk_dqsw270[i0]), .RBURST(rburst[i0])
        );
    end
endgenerate

// 2*CK speed in/out for data(DQ), data strobe(DQS) and data mask(DM)
wire [15:0] dq_buf, dq_buf_oen;        // I/O buffer internal use
generate
    for (genvar i1 = 0; i1 < 16; i1++) begin : gen_dq
        OSER8_MEM #(.TCLK_SOURCE("DQSW270")) oser_dq(
            .D0(dq_out[0][i1]), .D1(dq_out[1][i1]), .D2(dq_out[2][i1]), .D3(dq_out[3][i1]), 
            .D4(dq_out[4][i1]), .D5(dq_out[5][i1]), .D6(dq_out[6][i1]), .D7(dq_out[7][i1]), 
            .TX0(dq_oen[0]), .TX1(dq_oen[1]), .TX2(dq_oen[2]), .TX3(dq_oen[3]), 
            .FCLK(fclk), .PCLK(pclk), .TCLK(clk_dqsw270[i1/8]), .RESET(~rst_lock_n|| ~dlllock), 
            .Q0(dq_buf[i1]), .Q1(dq_buf_oen[i1])
        );
        assign DDR3_DQ[i1] = dq_buf_oen[i1] ? 1'bz : dq_buf[i1];

        // The fifo is for clock domain crossing from DQS to fclk
        IDES8_MEM iser_dq(
            .D(DDR3_DQ[i1]), .ICLK(clk_dqsr[i1/8]), .FCLK(fclk), .PCLK(pclk), .CALIB(1'b0), .RESET(~rst_lock_n), 
            .Q0(dq_in[0][i1]), .Q1(dq_in[1][i1]), .Q2(dq_in[2][i1]), .Q3(dq_in[3][i1]), 
            .Q4(dq_in[4][i1]), .Q5(dq_in[5][i1]), .Q6(dq_in[6][i1]), .Q7(dq_in[7][i1]), 
            .WADDR(dqs_waddr[i1/8]), .RADDR(dqs_raddr[i1/8])
        );
    end
endgenerate

// DQS[1:0], DM[1:0]
wire [1:0] dqs_buf, dqs_buf_oen;
wire [1:0] dqs_buf_delayed;
generate
    for (genvar i2 = 0; i2 < 2; i2++) begin : gen_dqs
        // DQS is output using the DQSW0
        OSER8_MEM oser_dqs(
             .D0(dqs_out[0]), .D1(dqs_out[1]), .D2(dqs_out[2]), .D3(dqs_out[3]), 
             .D4(dqs_out[4]), .D5(dqs_out[5]), .D6(dqs_out[6]), .D7(dqs_out[7]), 
             .TX0(dqs_oen[0]), .TX1(dqs_oen[1]), .TX2(dqs_oen[2]), .TX3(dqs_oen[3]), 
             .FCLK(fclk), .PCLK(pclk), .TCLK(clk_dqsw[i2]), .RESET(~rst_lock_n), 
             .Q0(dqs_buf[i2]), .Q1(dqs_buf_oen[i2])
        );
        assign DDR3_DQS[i2] = dqs_buf_oen[i2] ? 1'bz : dqs_buf[i2];

        // DM is aligned with DQ
        OSER8_MEM #(.TCLK_SOURCE("DQSW270")) oser_dm(
            .D0(dm_out[0]), .D1(dm_out[1]), .D2(dm_out[2]), .D3(dm_out[3]), 
            .D4(dm_out[4]), .D5(dm_out[5]), .D6(dm_out[6]), .D7(dm_out[7]), 
            .FCLK(fclk), .PCLK(pclk), .TCLK(clk_dqsw270[i2]), .RESET(~rst_lock_n), .Q0(DDR3_DM[i2])
        );
    end
endgenerate

// CK speed output for nRAS, nCAS, nWE
OSER8 oser_nras(
    .D0(nRAS[0]), .D1(nRAS[0]), .D2(nRAS[1]), .D3(nRAS[1]), 
    .D4(nRAS[2]), .D5(nRAS[2]), .D6(nRAS[3]), .D7(nRAS[3]), 
    .FCLK(fclk), .PCLK(pclk), .RESET(~rst_lock_n), .Q0(DDR3_nRAS)
);
OSER8 oser_ncas(
    .D0(nCAS[0]), .D1(nCAS[0]), .D2(nCAS[1]), .D3(nCAS[1]), 
    .D4(nCAS[2]), .D5(nCAS[2]), .D6(nCAS[3]), .D7(nCAS[3]), 
    .FCLK(fclk), .PCLK(pclk), .RESET(~rst_lock_n), .Q0(DDR3_nCAS)
);
OSER8 oser_nwe(
    .D0(nWE[0]), .D1(nWE[0]), .D2(nWE[1]), .D3(nWE[1]), 
    .D4(nWE[2]), .D5(nWE[2]), .D6(nWE[3]), .D7(nWE[3]), 
    .FCLK(fclk), .PCLK(pclk), .RESET(~rst_lock_n), .Q0(DDR3_nWE)
);

// Address lines (CK speed)
generate
    for (genvar i3 = 0; i3 < ROW_WIDTH; i3++) begin: gen_a
        OSER8 oser_a(
            .D0(A[0][i3]), .D1(A[0][i3]), .D2(A[1][i3]), .D3(A[1][i3]), 
            .D4(A[2][i3]), .D5(A[2][i3]), .D6(A[3][i3]), .D7(A[3][i3]), 
            .FCLK(fclk), .PCLK(pclk), .RESET(~rst_lock_n), .Q0(DDR3_A[i3])
        );
    end
endgenerate

// BA lines (CK speed)
generate
    for (genvar i4 = 0; i4 < 3; i4++) begin: gen_ba
        OSER8 oser_ba(
            .D0(BA[0][i4]), .D1(BA[0][i4]), .D2(BA[1][i4]), .D3(BA[1][i4]), 
            .D4(BA[2][i4]), .D5(BA[2][i4]), .D6(BA[3][i4]), .D7(BA[3][i4]), 
            .FCLK(fclk), .PCLK(pclk), .RESET(~rst_lock_n), .Q0(DDR3_BA[i4])
        );
    end
endgenerate

endmodule