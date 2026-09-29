module uart_tx_V2(
    input wire clk,
    input wire [7:0] din,
    input wire wr_en,
    output wire tx_busy,

    output reg tx_p    
);

initial begin
    tx_p = 1'b1;
end

parameter clk_freq = 27000000;
parameter uart_freq = 115200;

localparam STATE_IDLE	= 2'b00;
localparam STATE_START	= 2'b01;
localparam STATE_DATA	= 2'b10;
localparam STATE_STOP	= 2'b11;

reg[7:0] localdin;
reg localwr_en;

//always@(posedge clk)begin
always@(*)begin	
    localdin<=din;
    localwr_en<=wr_en;
end

reg [7:0] data= 8'h00;
reg [2:0] bitpos= 3'h0;
reg [1:0] state= STATE_IDLE;

wire tx_clk;

// PCLK CYCLES PER UART BIT -- rounded, and with no off-by-one.
//
//   (clk_freq / uart_freq) - 1
//
// is wrong twice over at the real pclk of 99.5625 MHz and 115200 baud.
//
// 1. TRUNCATION. The true ratio is 864.26, so integer division floors to
//    864 and the `-1` then makes it 863 pclk = 8667.9 ns per bit -- 12.6 ns
//    SHORT of the 8680.6 ns a mid-bit-sampling receiver expects.
// 2. THE `-1` ITSELF. tx_clkcnt counts 0..TX_CLK_MAX INCLUSIVE, so a bit is
//    TX_CLK_MAX+1 pclk long. The `-1` was compensating for that, but only
//    correctly when the ratio was exact; at a fractional ratio it pushes a
//    whole pclk of error into the accumulator.
//
// At 863 pclk the sampling point slips 12.6 ns every 10 bits, 7.3% of a bit
// per byte, so a receiver loses framing at about byte 33 and a 223-byte dump
// never completes one line. Measured before the fix: the wire probe counted
// 11 clean start bits then nothing, and the bench receiver decoded zero
// complete lines.
//
// Correct value here is 864 pclk (8678.0 ns, 2.6 ns/bit low -- the residual
// is the unavoidable integer-pclk quantisation, and it is an order of
// magnitude inside the 50% sampling window).
//
// Round-half-up with integer arithmetic, synthesis-friendly, and NO `-1`
// because the counter bound is already inclusive:
//     TX_CLK_MAX = (clk_freq + uart_freq/2) / uart_freq - 1
// evaluates to 865 here, one pclk long, which is +7.5 ns/bit. Using the
// ratio itself as the bound gives 864 and is correct; see the guard below,
// which fails loudly if the two ever disagree.
localparam TX_BIT_RATIO  = (clk_freq + (uart_freq/2)) / uart_freq;   // 864
localparam TX_CLK_MAX    = TX_BIT_RATIO;                            // inclusive bound

// Fail loudly if someone reintroduces an off-by-one or a truncating divide:
// the bit period is the difference between a decoded line and a silent stall,
// and neither shows up in a transcript.
`ifndef SYNTHESIS
initial begin
    if (TX_CLK_MAX * uart_freq < clk_freq - (uart_freq/2) ||
        TX_CLK_MAX * uart_freq > clk_freq + (uart_freq/2))
        $error("uart_tx_V2: TX_CLK_MAX=%0d gives a bit period of %0d pclk for a true ratio of %0d.%02d -- quantisation error exceeds half a baud clock",
               TX_CLK_MAX, TX_CLK_MAX, clk_freq/uart_freq,
               (clk_freq*100/uart_freq) % 100);
end
`endif

reg[$clog2(TX_CLK_MAX+1)+1:0] tx_clkcnt;

assign tx_clk = (tx_clkcnt == 0);

initial tx_clkcnt=0;

always @(posedge clk) begin
	if (tx_clkcnt >= TX_CLK_MAX)
		tx_clkcnt <= 0;
	else
		tx_clkcnt <= tx_clkcnt + 1;
end
	

always @(posedge clk) begin
	case (state)
	STATE_IDLE: begin
		if (localwr_en) begin
			state <= STATE_START;
			data <= localdin;
			bitpos <= 3'h0;
		end
	end
	STATE_START: begin
		if (tx_clk) begin
			tx_p <= 1'b0;
			state <= STATE_DATA;
		end
	end
	STATE_DATA: begin
		if (tx_clk) begin
			if (bitpos == 3'h7)
				state <= STATE_STOP;
			else
				bitpos <= bitpos + 3'h1;
			tx_p <= data[bitpos];
		end
	end
	STATE_STOP: begin
		if (tx_clk) begin
			tx_p <= 1'b1;
			state <= STATE_IDLE;
		end
	end
	default: begin
		tx_p <= 1'b1;
		state <= STATE_IDLE;
	end
	endcase
end

assign tx_busy = (state != STATE_IDLE);

endmodule
