// Language: Verilog 2001

`timescale 1ns / 1ps

/*
 * Content Addressable Memory (block RAM based)
 */
module cam_bram #(
    // search data bus width
    parameter DATA_WIDTH = 64,
    // memory size in log2(words)
    parameter ADDR_WIDTH = 5,
    // width of data bus slices
    parameter SLICE_WIDTH = 9
)
(
    input  wire                     clk,
    input  wire                     rst,

    input  wire [ADDR_WIDTH-1:0]    write_addr,
    input  wire [DATA_WIDTH-1:0]    write_data,
    input  wire                     write_delete,
    input  wire                     write_enable,
    output wire                     write_busy,

    input  wire [DATA_WIDTH-1:0]    compare_data,
    output wire [2**ADDR_WIDTH-1:0] match_many,
    output wire [2**ADDR_WIDTH-1:0] match_single,
    output wire [ADDR_WIDTH-1:0]    match_addr,
    output wire                     match
);

// total number of slices (enough to cover DATA_WIDTH with address inputs)
localparam SLICE_COUNT = (DATA_WIDTH + SLICE_WIDTH - 1) / SLICE_WIDTH;
// depth of RAMs
localparam RAM_DEPTH = 2**ADDR_WIDTH;

localparam [2:0]
    STATE_INIT = 3'd0,
    STATE_IDLE = 3'd1,
    STATE_DELETE_1 = 3'd2,
    STATE_DELETE_2 = 3'd3,
    STATE_WRITE_1 = 3'd4,
    STATE_WRITE_2 = 3'd5;

reg [2:0] state_reg = STATE_INIT, state_next;

wire [SLICE_COUNT*SLICE_WIDTH-1:0] compare_data_padded = {{SLICE_COUNT*SLICE_WIDTH-DATA_WIDTH{1'b0}}, compare_data};
wire [SLICE_COUNT*SLICE_WIDTH-1:0] write_data_padded = {{SLICE_COUNT*SLICE_WIDTH-DATA_WIDTH{1'b0}}, write_data};

reg [SLICE_WIDTH-1:0] count_reg = {SLICE_WIDTH{1'b1}}, count_next;

reg [SLICE_COUNT*SLICE_WIDTH-1:0] ram_addr = {SLICE_COUNT*SLICE_WIDTH{1'b0}};
reg [RAM_DEPTH-1:0] set_bit;
reg [RAM_DEPTH-1:0] clear_bit;
reg wr_en;

reg [ADDR_WIDTH-1:0] write_addr_reg = {ADDR_WIDTH{1'b0}}, write_addr_next;
reg [SLICE_COUNT*SLICE_WIDTH-1:0] write_data_padded_reg = {SLICE_COUNT*SLICE_WIDTH{1'b0}}, write_data_padded_next;
reg write_delete_reg = 1'b0, write_delete_next;

reg write_busy_reg = 1'b1;

assign write_busy = write_busy_reg;

reg [RAM_DEPTH-1:0] match_raw_out[SLICE_COUNT-1:0];
reg [RAM_DEPTH-1:0] match_many_raw;

assign match_many = match_many_raw;

reg [DATA_WIDTH-1:0] erase_ram [RAM_DEPTH-1:0];
reg [DATA_WIDTH-1:0] erase_data = {DATA_WIDTH{1'b0}};
reg erase_ram_wr_en;

integer i;

initial begin
    for (i = 0; i < RAM_DEPTH; i = i + 1) begin
        erase_ram[i] = {SLICE_COUNT*SLICE_WIDTH{1'b0}};
    end
end

integer k;

always @* begin
    match_many_raw = {RAM_DEPTH{1'b1}};
    for (k = 0; k < SLICE_COUNT; k = k + 1) begin
        match_many_raw = match_many_raw & match_raw_out[k];
    end
end

priority_encoder #(
    .WIDTH(RAM_DEPTH),
    .LSB_PRIORITY("HIGH")
)
priority_encoder_inst (
    .input_unencoded(match_many_raw),
    .output_valid(match),
    .output_encoded(match_addr),
    .output_unencoded(match_single)
);

// BRAMs
genvar slice_ind;
generate
    for (slice_ind = 0; slice_ind < SLICE_COUNT; slice_ind = slice_ind + 1) begin : slice
        localparam W = slice_ind == SLICE_COUNT-1 ? DATA_WIDTH-SLICE_WIDTH*slice_ind : SLICE_WIDTH;

        wire [RAM_DEPTH-1:0] match_data;
        wire [RAM_DEPTH-1:0] ram_data;

        ram_dp #(
            .DATA_WIDTH(RAM_DEPTH),
            .ADDR_WIDTH(W)
        )
        ram_inst
        (
            .a_clk(clk),
            .a_we(1'b0),
            .a_addr(compare_data[SLICE_WIDTH * slice_ind +: W]),
            .a_din({RAM_DEPTH{1'b0}}),
            .a_dout(match_data),
            .b_clk(clk),
            .b_we(wr_en),
            .b_addr(ram_addr[SLICE_WIDTH * slice_ind +: W]),
            .b_din((ram_data & ~clear_bit) | set_bit),
            .b_dout(ram_data)
        );

        always @* begin
            match_raw_out[slice_ind] <= match_data;
        end
    end
endgenerate

// erase
always @(posedge clk) begin
    erase_data <= erase_ram[write_addr_next];
    if (erase_ram_wr_en) begin
        erase_data <= write_data_padded_reg;
        erase_ram[write_addr_next] <= write_data_padded_reg;
    end
end

// write
always @* begin
    state_next = STATE_IDLE;

    count_next = count_reg;
    ram_addr = erase_data;
    set_bit = {RAM_DEPTH{1'b0}};
    clear_bit = {RAM_DEPTH{1'b0}};
    wr_en = 1'b0;

    erase_ram_wr_en = 1'b0;

    write_addr_next = write_addr_reg;
    write_data_padded_next = write_data_padded_reg;
    write_delete_next = write_delete_reg;

    case (state_reg)
        STATE_INIT: begin
            // zero out RAMs
            ram_addr = {SLICE_COUNT{count_reg}} & {{SLICE_COUNT*SLICE_WIDTH-DATA_WIDTH{1'b0}}, {DATA_WIDTH{1'b1}}};
            set_bit = {RAM_DEPTH{1'b0}};
            clear_bit = {RAM_DEPTH{1'b1}};
            wr_en = 1'b1;

            if (count_reg == 0) begin
                state_next = STATE_IDLE;
            end else begin
                count_next = count_reg - 1;
                state_next = STATE_INIT;
            end
        end
        STATE_IDLE: begin
            // idle state
            write_addr_next = write_addr;
            write_data_padded_next = write_data_padded;
            write_delete_next = write_delete;

            if (write_enable) begin
                // wait for read from erase_ram
                state_next = STATE_DELETE_1;
            end else begin
                state_next = STATE_IDLE;
            end
        end
        STATE_DELETE_1: begin
            // wait for read
            state_next = STATE_DELETE_2;
        end
        STATE_DELETE_2: begin
            // clear bit and write back
            clear_bit = 1'b1 << write_addr;
            wr_en = 1'b1;
            if (write_delete_reg) begin
                state_next = STATE_IDLE;
            end else begin
                erase_ram_wr_en = 1'b1;
                state_next = STATE_WRITE_1;
            end
        end
        STATE_WRITE_1: begin
            // wait for read
            state_next = STATE_WRITE_2;
        end
        STATE_WRITE_2: begin
            // set bit and write back
            set_bit = 1'b1 << write_addr;
            wr_en = 1'b1;
            state_next = STATE_IDLE;
        end
    endcase
end

always @(posedge clk) begin
    if (rst) begin
        state_reg <= STATE_INIT;
        count_reg <= {SLICE_WIDTH{1'b1}};
        write_busy_reg <= 1'b1;
    end else begin
        state_reg <= state_next;
        count_reg <= count_next;
        write_busy_reg <= state_next != STATE_IDLE;
    end

    write_addr_reg <= write_addr_next;
    write_data_padded_reg <= write_data_padded_next;
    write_delete_reg <= write_delete_next;
end

endmodule

/*
 * Priority encoder module
 */
module priority_encoder #
(
    parameter WIDTH = 4,
    // LSB priority: "LOW", "HIGH"
    parameter LSB_PRIORITY = "LOW"
)
(
    input  wire [WIDTH-1:0]         input_unencoded,
    output wire                     output_valid,
    output wire [$clog2(WIDTH)-1:0] output_encoded,
    output wire [WIDTH-1:0]         output_unencoded
);

// power-of-two width
parameter W1 = 2**$clog2(WIDTH);
parameter W2 = W1/2;

generate
    if (WIDTH == 1) begin
        // one input
        assign output_valid = input_unencoded;
        assign output_encoded = 0;
    end else if (WIDTH == 2) begin
        // two inputs - just an OR gate
        assign output_valid = |input_unencoded;
        if (LSB_PRIORITY == "LOW") begin
            assign output_encoded = input_unencoded[1];
        end else begin
            assign output_encoded = ~input_unencoded[0];
        end
    end else begin
        // more than two inputs - split into two parts and recurse
        // also pad input to correct power-of-two width
        wire [$clog2(W2)-1:0] out1, out2;
        wire valid1, valid2;
        priority_encoder #(
            .WIDTH(W2),
            .LSB_PRIORITY(LSB_PRIORITY)
        )
        priority_encoder_inst1 (
            .input_unencoded(input_unencoded[W2-1:0]),
            .output_valid(valid1),
            .output_encoded(out1)
        );
        priority_encoder #(
            .WIDTH(W2),
            .LSB_PRIORITY(LSB_PRIORITY)
        )
        priority_encoder_inst2 (
            .input_unencoded({{W1-WIDTH{1'b0}}, input_unencoded[WIDTH-1:W2]}),
            .output_valid(valid2),
            .output_encoded(out2)
        );
        // multiplexer to select part
        assign output_valid = valid1 | valid2;
        if (LSB_PRIORITY == "LOW") begin
            assign output_encoded = valid2 ? {1'b1, out2} : {1'b0, out1};
        end else begin
            assign output_encoded = valid1 ? {1'b0, out1} : {1'b1, out2};
        end
    end
endgenerate

// unencoded output
assign output_unencoded = 1 << output_encoded;

endmodule

/*
 * Generic dual-port RAM
 */
module ram_dp #
(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 10
)
(
    // port A
    input  wire                    a_clk,
    input  wire                    a_we,
    input  wire [ADDR_WIDTH-1:0]   a_addr,
    input  wire [DATA_WIDTH-1:0]   a_din,
    output wire [DATA_WIDTH-1:0]   a_dout,

    // port B
    input  wire                    b_clk,
    input  wire                    b_we,
    input  wire [ADDR_WIDTH-1:0]   b_addr,
    input  wire [DATA_WIDTH-1:0]   b_din,
    output wire [DATA_WIDTH-1:0]   b_dout
);

reg [DATA_WIDTH-1:0] a_dout_reg = {DATA_WIDTH{1'b0}};

reg [DATA_WIDTH-1:0] b_dout_reg = {DATA_WIDTH{1'b0}};

// (* RAM_STYLE="BLOCK" *)
reg [DATA_WIDTH-1:0] mem[(2**ADDR_WIDTH)-1:0];

assign a_dout = a_dout_reg;

assign b_dout = b_dout_reg;

integer i, j;

initial begin
    // two nested loops for smaller number of iterations per loop
    // workaround for synthesizer complaints about large loop counts
    for (i = 0; i < 2**ADDR_WIDTH; i = i + 2**(ADDR_WIDTH/2)) begin
        for (j = i; j < i + 2**(ADDR_WIDTH/2); j = j + 1) begin
            mem[j] = 0;
        end
    end
end

// port A
always @(posedge a_clk) begin
    a_dout_reg <= mem[a_addr];
    if (a_we) begin
        mem[a_addr] <= a_din;
        a_dout_reg <= a_din;
    end
end

// port B
always @(posedge b_clk) begin
    b_dout_reg <= mem[b_addr];
    if (b_we) begin
        mem[b_addr] <= b_din;
        b_dout_reg <= b_din;
    end
end

endmodule
