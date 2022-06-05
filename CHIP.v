// Your code 

/* Version
    6/5 (4:30)
    Kayn modified:
        complete ALU.
        ALU add in this file.
        add some reg & wire for control.
        complete control signal.
        
    
*/

module CHIP(clk,
            rst_n,
            // For mem_D
            mem_wen_D,
            mem_addr_D,
            mem_wdata_D,
            mem_rdata_D,
            // For mem_I
            mem_addr_I,
            mem_rdata_I);

    input         clk, rst_n ;
    // For mem_D
    output        mem_wen_D  ;
    output [31:0] mem_addr_D ;
    output [31:0] mem_wdata_D;
    input  [31:0] mem_rdata_D;
    // For mem_I
    output [31:0] mem_addr_I ;
    input  [31:0] mem_rdata_I;
    
    //---------------------------------------//
    // Do not modify this part!!!            //
    // Exception: You may change wire to reg //
    reg    [31:0] PC          ;              //
    reg    [31:0] PC_nxt      ;              // WIL : I changed wire to reg for PC_nxt
    wire          regWrite    ;              //
    wire   [ 4:0] rs1, rs2, rd;              //
    wire   [31:0] rs1_data    ;              //
    wire   [31:0] rs2_data    ;              //
    wire   [31:0] rd_data     ;              //
    //---------------------------------------//


    // Todo: other wire/reg 

    // Not solved yet
    assign valid = (rs1_data|rs2_data|mem_rdata_I); 
    assign mem_addr_I = PC;


    wire   [6:0]   opcode;
    wire   [14:12] funct3;
    wire   [31:25] funct7;
    reg    [2:0]   ALU_mode;
    wire           ALU_ready;
    reg            Jal;
    reg            Jalr;
    reg            Branch;
    reg            MemRead;
    reg            MemToReg;
    reg    [2:0]   ALUOp;
    reg            MemWrite;
    reg            ALUSrc_1; // For auipc, in_A = pc
    reg            ALUSrc_2; // select in_B between imme. and rs2
    reg            RegWrite;

    //Destruct Instruction
    assign opcode = mem_rdata_I[ 6: 0];
    assign rd     = mem_rdata_I[11: 7];
    assign funct3 = mem_rdata_I[14:12];
    assign rs1    = mem_rdata_I[19:15];
    assign rs2    = mem_rdata_I[24:20];
    assign funct7 = mem_rdata_I[31:25];

    ALU BasicALU(        
        .clk(clk), 
        .rst_n(rst_n), 
        .valid(valid), 
        .ready(ALU_ready), 
        .mode(ALU_mode), 
        .in_A(rs1_data), 
        .in_B(rs2_data), 
        .out(rd_data),

        .ALU_zero()   //Unsolved : Connect it to something!
    );

    //---------------------------------------//
    // Do not modify this part!!!            //
    reg_file reg0(                           //
        .clk(clk),                           //
        .rst_n(rst_n),                       //
        .wen(regWrite),                      //
        .a1(rs1),                            //
        .a2(rs2),                            //
        .aw(rd),                             //
        .d(rd_data),                         //
        .q1(rs1_data),                       //
        .q2(rs2_data)                        //
    );                                       //
    //---------------------------------------//

    // Todo: any combinational/sequential circuit

       
    // control signal
        /* need to supply: 
            auipc(0010111), jal (1101111), jalr(1100111)
            beq  (1100011), lw  (0000011), sw  (0100011)
            addi (0010011), slti(0010011), add (0110011), sub(0110011)
            xor  (0110011), mul (0110011)
        */

        // ================== Kayn did it
    always @(*) begin
        ALUOp = opcode[4:3];
        Jal = 0;
        Jalr = 0;
        Branch = 0;
        MemRead = 0;
        MemToReg = 0;
        MemWrite = 0;
        ALUSrc_1 = 0;
        ALUSrc_2 = 0;
        RegWrite = 0;
        case(opcode)
            // R-type: add, sub, xor, mul
            7'b0110011 : RegWrite = 1;

            // I-type: addi, slti
            7'b0010011 : begin
                ALUSrc_2 = 1;
                RegWrite = 1;
            end
            // auipc
            7'b0010111 : begin
                ALUSrc_1 = 1;
                ALUSrc_2 = 1;
                RegWrite = 1;
            end

            // jal
            7'b1101111 : begin
                Jal = 1;
                RegWrite = 1;
            end

            // jalr
            7'b1100111 : begin
                Jalr = 1;
                RegWrite = 1;
            end

            // beq
            7'b1100011 : Branch = 1;

            // lw
            7'b0000011 : begin
                MemRead = 1;
                MemToReg = 1;
                RegWrite = 1;
                ALUSrc_2 = 1;
            end

            // sw
            7'b0100011 : begin
                MemWrite = 1;
                ALUSrc_2 = 1;
            end
        endcase
    end
    // ALU control
    always @(*) begin
        
    end
        //=================== end

    // PC control        
    always @(*) begin 
        if (rst_n) PC_nxt = PC + 32'h00000004;
    end  

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
        end
        else begin
            PC <= PC_nxt;

        end
    end


endmodule

module reg_file(clk, rst_n, wen, a1, a2, aw, d, q1, q2);

    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth

    input clk, rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] d;
    input [addr_width-1:0] a1, a2, aw;

    output [BITS-1:0] q1, q2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign q1 = mem[a1];
    assign q2 = mem[a2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (aw == i)) ? d : mem[i];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end
    end
endmodule


// =================== ALU =================== //
module ALU(
    clk,
    rst_n,
    valid,
    ready,
    mode,
    in_A,
    in_B,
    out,
    ALU_zero
);

    // Definition of ports
    input         clk, rst_n;
    input         valid;
    input  [2:0]  mode; // mode: 0: add, 1: sub, 2: mul, 3: div, 4: 
    output        ready;
    input  [31:0] in_A, in_B;
    output [31:0] out;
    output        ALU_zero;

    // Definition of states
    parameter IDLE = 3'd0;
    parameter ADD  = 3'd1;
    parameter SUB  = 3'd2;
    parameter MUL  = 3'd3;
    parameter DIV  = 3'd4;
    parameter XOR  = 3'd5;
    parameter OUT  = 3'd6;

    // Todo: Wire and reg if needed
    reg  [2:0] state, state_nxt;
    reg  [4:0] counter, counter_nxt;
    reg  signed [63:0] shreg, shreg_nxt;
    reg  signed [31:0] alu_in, alu_in_nxt;
    reg  signed [32:0] alu_out;

    // Todo: Instatiate any primitives if needed

    // Todo 5: Wire assignments
    assign ready = (state == OUT);
    assign out = shreg[31:0];
    assign ALU_zero = (shreg[31:0] == 0);

    // Combinational always block
    // Todo 1: Next-state logic of state machine
    always @(*) begin
        case(state) 
            IDLE: begin
                if (valid) begin
                    if (mode[2])
                        shreg_nxt = XOR;
                    else
                        if (mode[1]) 
                            if (mode[0]) state_nxt = DIV;
                            else         state_nxt = MUL;
                        else 
                            if (mode[0]) state_nxt = SUB;
                            else         state_nxt = ADD;
                end
                else                 state_nxt = IDLE;
            end
            ADD :     state_nxt = OUT;
            SUB :     state_nxt = OUT;
            MUL : begin
                if (counter == 5'b11111) state_nxt = OUT;
                else                     state_nxt = MUL;
            end
            DIV : begin
                if (counter == 5'b11111) state_nxt = OUT;
                else                     state_nxt = DIV;
            end
            XOR :     state_nxt = OUT;            
            OUT :     state_nxt = IDLE;
            default : state_nxt = state;
        endcase
    end

    // Todo 2: Counter
    always @(*) begin
        case(state) 
            IDLE :    counter_nxt = 5'b00000;
            MUL :     counter_nxt = (counter == 5'b11111)? 5'b00000: counter + 1'd1;
            DIV :     counter_nxt = (counter == 5'b11111)? 5'b00000: counter + 1'd1;
            OUT :     counter_nxt = 5'b00000;
            default : counter_nxt = counter;
        endcase
    end

    // ALU input
    always @(*) begin
        case(state)
            IDLE : begin
                if (valid) alu_in_nxt = in_B;
                else       alu_in_nxt = 0;
            end
            OUT : alu_in_nxt = 0;
            default: alu_in_nxt = alu_in;
        endcase
    end

    // Todo 3: ALU output
    always @(*) begin
        case(state)
            ADD : alu_out[31:0] = shreg[31:0] + alu_in;
            SUB : alu_out[31:0] = shreg[31:0] - alu_in;
            MUL : begin
                if (shreg[0]) alu_out = alu_in + shreg[63:32];
                else          alu_out = shreg[63:32];
            end
            DIV : begin
                if (shreg[62:31] >= alu_in) alu_out = shreg[62:31] - alu_in;
                else                        alu_out = shreg[62:31];
            end
            XOR : alu_out[31:0] = shreg[31:0] ^ alu_in;
            default : alu_out = 32'd0;
        endcase
    end
    
    // Todo 4: Shift register
    always @(*) begin
        case(state)
            IDLE : begin
                if (valid) begin
                    shreg_nxt[63:32] = 32'd0;
                    shreg_nxt[31:0]  = in_A;
                end
                else shreg_nxt = shreg; 
            end
            ADD : begin
                shreg_nxt[31:0]  = alu_out[31:0];
                shreg_nxt[63:32] = 32'd0;
            end
            SUB : begin
                shreg_nxt[31:0]  = alu_out[31:0];
                shreg_nxt[63:32] = 32'd0;
            end
            MUL : begin
                shreg_nxt = shreg >> 1;
                shreg_nxt[63:31] = alu_out[32:0];
            end
            DIV : begin
                if (shreg[62:31] >= alu_in) begin
                    shreg_nxt = shreg << 1;
                    shreg_nxt[63:32] = alu_out[31:0];
                    shreg_nxt[0] = 1;
                end
                else begin
                    shreg_nxt = shreg << 1;
                    shreg_nxt[63:32] = alu_out[31:0];
                    shreg_nxt[0] = 0;
                end
            end
            XOR : begin
                shreg_nxt[31:0]  = alu_out[31:0];
                shreg_nxt[63:32] = 32'd0;
            end

            default : shreg_nxt = shreg;
        endcase
    end

    // Todo: Sequential always block
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= IDLE;
            counter <= 5'd0;
            alu_in  <= 32'd0;
            shreg   <= 64'd0;
        end
        else begin
            state   <= state_nxt;
            counter <= counter_nxt;
            alu_in  <= alu_in_nxt;
            shreg   <= shreg_nxt;
        end
    end
endmodule

 