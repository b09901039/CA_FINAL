// Your code
 
module ALU(
    clk,
    rst_n,
    valid,
    ready,

    opcode,
    funct3,
    funct7,
    in_A, 
    in_B,

    out
);

    // Definition of ports
    input         clk, rst_n;
    input         valid;
    output        ready; 

    input  [6:0]   opcode;
    input  [14:12] funct3;
    input  [31:25] funct7;
    input  [31:0] in_A, in_B;

    output [31:0] out;

    // Definition of states
    parameter IDLE = 6'd0;
    parameter ADD  = 6'd1;
    parameter SUB  = 6'd2;
    parameter ADDI = 6'd3;
    parameter XOR  = 6'd4;
     

    parameter OUT  = 6'd63;

    // Todo: Wire and reg if needed
    reg  [ 2:0] state, state_nxt; 
    reg  [31:0] shreg, shreg_nxt;
    reg  [31:0] alu_in, alu_in_nxt;
    reg  [31:0] alu_out;

 
    // Todo: Instatiate any primitives if needed    

    // Todo 5: Wire assignments      
    assign out = shreg;   
    assign ready = (state==OUT);

    wire [11:0] imm;
    assign      imm = {funct7 , in_B};
    
    // Combinational always block
    // Todo 1: Next-state logic of state machine
    always @(*) begin

        if (state == IDLE)begin
            if (valid == 1)begin
                if(opcode == 7'b0110011 && 
                    funct3 == 3'b0 && funct7 == 7'b0)
                    state_nxt = ADD;
                if(opcode == 7'b0110011 && 
                    funct3 == 3'b0 && funct7 == 7'b0100000 )
                    state_nxt = SUB;

                if(opcode == 7'b0010011 && 
                    funct3 == 3'b0 )
                    state_nxt = ADDI;

                if(opcode == 7'b0110011 && 
                    funct3 == 3'b100 && funct7 == 7'b0)
                    state_nxt = XOR;
                 
            end
            else state_nxt = IDLE;
             
        end
        else begin
            state_nxt = OUT; 
        end         
    end
     

    // ALU input
    always @(*) begin
        case(state)
            IDLE: begin
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
            //MUL : begin 
            ADD : begin              
                alu_out = alu_in[31:0] + shreg[31:0]; 
            end
            SUB : begin              
                alu_out = alu_in[31:0] - shreg[31:0]; 
            end
            ADDI: begin
                alu_out = alu_in[31:0] + imm; 
            end
            XOR : begin
                alu_out = alu_in[31:0] ^ shreg[31:0]; 
            end
                   
            default :
                alu_out = 32'd0;   
        endcase
    end
    // Todo 4: Shift register
    always @(*) begin
        
        if (state == IDLE) 
            shreg_nxt = (valid == 1)?in_A[31:0]:32'd0;        
        else 
            shreg_nxt = shreg;/*
        shreg_nxt = ( 
            (state == IDLE)                              ?
            ((valid == 1) ? {32'd0, in_A[31:0]} : 64'd0) :
            shreg
        );   */
    end

    // Todo: Sequential always block
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE; 
            shreg <= 0; 
            alu_in <= 0;
        end
        else begin
            alu_in <= alu_in_nxt;
            state <= state_nxt;
            shreg <= shreg_nxt; 
        end
    end

endmodule
     