`timescale 10ns / 1ns

module mips_cpu(
	input  rst,
	input  clk,

	output reg [31:0] PC,
	input  [31:0] Instruction,

	output [31:0] Address,
	output MemWrite,
	output [31:0] Write_data,

	input  [31:0] Read_data,
	output MemRead,

	output reg [31:0] cycle_cnt,		//counter of total cycles
	output [31:0] inst_cnt,			//counter of total instructions
	output [31:0] br_cnt,			//counter of branch/jump instructions
	output [31:0] ld_cnt,			//counter of load instructions
	output [31:0] st_cnt,			//counter of store instructions
	output [31:0] user1_cnt,		//user defined counter (reserved)
	output reg [31:0] user2_cnt,
	output reg [31:0] user3_cnt
);

	// TODO: insert your code
	wire  ALUSrcA, RegWrite, PCSrc, PCWriteCond, PCWrite, IRWrite;
    wire [1:0] ALUSrcB, PCSource, RegDst, MemtoReg;
    wire [2:0] ALUop, ALUOp;
    wire [31:0] regwdata;
    
    
    wire AZero, AOverflow, ACarryOut;
    wire [31:0] rdata1, rdata2, ALUResult, SgnExt, BResult;

    wire [4:0] rs, rt;
    wire [5:0] func;
    wire [4:0] rd;
    wire [31:0] ALUA, ALUB, nPCtemp;
    (*mark_debug = "true"*) reg [31:0] MDR, A, B, ALUOut, IR;
    wire [31:0] PCnext;
    wire IsBEQ;
    wire Ze;
    reg [4:0] s;

    always @ (posedge clk)
    begin
    if (rst)
        begin
            PC = 32'd0;
            cycle_cnt = 0;
            user2_cnt = 0;
            user3_cnt = 0;
        end
    else
        begin
            if (IRWrite) IR = Instruction;
            MDR = Read_data;
            A = rdata1;
            B = rdata2;
            s = IR[10:6];
            ALUOut = ALUResult;
            if(PCWrite || PCSrc)
                PC = PCnext; 
            cycle_cnt = cycle_cnt + 1;
        end
    end
    
    
    assign rs = IR[25:21];
    assign rt = IR[20:16];
    assign SgnExt = {{16{IR[15]}},{IR[15:0]}};
    assign func = IR[5:0];
    assign Write_data = B;
    assign ALUA = ALUSrcA ? A : PC;
    assign Address = ALUOut;
    assign PCSrc =  PCWriteCond & Ze;
    assign Ze = IsBEQ ? AZero : ~AZero;
    assign IsBEQ = IR[31:26] == 6'b000100 ? 1 : 0;
    
    reg_file reg_file1(
        .clk(clk), 
        .rst(rst), 
        .raddr1(rs), 
        .raddr2(rt), 
        .waddr(rd), 
        .wen(RegWrite), 
        .wdata(regwdata), 
        .rdata1(rdata1), 
        .rdata2(rdata2));
    alu alu1(
        .A(ALUA), 
        .B(ALUB), 
        .ALUop(ALUop), 
        .Overflow(AOverflow), 
        .CarryOut(ACarryOut), 
        .Zero(AZero),
        .Result(ALUResult));
    control CPUcontrol(
        .rst(rst), 
        .clk(clk), 
        .op(IR[31:26]), 
        .func(IR[5:0]), 
        .RegDst(RegDst), 
        .ALUSrcA(ALUSrcA),
        .ALUSrcB(ALUSrcB), 
        .MemtoReg(MemtoReg), 
        .RegWrite(RegWrite), 
        .MemRead(MemRead), 
        .MemWrite(MemWrite), 
        .IRWrite(IRWrite), 
        .PCWrite(PCWrite), 
        .PCWriteCond(PCWriteCond),
        .PCSource(PCSource), 
        .ALUOp(ALUOp), 
        .inst(inst_cnt), 
        .br(br_cnt), 
        .ld(ld_cnt),
        .st(st_cnt),
        .addiu(user1_cnt));
    ALUcontrol Acontrol(
        .ALUOp(ALUOp), 
        .func(IR[5:0]), 
        .ALUop(ALUop));
    MUX4 ALUBMux(
        .op(ALUSrcB), 
        .B(B),
        .SgnExt(SgnExt), 
        .SgnExtLF(SgnExt << 2),
        .ALUB(ALUB));
    MUX3 PCSourceMux(
        .PCS1(ALUResult), 
        .PCS2(ALUOut),
        .PCS3({PC[31:28],IR[25:0],2'b00}),
        .op(PCSource), 
        .PC(PCnext));
    WaddrMUX WaddrMUX(
        .Instruction(IR[20:11]),
        .RegDst(RegDst),
        .waddr(rd));
    WdataMUX WdataMUX(
        .MDR(MDR), 
        .ALUOut(ALUOut), 
        .ALUOutLF(ALUOut << s),
        .MemtoReg(MemtoReg),
        .WriteData(regwdata));
 
    
endmodule

module control(
    input rst,
    input clk,
    input [5:0] op,
    input [5:0] func,
    output [1:0] RegDst,
    output ALUSrcA,
    output [1:0] ALUSrcB,
    output [1:0] MemtoReg,
    output RegWrite,
    output MemRead,
    output MemWrite,
    output PCWriteCond,
    output PCWrite,
    output IRWrite,
    output [1:0] PCSource,
    output [2:0] ALUOp,
    output reg [31:0] inst,
    output reg [31:0] br,
    output reg [31:0] st,
    output reg [31:0] ld,
    output reg [31:0] addiu
   );
   
   reg [20:0] StateC, StateN;
   
   assign RegDst[1] = StateC[12];
   assign RegDst[0] = StateC[7] || StateC[16];
   assign ALUSrcA = StateC[2] || StateC[6] || StateC[8] || StateC[9] || StateC[15] || StateC[17] || StateC[19];
   assign ALUSrcB[1] = StateC[1] || StateC[2] || StateC[9] || StateC[15] || StateC[17] || StateC[19];
   assign ALUSrcB[0] = StateC[0] || StateC[1] || StateC[11];
   assign MemtoReg[0] = StateC[4];
   assign MemtoReg[1] = StateC[16];
   assign RegWrite = StateC[4] || StateC[7] || StateC[10] || StateC[12] || StateC[16] || StateC[18] || StateC[20];
   assign MemRead = StateC[0] || StateC[3];
   assign MemWrite = StateC[5];
   assign PCWriteCond = StateC[8];
   assign PCWrite = StateC[0] || StateC[12] || StateC[13] || StateC[14];
   assign IRWrite = StateC[0];
   assign PCSource[1] = StateC[12] || StateC[13];
   assign PCSource[0] = StateC[8] || StateC[14];
   assign ALUOp[2] = StateC[17] || StateC[19];
   assign ALUOp[1] = StateC[6] || StateC[15];
   assign ALUOp[0] = StateC[8] || StateC[15] || StateC[19];

   
   parameter [20:0]
        S0 = 21'b0000000_0000_0000_0000_01,
        S1 = 21'b0000000_0000_0000_0000_10,
        S2 = 21'b0000000_0000_0000_0001_00,
        S3 = 21'b0000000_0000_0000_0010_00,
        S4 = 21'b0000000_0000_0000_0100_00,
        S5 = 21'b0000000_0000_0000_1000_00,
        S6 = 21'b0000000_0000_0001_0000_00,
        S7 = 21'b0000000_0000_0010_0000_00,
        S8 = 21'b0000000_0000_0100_0000_00,
        S9 = 21'b0000000_0000_1000_0000_00,
        S10 = 21'b000000_0000_1000_0000_000,
        S11 = 21'b000000_0001_0000_0000_000,
        S12 = 21'b000000_0010_0000_0000_000,
        S13 = 21'b000000_0100_0000_0000_000,
        S14 = 21'b000000_1000_0000_0000_000,
        S15 = 21'b000001_0000_0000_0000_000,
        S16 = 21'b000010_0000_0000_0000_000,
        S17 = 21'b000100_0000_0000_0000_000,
        S18 = 21'b001000_0000_0000_0000_000,
        S19 = 21'b010000_0000_0000_0000_000,
        S20 = 21'b100000_0000_0000_0000_000;

 
    always @(posedge clk or posedge rst)
    if(rst)
    begin
        StateC <= S0;
        inst <= 0;
        br <= 0;
        addiu <= 0;
        st <= 0;
        ld <= 0;
    end    
    else
    begin
        StateC <= StateN;
        case (StateC)
        S0: inst <= inst + 1;
        S8, S11, S13: br <= br + 1;
        S9: addiu <= addiu + 1;
        S3: ld <= ld + 1;
        S5: st <= st + 1;
        endcase
    end
    
    always @ (StateC)
    begin
        StateN = 21'b0;
        case(StateC)
        S0: StateN = S1;
        S1: begin
                //inst = inst + 1;
                case(op)
                6'b100011, 6'b101011: StateN = S2; //LW, SW
                6'b000000:            StateN = S6; //R-type       
                6'b000101, 6'b000100: begin 
                                          StateN = S8; //BNE, BEQ
                                   //       br = br + 1;
                                      end     
                6'b001001:            begin
                                          StateN = S9; //ADDIU
                                  //        addiu = addiu + 1;
                                      end        
                6'b000011:            begin
                                          StateN = S11; //JAL
                                  //        br = br + 1;
                                      end
                6'b000010:            begin 
                                          StateN = S13; //JUMP
                                  //        br = br + 1;
                                      end        
                6'b001111:            StateN = S15; //LUI
                6'b001010:            StateN = S17; //SLTI
                6'b001011:            StateN = S19; //SLTIU
                default:              StateN = S0;
                endcase
            end
        S2: begin
                case(op)
                6'b100011: begin
                                StateN = S3; //LW
                          //      ld = ld + 1;
                            end
                6'b101011: begin 
                                StateN = S5; //SW
                          //      st = st + 1;
                           end
                default:   StateN = S0;
                endcase
            end
        S3: StateN = S4;
        S4: StateN = S0;
        S5: StateN = S0;
        S6: begin
                case(func)
                6'b001000: StateN = S14;
                6'b000000: StateN = S16;
                default:   StateN = S7;
                endcase
            end
        S7: StateN = S0;
        S8: StateN = S0;
        S9: StateN = S10;
        S10: StateN = S0;
        S11: StateN = S12;
        S12: StateN = S0;
        S13: StateN = S0;
        S14: StateN = S0;
        S15: StateN = S10;
        S16: StateN = S0;
        S17: StateN = S18;
        S18: StateN = S0;
        S19: StateN = S20;
        S20: StateN = S0;
        default: StateN = S0;
        endcase
    end
   endmodule

module ALUcontrol(
    input [5:0] func,
    input [2:0] ALUOp,
    output [2:0] ALUop
    );
    wire IsSlt;
    wire IsOr;
    assign IsSlt = func[5] & ~func[4] & func[3] & ~func[2] & func[1] & ~func[0];
    assign IsOr = func[5] & ~func[4] & ~func[3] & func[2] & ~func[1] & func[0];
    assign ALUop[2] = (ALUOp[0] && ~ALUOp[1] && ~ALUOp[2]) || (ALUOp[1] && ~ALUOp[0] && IsSlt) || ALUOp[2];
    assign ALUop[1] = (~IsOr && ~ALUOp[2] && ALUOp[1] && ~ALUOp[0]) || 
                      (~ALUOp[2] && ~ALUOp[1] && ~ALUOp[0]) || 
                      (ALUOp[0] && ALUOp[1] && ~ALUOp[2]) || 
                      (ALUOp[2] && ~ALUOp[1] && ~ALUOp[0]) ||
                      (~ALUOp[2] && ~ALUOp[1] && ALUOp[0]);
    assign ALUop[0] = (IsSlt && ~ALUOp[2] && ALUOp[1] && ~ALUOp[0]) || 
                      (IsOr && ~ALUOp[2] && ALUOp[1] && ~ALUOp[0]) || 
                      (ALUOp[0] && ALUOp[1] && ~ALUOp[2]) || 
                      (ALUOp[2] && ~ALUOp[1] && ~ALUOp[0]);
endmodule

module MUX4(
    input [31:0] B, SgnExt, SgnExtLF,
    input [1:0] op,
    output [31:0] ALUB
    );
    wire [31:0] and1, and2, and3, and4, op1, op1x, op0, op0x, cons;
    assign op1 = {32{op[1]}};
    assign op1x = {32{~op[1]}};
    assign op0 = {32{op[0]}};
    assign op0x = {32{~op[0]}};
    assign cons = 32'd4;
    assign and1 = B & op1x & op0x;
    assign and2 = cons & op1x & op0;
    assign and3 = SgnExt & op1 & op0x;
    assign and4 = SgnExtLF & op1 & op0;
    assign ALUB = and1 | and2 | and3 | and4;
endmodule

module MUX3(
    input [31:0] PCS1, PCS2, PCS3,
    input [1:0] op,
    output [31:0] PC
    );
    wire [31:0] and1, and2, and3, op1, op1x, op0, op0x;
    assign op1 = {32{op[1]}};
    assign op1x = {32{~op[1]}};
    assign op0 = {32{op[0]}};
    assign op0x = {32{~op[0]}};
    assign and1 = PCS1 & op1x & op0x;
    assign and2 = PCS2 & op1x & op0;
    assign and3 = PCS3 & op1 & op0x;
    assign PC = and1 | and2 | and3;
endmodule

module WaddrMUX(
    input [9:0] Instruction,
    input [1:0] RegDst,
    output [4:0] waddr
    );
    wire [4:0] and1, and2, and3, op1, op1x, op0, op0x, addr1, addr2, addr3;
    assign op1 = {5{RegDst[1]}};
    assign op1x = {5{~RegDst[1]}};
    assign op0 = {5{RegDst[0]}};
    assign op0x = {5{~RegDst[0]}};
    assign addr1 = Instruction[9:5];
    assign addr2 = Instruction[4:0];
    assign addr3 = 5'b11111;
    assign and1 = addr1 & op1x & op0x;
    assign and2 = addr2 & op1x & op0;
    assign and3 = addr3 & op1 & op0x;
    assign waddr = and1 | and2 | and3;
endmodule

module WdataMUX(
    input [31:0] MDR, ALUOut, ALUOutLF,
    input [1:0] MemtoReg,
    output [31:0] WriteData
    );
    wire [31:0] and1, and2, and3, op1, op1x, op0, op0x;
    assign op1 = {32{MemtoReg[1]}};
    assign op1x = {32{~MemtoReg[1]}};
    assign op0 = {32{MemtoReg[0]}};
    assign op0x = {32{~MemtoReg[0]}};
    assign and1 = ALUOut & op1x & op0x;
    assign and2 = MDR & op1x & op0;
    assign and3 = ALUOutLF & op1 & op0x;
    assign WriteData = and1 | and2 | and3;
endmodule
	

