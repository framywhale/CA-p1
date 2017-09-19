`timescale 10ns / 1ns


///////////////////////////////////////////////////////////////////////////////////////////////////////
//淇敼浜嗕笌data_sram淇″彿鏈夊叧鐨勯儴鍒嗭紝鏇存柊浜嗕富妯″潡鐨勮緭鍏ヨ緭鍑轰俊鍙凤紝骞跺皢鍘焟ips_cpu.v涓殑Control module鏇存柊锛�
//鍏朵綑閮ㄥ垎鏈綔鏀瑰姩
///////////////////////////////////////////////////////////////////////////////////////////////////////

module mycpu_top(
/*	input  resetn,
	input  clk,

	output reg [31:0] PC,
	input  [31:0] Instruction,

	output [31:0] data_sram_addr,
	output data_sram_wen,
	output [31:0] data_sram_wdata,

	input  [31:0] data_sram_rdata,
	output MemRead,
*/	
	
	input  clk,
    input  resetn,

    // instruction fetch port
    output             inst_sram_en,
    output      [ 3:0] inst_sram_wen,
    output      [31:0] inst_sram_addr, // PC 
    output      [31:0] inst_sram_wdata,
    input       [31:0] inst_sram_rdata,
    
    // data fetch port
    output             data_sram_en,
    output      [ 3:0] data_sram_wen,
    output      [31:0] data_sram_addr,
    output      [31:0] data_sram_wdata,
    input       [31:0] data_sram_rdata,

    // debug signal
	output reg [31:0] debug_wb_pc, 
	output     [ 3:0] debug_wb_rf_wen,
	output     [ 4:0] debug_wb_rf_wnum,
	output     [31:0] debug_wb_rf_wdata
);

	// TODO: insert your code
	wire  ALUSrcA, PCSrc, PCWriteCond, PCWrite, IRWrite;
    wire [1:0] ALUSrcB, PCSource, RegDst, MemtoReg;
    wire [2:0] ALUop, ALUOp;
    wire [31:0] regwdata;
    wire [3:0] RegWrite;
    
    wire AZero, AOverflow, ACarryOut;
    wire [31:0] rdata1, rdata2, ALUResult, SgnExt;

    wire rst;

    wire [4:0] rs, rt;
    wire [5:0] func;
    wire [4:0] rd;
    wire [31:0] ALUA, ALUB;
    reg [31:0] /*MDR,*/ A, B, ALUOut, IR;
    wire [31:0] PCnext;
    wire IsBEQ;
    wire Ze;
    reg [4:0] s;
    wire PCdebug;
    reg [31:0] PCdb;
    reg [31:0] PC;
    
    always @ (posedge clk)
    begin
    if (~resetn)
        begin
            PC <= 32'hbfc00000;
        end
    else
        begin
            if (IRWrite) IR <= inst_sram_rdata;
			if (PCdebug) debug_wb_pc <= PC;           //wbpc
			
 //           MDR <= data_sram_rdata;                               //淇敼淇″彿鍚嶇О
			
            A <= rdata1;
            B <= rdata2;
            s <= IR[10:6];
            ALUOut <= ALUResult;
            if(PCWrite || PCSrc)
                PC <= PCnext;
//            cycle_cnt = cycle_cnt + 1;
        end
    end
/* 
always @(posedge clk) begin
    if (|data_sram_wen) $display ("sw: %h %h", data_sram_addr, data_sram_wdata);
    if (data_sram_en && ~|data_sram_wen) $display ("lw: %h %h", data_sram_addr, data_sram_rdata);
    end 
 */
    assign rst = ~resetn;
    
    assign rs = IR[25:21];
    assign rt = IR[20:16];
    assign SgnExt = {{16{IR[15]}},{IR[15:0]}};
    assign func = IR[5:0];
	
    assign data_sram_wdata = B;                                  //淇敼淇″彿鍚嶇О             
    
	assign ALUA = ALUSrcA ? A : PC;
    
//	assign data_sram_addr = ALUOut;                              //淇敼淇″彿鍚嶇О
    assign data_sram_addr = ALUResult;    
    
	assign PCSrc =  PCWriteCond & Ze;
    assign Ze = IsBEQ ? AZero : ~AZero;
    assign IsBEQ = IR[31:26] == 6'b000100 ? 1 : 0;
	
	assign data_sram_en = (data_sram_wen != 4'b0000) || MemRead;  //(姝よ鏄柊鍔犲叆鐨�)鍘烳emWrite鎴朚enRead淇″彿婵�鍙戞淇″彿
    
//    assign inst_sram_en = 1;
    assign inst_sram_en = PCWrite || PCSrc;
    
    assign inst_sram_addr = ~resetn ? PC : PCnext;
    
    assign inst_sram_wen = 4'b0000;
    assign inst_sram_wdata = 32'd0;
    
    assign debug_wb_rf_wen = RegWrite;
    assign debug_wb_rf_wnum = rd;
    assign debug_wb_rf_wdata = regwdata;
    
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
        .MemRead(MemRead),                                 //MemRead娌℃湁淇敼
        .data_sram_wen(data_sram_wen),                     //淇敼淇″彿鍚嶇О
        .IRWrite(IRWrite), 
        .PCWrite(PCWrite), 
        .PCWriteCond(PCWriteCond),
        .PCSource(PCSource), 
        .ALUOp(ALUOp), 
        .PCdebug(PCdebug)
        );
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
        .MDR(data_sram_rdata), 
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
    output [3:0] RegWrite,
    output MemRead,                                        //姝よ骞舵湭淇敼
	
    output [3:0] data_sram_wen,                            //淇敼淇″彿鍚嶇О锛屾墿灞曚负4浣�
    
	output PCWriteCond,
    output PCWrite,
    output IRWrite,
    output [1:0] PCSource,
    output [2:0] ALUOp,
    output PCdebug
   );
   
   reg [20:0] StateC, StateN;
   
   assign PCdebug = StateC[0];
   assign RegDst[1]   = StateC[12];
   assign RegDst[0]   = StateC[7]  || StateC[16];
   assign ALUSrcA     = StateC[2]  || StateC[3]  || StateC[5]   || StateC[6]  || StateC[8]  || StateC[9]  || StateC[15] || StateC[17] || StateC[19];
   assign ALUSrcB[1]  = StateC[1]  || StateC[2]  || StateC[3]   || StateC[5]  || StateC[9]  || StateC[15] || StateC[17] || StateC[19];
   assign ALUSrcB[0]  = StateC[0]  || StateC[1]  || StateC[11];
   assign MemtoReg[0] = StateC[4];
   assign MemtoReg[1] = StateC[16];
   assign RegWrite    = {4{StateC[4]  || StateC[7]  || StateC[10] || StateC[12] || StateC[16] || StateC[18] || StateC[20]}};
   assign MemRead     = StateC[0]  || StateC[3];		  //姝よ骞舵湭淇敼
   
   assign data_sram_wen    = {4{StateC[5]}};              //淇敼淇″彿鍚嶇О锛屾墿灞曚负4浣�
   
   assign PCWriteCond = StateC[8];
   assign PCWrite     = StateC[0]  || StateC[12] || StateC[13] || StateC[14];
   assign IRWrite     = StateC[0];
   assign PCSource[1] = StateC[12] || StateC[13];
   assign PCSource[0] = StateC[8]  || StateC[14];
   assign ALUOp[2]    = StateC[17] || StateC[19];
   assign ALUOp[1]    = StateC[6]  || StateC[15];
   assign ALUOp[0]    = StateC[8]  || StateC[15] || StateC[19];


   
   parameter [20:0]
        InstFetch  = 21'b0000000_0000_0000_0000_01, //S0    鐢变簬涓婇潰鐨勪俊鍙烽渶瑕佺敱State鍙橀噺涓殑瀵瑰簲浣嶆潵鎺у埗锛屾晠淇濈暀S0~S20鐨勬敞閲�
        InstAnal   = 21'b0000000_0000_0000_0000_10, //S1
        LWSW       = 21'b0000000_0000_0000_0001_00, //S2
        LW         = 21'b0000000_0000_0000_0010_00, //S3
        LW_WB      = 21'b0000000_0000_0000_0100_00, //S4
        SW         = 21'b0000000_0000_0000_1000_00, //S5
        R_TYPE     = 21'b0000000_0000_0001_0000_00, //S6
        R_WB       = 21'b0000000_0000_0010_0000_00, //S7
        BRANCH     = 21'b0000000_0000_0100_0000_00, //S8
        ADDIU      = 21'b0000000_0000_1000_0000_00, //S9
        ADDIU_WB   = 21'b000000_0000_1000_0000_000, //S10
        JAL        = 21'b000000_0001_0000_0000_000, //S11
        JAL_PCW    = 21'b000000_0010_0000_0000_000, //S12
        JUMP       = 21'b000000_0100_0000_0000_000, //S13
        JR         = 21'b000000_1000_0000_0000_000, //S14
        LUI        = 21'b000001_0000_0000_0000_000, //S15
        SLL	       = 21'b000010_0000_0000_0000_000, //S16
        SLTI       = 21'b000100_0000_0000_0000_000, //S17
        SLTI_WB    = 21'b001000_0000_0000_0000_000, //S18
        SLTIU      = 21'b010000_0000_0000_0000_000, //S19
        SLTIU_WB   = 21'b100000_0000_0000_0000_000; //S20

    
    always @(posedge clk)
    if(rst)
    begin
        StateC <= InstFetch;
//      inst <= 0;
//      br <= 0;
//      addiu <= 0;
//      st <= 0;
//      ld <= 0;
    end    
    else
    begin
        StateC <= StateN;
/*      case (StateC)
        InstFetch:         inst <= inst + 1;
        BRANCH, JAL, JUMP: br <= br + 1;
        ADDIU:             addiu <= addiu + 1;
        LW:                ld <= ld + 1;
        SW:                st <= st + 1;       
        endcase                     */
    end
    
    always @ (StateC)
    begin
        StateN = InstFetch;
        case(StateC)
        InstFetch: StateN = InstAnal;
        InstAnal: begin
                case(op)
                6'b100011, 6'b101011:     StateN = LWSW; //LW, SW
                6'b000000:                StateN = R_TYPE; //R-type       
                6'b000101, 6'b000100: begin 
                                          StateN = BRANCH; //BNE, BEQ
                                      end     
                6'b001001:            begin
                                          StateN = ADDIU; //ADDIU
                                      end        
                6'b000011:            begin
                                          StateN = JAL; //JAL
                                      end
                6'b000010:            begin 
                                          StateN = JUMP; //JUMP
                                      end        
                6'b001111:                StateN = LUI; //LUI
                6'b001010:                StateN = SLTI; //SLTI
                6'b001011:                StateN = SLTIU; //SLTIU
                default:                  StateN = InstFetch;
                endcase
            end
        LWSW: begin
                case(op)
                6'b100011: begin
                               StateN = LW; //LW
                           end
                6'b101011: begin 
                               StateN = SW; //SW
                           end
                default:       StateN = InstFetch;
                endcase
              end
        LW:       StateN = LW_WB;
        LW_WB:    StateN = InstFetch;
        SW:       StateN = InstFetch;
        R_TYPE: begin
                case(func)
                6'b001000: StateN = JR;
                6'b000000: StateN = SLL;
                default:   StateN = R_WB;
                endcase
            end
        R_WB:     StateN = InstFetch;
        BRANCH:   StateN = InstFetch;
        ADDIU:    StateN = ADDIU_WB;
        ADDIU_WB: StateN = InstFetch;
        JAL:      StateN = JAL_PCW;
        JAL_PCW:  StateN = InstFetch;
        JUMP:     StateN = InstFetch;
        JR:       StateN = InstFetch;
        LUI:      StateN = ADDIU_WB;
        SLL:      StateN = InstFetch;
        SLTI:     StateN = SLTI_WB;
        SLTI_WB:  StateN = InstFetch;
        SLTIU:    StateN = SLTIU_WB;
        SLTIU_WB: StateN = InstFetch;
        default:  StateN = InstFetch;
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
    assign ALUop[2] = (ALUOp[0] && ~ALUOp[1] && ~ALUOp[2]) || 
					  (ALUOp[1] && ~ALUOp[0] && IsSlt) || 
					   ALUOp[2];
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
	

