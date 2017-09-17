//ALUOp仍为3位，ALUSrc未改动


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
   
   assign RegDst[1]   = StateC[12];
   assign RegDst[0]   = StateC[7]  || StateC[16];
   assign ALUSrcA     = StateC[2]  || StateC[6]  || StateC[8]  || StateC[9]  || StateC[15] || StateC[17] || StateC[19];
   assign ALUSrcB[1]  = StateC[1]  || StateC[2]  || StateC[9]  || StateC[15] || StateC[17] || StateC[19];
   assign ALUSrcB[0]  = StateC[0]  || StateC[1]  || StateC[11];
   assign MemtoReg[0] = StateC[4];
   assign MemtoReg[1] = StateC[16];
   assign RegWrite    = StateC[4]  || StateC[7]  || StateC[10] || StateC[12] || StateC[16] || StateC[18] || StateC[20];
   assign MemRead     = StateC[0]  || StateC[3];
   assign MemWrite    = StateC[5];
   assign PCWriteCond = StateC[8];
   assign PCWrite     = StateC[0]  || StateC[12] || StateC[13] || StateC[14];
   assign IRWrite     = StateC[0];
   assign PCSource[1] = StateC[12] || StateC[13];
   assign PCSource[0] = StateC[8]  || StateC[14];
   assign ALUOp[2]    = StateC[17] || StateC[19];
   assign ALUOp[1]    = StateC[6]  || StateC[15];
   assign ALUOp[0]    = StateC[8]  || StateC[15] || StateC[19];

   
   parameter [20:0]
        InstFetch  = 21'b0000000_0000_0000_0000_01, //S0    由于上面的信号需要由State变量中的对应位来控制，故保留S0~S20的注释
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

 
    always @(posedge clk or posedge rst)
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
        SW:                st <= st + 1;       */
        endcase
    end
    
    always @ (StateC)
    begin
        StateN = InstFetch;
        case(StateC)
        InstFetch: StateN = InstAnal;
        InstAnal: begin
                case(op)
                6'b100011, 6'b101011:     StateN = LWSW; //LW, SW
                6'b000000:                StateN = R_Type; //R-type       
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
                default:   StateN = S7;
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