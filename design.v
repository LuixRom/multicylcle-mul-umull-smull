//Nivel 0: une todo, mem, arm
module top (
	clk,
	reset,
	WriteData,
	Adr,
	MemWrite
);
	input wire clk;
	input wire reset;
	output wire [31:0] WriteData;
	output wire [31:0] Adr;
	output wire MemWrite;
	wire [31:0] PC;
	wire [31:0] Instr;
	wire [31:0] ReadData;
	// instantiate processor and shared memory
	arm arm(
		.clk(clk),
		.reset(reset),
		.MemWrite(MemWrite),
		.Adr(Adr),
		.WriteData(WriteData),
		.ReadData(ReadData)
	);
	mem mem(
		.clk(clk),
		.we(MemWrite),
		.a(Adr),
		.wd(WriteData),
		.rd(ReadData)
	);
endmodule

//Nivel 2: Memoria 
module mem (
	clk,
	we,
	a,
	wd,
	rd
);
	input wire clk;
	input wire we;
	input wire [31:0] a;
	input wire [31:0] wd;
	output wire [31:0] rd;
	reg [31:0] RAM [63:0];
	initial $readmemh("memfile.dat", RAM);
	assign rd = RAM[a[31:2]]; // word aligned
	always @(posedge clk)
		if (we)
			RAM[a[31:2]] <= wd;
endmodule

//Nivel 2: Une el controller y el datapath
module arm (
	clk,
	reset,
	MemWrite,
	Adr,
	WriteData,
	ReadData
);
	input wire clk;
	input wire reset;
	output wire MemWrite;
	output wire [31:0] Adr;
	output wire [31:0] WriteData;
	input wire [31:0] ReadData;
	wire [31:0] Instr;
	wire [3:0] ALUFlags;
	wire PCWrite;
  	wire [1:0] RegWrite;
	wire IRWrite;
	wire AdrSrc;
	wire [1:0] RegSrc;
	wire [1:0] ALUSrcA;
	wire [1:0] ALUSrcB;
	wire [1:0] ImmSrc;
  	wire [2:0] ALUControl;
	wire [1:0] ResultSrc;
  	wire lmulFlag;
  	wire FpuWrite;
  	wire RegSrcMul;
  	wire mullargo;
  
	controller c(
		.clk(clk),
		.reset(reset),
		.Instr(Instr),
		.ALUFlags(ALUFlags),
		.PCWrite(PCWrite),
		.MemWrite(MemWrite),
		.RegWrite(RegWrite),
		.IRWrite(IRWrite),
		.AdrSrc(AdrSrc),
		.RegSrc(RegSrc),
		.ALUSrcA(ALUSrcA),
		.ALUSrcB(ALUSrcB),
		.ResultSrc(ResultSrc),
		.ImmSrc(ImmSrc),
        .ALUControl(ALUControl),
     	.lmulFlag(lmulFlag),
      	.FpuWrite(FpuWrite),
      	.RegSrcMul(RegSrcMul),
      	.mullargo(mullargo)
	);
  
	datapath dp(
		.clk(clk),
		.reset(reset),
		.Adr(Adr),
		.WriteData(WriteData),
		.ReadData(ReadData),
		.Instr(Instr),
		.ALUFlags(ALUFlags),
		.PCWrite(PCWrite),
      	.RegWrite(RegWrite),
		.IRWrite(IRWrite),
		.AdrSrc(AdrSrc),
		.RegSrc(RegSrc),
		.ALUSrcA(ALUSrcA),
		.ALUSrcB(ALUSrcB),
		.ResultSrc(ResultSrc),
		.ImmSrc(ImmSrc),
      	.ALUControl(ALUControl),
      	.lmulFlag(lmulFlag),
      	.FpuWrite(FpuWrite),
      	.RegSrcMul(RegSrcMul),
      	.mullargo(mullargo)
	);
endmodule

//Contiene la lógica condlogic, decode
module controller (
	clk,
	reset,
	Instr,
	ALUFlags,
	PCWrite,
	MemWrite,
	RegWrite,
	IRWrite,
	AdrSrc,
	RegSrc,
	ALUSrcA,
	ALUSrcB,
	ResultSrc,
	ImmSrc,
	ALUControl,
  	lmulFlag,
  	FpuWrite,
  	RegSrcMul,
  	mullargo
);
	input wire clk;
	input wire reset;
  	input wire [31:0] Instr;
	input wire [3:0] ALUFlags;
	output wire PCWrite;
	output wire MemWrite;
	output wire RegWrite;
	output wire IRWrite;
	output wire AdrSrc;
	output wire [1:0] RegSrc;
	output wire [1:0] ALUSrcA;
	output wire [1:0] ALUSrcB;
	output wire [1:0] ResultSrc;
	output wire [1:0] ImmSrc;
  	output wire [2:0] ALUControl;
  	output wire lmulFlag;
  	output wire FpuWrite;
  	output wire RegSrcMul;
  	output reg mullargo;
  
	wire [1:0] FlagW;
	wire PCS;
	wire NextPC;
	wire RegW;
  	wire RegW2;
	wire MemW;
  	wire FpuW;
  
  
	decode dec(
		.clk(clk),
		.reset(reset),
		.Op(Instr[27:26]),
      	.Mop(Instr[7:4]),	
		.Funct(Instr[25:20]),
		.Rd(Instr[15:12]),
		.FlagW(FlagW),
		.PCS(PCS),
		.NextPC(NextPC),
		.RegW(RegW),
      	.RegW2(RegW2),
		.MemW(MemW),
		.IRWrite(IRWrite),
		.AdrSrc(AdrSrc),
		.ResultSrc(ResultSrc),
		.ALUSrcA(ALUSrcA),
		.ALUSrcB(ALUSrcB),
		.ImmSrc(ImmSrc),
		.RegSrc(RegSrc),
      	.ALUControl(ALUControl),
      	.lmulFlag(lmulFlag),
      	.FpuW(FpuW),
      	.RegSrcMul(RegSrcMul),
      	.mullargo(mullargo)
	);
	condlogic cl(
		.clk(clk),
		.reset(reset),
		.Cond(Instr[31:28]),
		.ALUFlags(ALUFlags),
		.FlagW(FlagW),
		.PCS(PCS),
		.NextPC(NextPC),
		.RegW(RegW),
      	.RegW2(RegW2),
		.MemW(MemW),
      	.FpuW(FpuW),
		.PCWrite(PCWrite),
		.RegWrite(RegWrite),
      	.MemWrite(MemWrite),
      	.FpuWrite(FpuWrite)
	);
endmodule

module decode (
	clk,
	reset,
	Op,
  	Mop,
	Funct,
	Rd,
	FlagW,
	PCS,
	NextPC,
	RegW,
  	RegW2,
	MemW,
	IRWrite,
	AdrSrc,
	ResultSrc,
	ALUSrcA,
	ALUSrcB,
	ImmSrc,
	RegSrc,
	ALUControl,
  	lmulFlag,
  	FpuW,
  	RegSrcMul,
  	mullargo
);
	input wire clk;
	input wire reset;
	input wire [1:0] Op;
	input wire [5:0] Funct;
  	input wire [3:0] Mop;
	input wire [3:0] Rd;
	output reg [1:0] FlagW;
	output wire PCS;
	output wire NextPC;
	output wire RegW;
  	output wire RegW2;
	output wire MemW;
	output wire IRWrite;
	output wire AdrSrc;
	output wire [1:0] ResultSrc;
	output wire [1:0] ALUSrcA;
	output wire [1:0] ALUSrcB;
	output wire [1:0] ImmSrc;
	output wire [1:0] RegSrc;
  	output wire lmulFlag;
  	output reg [2:0] ALUControl;
  	output wire FpuW;
	wire Branch;
	wire ALUOp;
  	output wire RegSrcMul;//Cambios en los operandos 
  	output wire mullargo;
  	reg mullargo_reg;
  
  	assign mullargo = mullargo_reg;

	// Main FSM
	mainfsm fsm(
		.clk(clk),
		.reset(reset),
		.Op(Op),
		.Funct(Funct),
		.IRWrite(IRWrite),
		.AdrSrc(AdrSrc),
		.ALUSrcA(ALUSrcA),
		.ALUSrcB(ALUSrcB),
		.ResultSrc(ResultSrc),
		.NextPC(NextPC),
		.RegW(RegW),
      	.RegW2(RegW2),
		.MemW(MemW),
		.Branch(Branch),
      	.ALUOp(ALUOp),	
      	.mullargo(mullargo),
      	.lmulFlag(lmulFlag),
      	.FpuW(FpuW)
	);

	// ADD CODE BELOW
	// Add code for the ALU Decoder and PC Logic.
	// Remember, you may reuse code from previous labs.
	// ALU Decoder
	always @(*) begin
      	mullargo_reg = 0;
    	if (ALUOp) begin
        	if (Mop[3:0] == 4'b1001) begin
            	case (Funct[4:1])
                	4'b0000: ALUControl = 3'b101;       // MUL
                  	4'b0100: begin
                         ALUControl = 3'b110; //UMULL
                      	 mullargo_reg = 1; 
                    end
                  
                  	4'b0110: begin
                         ALUControl = 3'b111; //SMULL
                      	 mullargo_reg = 1; 
                    end
            	endcase
        	end
          else if(Mop[3:0] == 4'b0111) begin
            if (Funct[4:1] == 4'b0001)
              ALUControl= 3'b110;  //UDIV
          end
          else begin
            	case (Funct[4:1])
                	4'b0100: ALUControl = 3'b000;       // ADD
                	4'b0010: ALUControl = 3'b001;       // SUB
                	4'b0000: ALUControl = 3'b010;       // AND
                	4'b1100: ALUControl = 3'b011;       // ORR
                	4'b0001: ALUControl = 3'b100;       // EOR
                	default: ALUControl = 3'bxxx;
            	endcase
        	end
        	FlagW[1] = Funct[0];
        	FlagW[0] = Funct[0] & ((ALUControl == 3'b000) | (ALUControl == 3'b001));
    	end
      
      //
    	else begin
        	ALUControl = 3'b000;
        	FlagW = 2'b00;
    	end
	end
	// PC Logic
	assign PCS = ((Rd == 4'b1111) & RegW) | Branch;
	// Add code for the Instruction Decoder (Instr Decoder) below.
	// Recall that the input to Instr Decoder is Op, and the outputs are
	// ImmSrc and RegSrc. We've completed the ImmSrc logic for you.

	// Instr Decoder
	assign ImmSrc = Op;
  	assign RegSrc[1] = Op == 2'b01; // RegSrc1 is 1 for STR, 0 for DP and the rest don't care
	assign RegSrc[0] = Op == 2'b10; // RegSrc0 is only 1 for B instructions
          
    assign RegSrcMul = Mop[3:0] == 4'b1001;
endmodule

module mainfsm (
	clk,
	reset,
	Op,
	Funct,
	IRWrite,
	AdrSrc,
	ALUSrcA,
	ALUSrcB,
	ResultSrc,
	NextPC,
	RegW,
  	RegW2,
	MemW,
	Branch,
	ALUOp,
  	mullargo,
  	lmulFlag,
  	FpuW
);
	input wire clk;
	input wire reset;
	input wire [1:0] Op;
	input wire [5:0] Funct;
	output wire IRWrite;
	output wire AdrSrc;
	output wire [1:0] ALUSrcA;
	output wire [1:0] ALUSrcB;
	output wire [1:0] ResultSrc;
	output wire NextPC;
	output wire RegW;
  	output wire RegW2;
	output wire MemW;
	output wire Branch;
	output wire ALUOp;
  	output wire lmulFlag;
  	output wire FpuW;
	reg [3:0] state;
	reg [3:0] nextstate;
  	reg [14:0] controls;
    input wire mullargo;
  
	localparam [3:0] FETCH = 0;
	localparam [3:0] DECODE = 1;
	localparam [3:0] MEMADR = 2;
	localparam [3:0] MEMRD = 3;
	localparam [3:0] MEMWB = 4;
	localparam [3:0] MEMWR = 5;
	localparam [3:0] EXECUTER = 6;
	localparam [3:0] EXECUTEI = 7;
	localparam [3:0] ALUWB = 8;
	localparam [3:0] BRANCH = 9;
	localparam [3:0] UNKNOWN = 10;
  	localparam [3:0] EXECUTEF = 11; 
  	localparam [3:0] FPUWB    = 12; 
  	localparam [3:0] ALUWB2   = 13; 

	// state register
	always @(posedge clk or posedge reset)
		if (reset)
			state <= FETCH;
		else
			state <= nextstate;
	

	// ADD CODE BELOW
  	// Finish entering the next state logic below.  We've completed the 
  	// first two states, FETCH and DECODE, for you.

  	// next state logic
	always @(*)
		casex (state)
			FETCH: nextstate = DECODE;
			DECODE:
				case (Op)
					2'b00:
						if (Funct[5])
							nextstate = EXECUTEI;
						else
							nextstate = EXECUTER;
					2'b01: nextstate = MEMADR;
					2'b10: nextstate = BRANCH;
                  	2'b11: nextstate = EXECUTEF;
					default: nextstate = UNKNOWN;
				endcase
			MEMADR:
				if (Funct[0]) 
					nextstate = MEMRD;
				else
					nextstate = MEMWR;
          
			MEMRD: nextstate = MEMWB;
			MEMWB: nextstate = FETCH;
			MEMWR: nextstate = FETCH;
			EXECUTER: nextstate = mullargo == 1? ALUWB2 : ALUWB;
			EXECUTEI: nextstate = mullargo == 1? ALUWB2 : ALUWB;
          	EXECUTEF: nextstate = FPUWB;
            FPUWB:    nextstate = FETCH;
			ALUWB: nextstate = FETCH;
			BRANCH: nextstate = FETCH;
          	ALUWB2:   nextstate = FETCH;
			default: nextstate = FETCH;
		endcase

	// ADD CODE BELOW
	// Finish entering the output logic below.  We've entered the
	// output logic for the first two states, FETCH and DECODE, for you.

	// state-dependent output logic
	always @(*) begin
		case (state)
			FETCH: controls =    16'b0100010100110000;
			DECODE: controls =   16'b0000000100110000;
          	EXECUTER: controls = 16'b0000000000000100;
			EXECUTEI: controls = 16'b0000000000001100;
          	ALUWB: controls =    16'b0000100000000000;
          	MEMADR: controls =   16'b0000000000001000;
          	MEMWR: controls =    16'b0001001000000000;
			MEMRD: controls =    16'b0000001000000000;
			MEMWB: controls =    16'b0000100010000000;
          	BRANCH: controls =   16'b0010000101001000;
          	EXECUTEF: controls = 16'b0000000000000000;
            FPUWB:    controls = 16'b0000000000000001;
          	ALUWB2: controls = 	 16'b1000100000000010;
			default: controls =  16'bxxxxxxxxxxxxxxxx;
		endcase
	end
  assign {RegW2, NextPC, Branch, MemW, RegW, IRWrite, AdrSrc,
        ResultSrc, ALUSrcA, ALUSrcB, ALUOp, lmulFlag, FpuW } = controls;
endmodule

// ADD CODE BELOW
// Add code for the condlogic and condcheck modules. Remember, you may
// reuse code from prior labs.
module condlogic (
	clk,
	reset,
	Cond,
	ALUFlags,
	FlagW,
	PCS,
	NextPC,
	RegW,
	RegW2,
	MemW,
  	FpuW,
	PCWrite,
	RegWrite,
	MemWrite,
  	FpuWrite
);
	input wire clk;
	input wire reset;
	input wire [3:0] Cond;
	input wire [3:0] ALUFlags;
	input wire [1:0] FlagW;
	input wire PCS;
	input wire NextPC;
	input wire RegW;
  	input wire RegW2;
	input wire MemW;
  	input wire FpuW;
	output wire PCWrite;
  	output wire [1:0] RegWrite;
	output wire MemWrite;
  	output wire FpuWrite;
	wire [1:0] FlagWrite;
	wire [3:0] Flags;
	wire CondEx;
  
  	wire actualCondEx;
  	wire PCSrc;


	// ADD CODE HERE
	flopenr #(2) flagreg1 (     // N y Z
        .clk(clk),
		.reset(reset),
		.en(FlagWrite[1]),
		.d(ALUFlags[3:2]),
		.q(Flags[3:2])           
    );
  
  	flopenr #(2) flagreg0 (     
        .clk(clk),
		.reset(reset),
		.en(FlagWrite[0]),
		.d(ALUFlags[1:0]),
		.q(Flags[1:0])
    );
  
  	flopr #(1) condexreg (
        .clk(clk),
		.reset(reset),
		.d(CondEx),
		.q(actualCondEx)
    );
  
  	condcheck cc (
        .Cond(Cond),
        .Flags(Flags),
        .CondEx(CondEx)
    );
  
  	assign FlagWrite = FlagW & {2 {CondEx}};
  	assign RegWrite[0] = RegW & actualCondEx;
  	assign RegWrite[1]  = RegW2  & actualCondEx;
    assign MemWrite = MemW & actualCondEx;
  	assign FpuWrite = actualCondEx & FpuW;
    assign PCSrc   = PCS & actualCondEx;
    assign PCWrite = PCSrc | NextPC;
  	
endmodule

module condcheck (
	Cond,
	Flags,
	CondEx
);
	input wire [3:0] Cond;
	input wire [3:0] Flags;
	output reg CondEx;

	// ADD CODE HERE
  	wire neg;
	wire zero;
	wire carry;
	wire overflow;
	wire ge;
	assign {neg, zero, carry, overflow} = Flags;
	assign ge = neg == overflow;
	always @(*)
		case (Cond)
			4'b0000: CondEx = zero;
			4'b0001: CondEx = ~zero;
			4'b0010: CondEx = carry;
			4'b0011: CondEx = ~carry;
			4'b0100: CondEx = neg;
			4'b0101: CondEx = ~neg;
			4'b0110: CondEx = overflow;
			4'b0111: CondEx = ~overflow;
			4'b1000: CondEx = carry & ~zero;
			4'b1001: CondEx = ~(carry & ~zero);
			4'b1010: CondEx = ge;
			4'b1011: CondEx = ~ge;
			4'b1100: CondEx = ~zero & ge;
			4'b1101: CondEx = ~(~zero & ge);
			4'b1110: CondEx = 1'b1;
			default: CondEx = 1'bx;
		endcase
endmodule

// ADD CODE BELOW
// Complete the datapath module below for Lab 11.
// You do not need to complete this module for Lab 10.
// The datapath unit is a structural SystemVerilog module. That is,
// it is composed of instances of its sub-modules. For example,
// the instruction register is instantiated as a 32-bit flopenr.
// The other submodules are likewise instantiated. 
module datapath (
	clk,
	reset,
	Adr,
	WriteData,
	ReadData,
	Instr,
	ALUFlags,
	PCWrite,
	RegWrite,
	IRWrite,
	AdrSrc,
	RegSrc,
	ALUSrcA,
	ALUSrcB,
	ResultSrc,
	ImmSrc,
	ALUControl,
  	lmulFlag,
  	FpuWrite,
  	RegSrcMul,
  	mullargo,
);
	input wire clk;
	input wire reset;
	output wire [31:0] Adr;
	output wire [31:0] WriteData;
	input wire [31:0] ReadData;
	output wire [31:0] Instr;
	output wire [3:0] ALUFlags;
	input wire PCWrite;
	input wire RegWrite;
	input wire IRWrite;
	input wire AdrSrc;
	input wire [1:0] RegSrc;
	input wire [1:0] ALUSrcA;
	input wire [1:0] ALUSrcB;
	input wire [1:0] ResultSrc;
	input wire [1:0] ImmSrc;
  	input wire [2:0] ALUControl;
  	
  	input wire lmulFlag;
  	input wire FpuWrite;
  	input wire RegSrcMul;
  	input wire mullargo;
  
	wire [31:0] PCNext;
	wire [31:0] PC;
	wire [31:0] ExtImm;
	wire [31:0] SrcA;
	wire [31:0] SrcB;
	wire [31:0] Result;
	wire [31:0] Data;
	wire [31:0] RD1;
	wire [31:0] RD2;
	wire [31:0] A;
	wire [31:0] ALUResult;
    wire [31:0] ALUResult2;
	wire [31:0] ALUOut;
  	wire [31:0] ALUOut2;
	wire [3:0] RA1;
	wire [3:0] RA2;
  	wire [63:0] FRD1;
    wire [63:0] FRD2;
    wire [63:0] FA;
    wire [63:0] FWriteData;
    wire [63:0] FResult;
    wire [63:0] FPUResult;
  
  
  	wire [3:0] _RA1, _RA2, A3;

	// Your datapath hardware goes below. Instantiate each of the 
	// submodules that you need. Remember that you can reuse hardware
	// from previous labs. Be sure to give your instantiated modules 
	// applicable names such as pcreg (PC register), adrmux 
	// (Address Mux), etc. so that your code is easier to understand.

	// ADD CODE HERE
  	flopenr #(32) pcreg(
		.clk(clk),
		.reset(reset),
		.en(PCWrite),
		.d(Result),
		.q(PC)
	);
  
	mux2 #(32) adrmux(
		.d0(PC),
		.d1(Result),
		.s(AdrSrc),
		.y(Adr)
	);
  	
	// here goes (implicitly) the instruction/data memory
	flopenr #(32) instrreg(
		.clk(clk),
		.reset(reset),
		.en(IRWrite),
		.d(ReadData),
		.q(Instr)
	);
  
	flopr #(32) readdatareg(
		.clk(clk),
		.reset(reset),
		.d(ReadData),
		.q(Data)
	);
  
 	//RA1
	
  	mux2 #(4) ra1mulmux(
      .d0(Instr[19:16]), 
      .d1(Instr[3:0]), 
      .s(RegSrcMul), 
      .y(_RA1)
    );
  
    mux2 #(4) ra1mux(
    	.d0(_RA1),
    	.d1(4'd15),
    	.s(RegSrc[0]),
    	.y(RA1)
	);
  
  	//RA2
  
  	mux2 #(4) ra2mulmux(
    	.d0(Instr[3:0]),    // Rm normal
		.d1(Instr[11:8]),
    	.s(RegSrcMul),
    	.y(_RA2)
	);
  
  
  	mux2 #(4) ra2mux(
    	.d0(_RA2),
    	.d1(Instr[15:12]),
    	.s(RegSrc[1]),
    	.y(RA2)
	);
  
  	//A3
  
  	mux2 #(4) a3mux(
    	.d0(Instr[15:12]),  // Rd normal
    	.d1(Instr[19:16]),  // Rn como destino en MUL
    	.s(RegSrcMul),
    	.y(A3)
	);
  
  	regfile rf(
		.clk(clk),
		.we3(RegWrite),
		.ra1(RA1),
		.ra2(RA2),
      	.wa3(A3),
      	.wa4(Instr[15:12]),
		.wd3(Result),
      	.wd4(ALUOut2),
      	.mullargo(lmulFlag),
		.r15(Result),
		.rd1(RD1),
		.rd2(RD2)
	);
  
  
  	extend ext(
		.Instr(Instr[23:0]),
		.ImmSrc(ImmSrc),
		.ExtImm(ExtImm)
    );
  
  
  	flopr #(64) rdreg(
      .clk(clk), 
      .reset(reset), 
      .d({RD1, RD2}), 
      .q({A, WriteData})
    );
  
  	//amux
  	mux2 #(32) srcamux(
		.d0(A),
		.d1(PC),
      	.s(ALUSrcA[0]),
		.y(SrcA)
	);
  
  	mux3 #(32) srcbmux(
		.d0(WriteData),
		.d1(ExtImm),
      	.d2(32'd4),
		.s(ALUSrcB),
		.y(SrcB)
	);
  
  	alu alu(
		SrcA,
		SrcB,
          ALUControl,
      	mullargo,
		ALUResult,
      	ALUResult2,
		ALUFlags
	);
  	
  	
	fpu_regfile fpu_regfile(
      .clk(clk), 
      .we3(FpuWrite), 
      .ra1(Instr[19:16]), 
      .ra2(Instr[3:0]), 
      .wa3(Instr[15:12]), 
      .A1(Instr[7]), 
      .A2(Instr[5]), 
      .A3(Instr[6]), 
      .sod(Instr[8]), 
      .wd3(FResult), 
      .rd1(FRD1), 
      .rd2(FRD2)
    );
  	
  	flopr #(128) frdreg(
      .clk(clk), 
      .reset(reset), 
      .d({FRD1, FRD2}), 
      .q({FA, FWriteData})
    );
  
  	fpu f(
      .a(FA), 
      .b(FWriteData), 
      .double(Instr[8]),
      .isMul(Instr[4]),
      .Result(FPUResult)
    );
  	
  	flopr #(64) fpureg(
      .clk(clk), 
      .reset(reset), 
      .d(FPUResult), 
      .q(FResult)
    );
    
    
  
	flopr #(32) aluresultreg(
		.clk(clk),
		.reset(reset),
		.d(ALUResult),
		.q(ALUOut)
	);
  
  flopr #(32) aluresultreg2(
		.clk(clk),
		.reset(reset),
    	.d(ALUResult2),
    	.q(ALUOut2)
	);
  
	mux3 #(32) resmux(
		.d0(ALUOut),
		.d1(Data),
		.d2(ALUResult),
		.s(ResultSrc),
		.y(Result)
	);
endmodule

// ADD CODE BELOW
// Add needed building blocks below (i.e., parameterizable muxes, 
// registers, etc.). Remember, you can reuse code from previous labs.
// We've also provided a parameterizable 3:1 mux below for your 
// convenience.

module mux3 (
	d0,
	d1,
	d2,
	s,
	y
);
	parameter WIDTH = 8;
	input wire [WIDTH - 1:0] d0;
	input wire [WIDTH - 1:0] d1;
	input wire [WIDTH - 1:0] d2;
	input wire [1:0] s;
	output wire [WIDTH - 1:0] y;
	assign y = (s[1] ? d2 : (s[0] ? d1 : d0));
endmodule


module flopr (
	clk,
	reset,
	d,
	q
);
	parameter WIDTH = 8;
	input wire clk;
	input wire reset;
	input wire [WIDTH - 1:0] d;
	output reg [WIDTH - 1:0] q;
	always @(posedge clk or posedge reset)
		if (reset)
			q <= 0;
		else
			q <= d;
endmodule


module alu (
    input  wire [31:0] a,
    input  wire [31:0] b,
    input  wire [2:0]  ALUControl,   
                                     
    input  wire        mullargo,     
    output reg  [31:0] Result,       
    output reg  [31:0] Result2,      
    output wire [3:0]  ALUFlags      
);


    wire [31:0] condinvb  = ALUControl[0] ? ~b : b;  
    wire [32:0] sum       = a + condinvb + ALUControl[0];

    always @(*) begin

        Result  = 32'd0;
        Result2 = 32'd0;

        case (ALUControl)

            3'b000,
            3'b001:  Result = sum[31:0];


            3'b010:  Result = a & b;
            3'b011:  Result = a | b;
            3'b100:  Result = a ^ b;
            3'b101:  Result = a * b;

          
            3'b110:  begin
                         if (mullargo)
                             {Result2, Result} = a * b;     
                         else begin
                             Result  = a / b;               
                             Result2 = a % b;               
                         end
                     end

            
            3'b111:  {Result2, Result} = $signed(a) * $signed(b);
        endcase
    end


    wire N = mullargo ? Result2[31]                     
                      : Result[31];                    

    wire Z = mullargo ? ((Result2 == 32'd0) &&
                         (Result  == 32'd0))            
                      : (Result == 32'd0);             
    wire C = (!mullargo) &&
             (ALUControl[1] == 1'b0) && sum[32];

    wire V = (!mullargo) &&
             (ALUControl[1] == 1'b0) &&
             ~(a[31] ^ b[31] ^ ALUControl[0]) &
              (a[31] ^ sum[31]);

    assign ALUFlags = {N, Z, C, V};

endmodule


module flopr2 (
	clk,
	reset,
	d0,
    d1,
	q0,
    q1
);
	parameter WIDTH = 8;
	input wire clk;
	input wire reset;
	input wire [WIDTH - 1:0] d0;
    input wire [WIDTH - 1:0] d1;
	output reg [WIDTH - 1:0] q0;
    output reg [WIDTH - 1:0] q1;
	always @(posedge clk or posedge reset) begin
		if (reset) begin
			q0 <= 0;
            q1 <= 0;
        end
		else begin
			q0 <= d0;
            q1 <= d1;
        end
    end
endmodule

module extend (
	Instr,
	ImmSrc,
	ExtImm
);
	input wire [23:0] Instr;
	input wire [1:0] ImmSrc;
	output reg [31:0] ExtImm;
	always @(*)
		case (ImmSrc)
			2'b00: ExtImm = {24'b000000000000000000000000, Instr[7:0]};
			2'b01: ExtImm = {20'b00000000000000000000, Instr[11:0]};
			2'b10: ExtImm = {{6 {Instr[23]}}, Instr[23:0], 2'b00};
			default: ExtImm = 32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx;
		endcase
endmodule

module regfile (
	clk,
	we3,
	ra1,
	ra2,
	wa3,
  	wa4,
	wd3,
  	wd4,
  	mullargo,
	r15,
	rd1,
	rd2
);
	input wire clk;
	input wire we3;
	input wire [3:0] ra1;
	input wire [3:0] ra2;
	input wire [3:0] wa3;
  	input wire [3:0] wa4;
	input wire [31:0] wd3;
  	input wire [31:0] wd4;
  	input wire mullargo;
	input wire [31:0] r15;
	output wire [31:0] rd1;
	output wire [31:0] rd2;
	reg [31:0] rf [14:0];
	always @(posedge clk)
		if (we3) begin
			rf[wa3] <= wd3;
          if(mullargo) begin
            rf[wa4] <=wd4;
          end
		end
	assign rd1 = (ra1 == 4'b1111 ? r15 : rf[ra1]);
	assign rd2 = (ra2 == 4'b1111 ? r15 : rf[ra2]);
endmodule

module mux2 (
	d0,
	d1,
	s,
	y
);
	parameter WIDTH = 8;
	input wire [WIDTH - 1:0] d0;
	input wire [WIDTH - 1:0] d1;
	input wire s;
	output wire [WIDTH - 1:0] y;
	assign y = (s ? d1 : d0);
endmodule

module flopenr (
	clk,
	reset,
	en,
	d,
	q
);
	parameter WIDTH = 8;
	input wire clk;
	input wire reset;
	input wire en;
	input wire [WIDTH - 1:0] d;
	output reg [WIDTH - 1:0] q;
	always @(posedge clk or posedge reset)
		if (reset)
			q <= 0;
		else if (en)
			q <= d;
endmodule


module double_adder (
    input  [63:0] srcA,
    input  [63:0] srcB,
    output [63:0] result
);
    // Desempaquetar
    wire        signA = srcA[63];
    wire        signB = srcB[63];
    wire [10:0] expA  = srcA[62:52];
    wire [10:0] expB  = srcB[62:52];
    wire [52:0] manA  = {1'b1, srcA[51:0]};
    wire [52:0] manB  = {1'b1, srcB[51:0]};

    // Mayor y menor
    wire swap      = (expB > expA) |
                     ((expA == expB) & (manB > manA));
    wire [10:0] expL     = swap ? expB : expA;
    wire [10:0] expS     = swap ? expA : expB;
    wire [52:0] manL     = swap ? manB : manA;
    wire [52:0] manS     = swap ? manA : manB;
    wire        signL    = swap ? signB : signA;

    // Alineamiento y suma
    wire [10:0] diff     = expL - expS;
    wire [52:0] manSsh   = manS >> diff;
    wire [53:0] sum      = manL + manSsh;
    wire        carry    = sum[53];

    wire [10:0] expTmp   = expL + carry;
    wire [52:0] manTmp   = carry ? sum[53:1] : sum[52:0];

    reg  [4:0] lshift;
    always @(*) begin
        casex (manTmp[52:50])
            3'b1??: lshift = 5'd0;
            3'b01?: lshift = 5'd1;
            3'b001: lshift = 5'd2;
            default: lshift = 5'd3;
        endcase
    end

    wire [52:0] manShift = manTmp << lshift;
    wire [51:0] mantOK   = manShift[52:1];
    wire [10:0] expOK    = expTmp - lshift;

    assign result = {signL, expOK, mantOK};
endmodule



module fp_adder (
    input  [31:0] srcA,
    input  [31:0] srcB,
    output [31:0] result
);
    wire        signA = srcA[31];
    wire        signB = srcB[31];
    wire  [7:0] expA  = srcA[30:23];
    wire  [7:0] expB  = srcB[30:23];
    wire [23:0] manA  = {1'b1, srcA[22:0]};
    wire [23:0] manB  = {1'b1, srcB[22:0]};

    wire swap      = (expB > expA) |
                     ((expA == expB) & (manB > manA));
    wire  [7:0] expL     = swap ? expB : expA;
    wire  [7:0] expS     = swap ? expA : expB;
    wire [23:0] manL     = swap ? manB : manA;
    wire [23:0] manS     = swap ? manA : manB;
    wire        signL    = swap ? signB : signA;

    wire  [7:0] diff     = expL - expS;
    wire [23:0] manSsh   = manS >> diff;
    wire [24:0] sum      = manL + manSsh;
    wire        carry    = sum[24];

    wire  [7:0] expTmp   = expL + carry;
    wire [23:0] manTmp   = carry ? sum[24:1] : sum[23:0];

    reg  [1:0] lshift;
    always @(*) begin
        casez (manTmp[23:22])
            2'b1?:  lshift = 2'd0;
            2'b01:  lshift = 2'd1;
            default lshift = 2'd2;
        endcase
    end

    wire [23:0] manShift = manTmp << lshift;
    wire [22:0] mantOK   = manShift[23:1];
    wire  [7:0] expOK    = expTmp - lshift;

    assign result = {signL, expOK, mantOK};
endmodule


module fp_multiplier (
    input  [31:0] a,
    input  [31:0] b,
    output [31:0] result
);
    wire sign = a[31] ^ b[31];
    wire [7:0] expA = a[30:23];
    wire [7:0] expB = b[30:23];
    wire [23:0] manA = {1'b1, a[22:0]};
    wire [23:0] manB = {1'b1, b[22:0]};
    
    wire [47:0] manMult = manA * manB;
    wire [7:0] expMult = expA + expB - 127;

    wire norm = manMult[47]; // overflow, shift right
    wire [7:0] expNorm = norm ? expMult + 1 : expMult;
    wire [22:0] manNorm = norm ? manMult[46:24] : manMult[45:23];

    assign result = {sign, expNorm, manNorm};
endmodule

module double_multiplier (
    input  [63:0] a,
    input  [63:0] b,
    output [63:0] result
);
    wire sign = a[63] ^ b[63];
    wire [10:0] expA = a[62:52];
    wire [10:0] expB = b[62:52];
    wire [52:0] manA = {1'b1, a[51:0]};
    wire [52:0] manB = {1'b1, b[51:0]};

    wire [105:0] manMult = manA * manB;
    wire [10:0] expMult = expA + expB - 1023;

    wire norm = manMult[105]; // overflow, shift right
    wire [10:0] expNorm = norm ? expMult + 1 : expMult;
    wire [51:0] manNorm = norm ? manMult[104:53] : manMult[103:52];

    assign result = {sign, expNorm, manNorm};
endmodule



module fpu (
    input  [63:0] a, b,
    input         double,   // 0: float, 1: double
    input         isMul,    // 0: add,   1: mul
    output [63:0] Result
);
    wire [31:0] f_add, f_mul;
    wire [63:0] d_add, d_mul;

    // Submódulos
    fp_adder         fadd (.srcA(a[31:0]), .srcB(b[31:0]), .result(f_add));
    fp_multiplier    fmul (.a(a[31:0]),    .b(b[31:0]),    .result(f_mul));
    double_adder     dadd (.srcA(a),       .srcB(b),       .result(d_add));
    double_multiplier dmul (.a(a),         .b(b),          .result(d_mul));

    assign Result = double ?
                    (isMul ? d_mul : d_add) :
                    {32'd0, (isMul ? f_mul : f_add)};
endmodule



module fpu_regfile (
    input  wire         clk,
    input  wire         we3, // 1= esribe el registro flotante 
  input  wire  [3:0]  ra1, ra2, wa3, //indice para leer operandos ra1 y ra2 y indices para escribir resultados wa3
    input  wire         A1,  A2,  A3,   // 0=baja, 1=alta
    input  wire         sod,            // 1=double, 0=float
  input  wire  [63:0] wd3, //lo que se escribe en el registro
    output wire  [63:0] rd1, rd2
);
    reg [63:0] rf [15:0];

    always @(posedge clk)
        if (we3) begin
            if (sod)
                rf[wa3] <= wd3;                // double
            else if (A3)
                rf[wa3][63:32] <= wd3[31:0];   // float alta
            else
                rf[wa3][31:0]  <= wd3[31:0];   // float baja
        end

    wire [31:0] rd1f = A1 ? rf[ra1][63:32] : rf[ra1][31:0];
    wire [31:0] rd2f = A2 ? rf[ra2][63:32] : rf[ra2][31:0];

    assign rd1 = sod ? rf[ra1] : {32'd0, rd1f};
    assign rd2 = sod ? rf[ra2] : {32'd0, rd2f};
endmodule


