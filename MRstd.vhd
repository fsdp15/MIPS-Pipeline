library IEEE;
use IEEE.STD_LOGIC_1164.all;

package p_MI0 is  
  
  subtype reg32 is std_logic_vector(31 downto 0);  
  -- tipo para os barramentos de 32 bits
  
  type inst_type is (ADDU, SUBU, AAND, OOR, XXOR, NNOR, LW, SW, 
  	ORI, invalid_instruction);

  type microinstruction is record
    ce:    std_logic;       -- ce e rw são os controles da memória
    rw:    std_logic;
    i:     inst_type;        
    wreg:  std_logic;       -- wreg diz se o banco de registradores
							-- deve ou não ser escrito
  end record;
    
end p_MI0;

library IEEE; 
use IEEE.STD_LOGIC_1164.all;
use work.p_MI0.all;

-----------------------------------------------------------------------------------

library IEEE; ---- Registradores
use IEEE.STD_LOGIC_1164.all; 
use IEEE.STD_LOGIC_ARITH.all;
use work.p_MI0.all;

entity reg is -- flip-flop com rst, enable e clear
	generic (width: integer); --Tamanho do registrador é maleável
	port (ck, rst, ce, clear, eh_pc, eh_banco: in STD_LOGIC;
		D: in STD_LOGIC_VECTOR(width-1 downto 0);
		Q: out STD_LOGIC_VECTOR(width-1 downto 0));
end;

architecture reg of reg is
begin
	process(ck, rst, ce, clear) begin
		if ck'event and ck = '1' and eh_banco = '0' then
			if rst = '1' AND eh_pc ='1' then -- eh_pc verifica se é o pc inicial, que deve ser 0x00400000
				Q <= x"00400000";
			elsif rst ='1' then 
				Q <= CONV_STD_LOGIC_VECTOR(0, width);
			elsif clear = '1' then --clear é para caso de stall
				Q <= CONV_STD_LOGIC_VECTOR(0, width);
			elsif ce = '1' then 
				Q <= D;
			end if;
		end if;

		if ck'event and ck = '0' and eh_banco = '1' then -- eh_banco verifica se é registrador do banco
			if rst ='1' then 	--Os registradores do banco possuem escrita na borda de descida
				Q <= CONV_STD_LOGIC_VECTOR(0, width);
			elsif clear = '1' then 
				Q <= CONV_STD_LOGIC_VECTOR(0, width);
			elsif ce = '1' then 
				Q <= D;
			end if;
		end if;
	end process;

end;

----------------------------------------------------------------------------------------

library IEEE; -- Banco de registradores
use IEEE.Std_Logic_1164.all;
use ieee.STD_LOGIC_UNSIGNED.all;   
use work.p_MI0.all;

entity reg_bank is
       port( ck, rst, wreg :    in std_logic;
             AdRs, AdRt, adRD : in std_logic_vector( 4 downto 0);
             RD : in reg32;
             R1, R2: out reg32 
           );
end reg_bank;

architecture reg_bank of reg_bank is
   type bank is array(0 to 31) of reg32;
   signal reg : bank ;                            
   signal wen : reg32 ;
begin            

    g1: for i in 0 to 31 generate        

        wen(i) <= '1' when i/=0 and adRD=i and wreg='1' else '0';
         
        rx: entity work.reg
			generic map(32)
			port map (ck=>ck, rst=>rst, ce=>wen(i), clear=>'0', eh_pc=>'0', D=>RD, Q=>reg(i), eh_banco=>'1');                   
        
    end generate g1;      

    R1 <= reg(CONV_INTEGER(AdRs));    -- seleção do fonte 1  

    R2 <= reg(CONV_INTEGER(AdRt));    -- seleção do fonte 2 
   
end reg_bank;

----------------------------------------------------------------------------------------

library IEEE; --ULA
use IEEE.std_logic_1164.all;
use IEEE.std_logic_unsigned.all;
use work.p_MI0.all;

entity alu is
       port(alu_op1, alu_op2 : in  reg32;
	    alu_outalu :   out reg32; 
        alu_op_alu :  in STD_LOGIC_VECTOR(2 downto 0)
           );
end alu;

architecture alu_arq of alu is --é usado o alucontrol decodificado no controle para determinar a operação da ALU
signal alu_int_alu	: reg32;
begin
    alu_outalu <= alu_int_alu;
    alu_int_alu <=  
        alu_op1 - alu_op2      when  alu_op_alu="110" 			else
        alu_op1 and alu_op2    when  alu_op_alu="011" 			else 
        alu_op1 or  alu_op2    when  alu_op_alu="001" 			else 
        alu_op1 xor alu_op2    when  alu_op_alu="101"            	else 
        alu_op1 nor alu_op2    when  alu_op_alu="111"             	else 
        alu_op1 + alu_op2;      --- default é a soma
end alu_arq;

-------------------------------------------------------------------------------------------------

library IEEE; -- Mux 3
use IEEE.STD_LOGIC_1164.all;
use work.p_MI0.all;
entity mux3 is
	generic (width: integer);
	port (d0, d1, d2: in STD_LOGIC_VECTOR(width-1 downto 0);
	s: in STD_LOGIC_VECTOR(1 downto 0);
	y: out STD_LOGIC_VECTOR(width-1 downto 0));
end;

architecture mux3 of mux3 is
begin
	process(s, d0, d1, d2) begin
	case s is
	when "00" => y <= d0;
	when "01" => y <= d1; 
	when "10" => y <= d2;
	when others => y <= d0;
	end case;
	end process;
end;

-----------------------------------------------------------------------------

library IEEE; -- Mux 2
use IEEE.STD_LOGIC_1164.all;
use work.p_MI0.all;
entity mux2 is 
	generic (width: integer);
	port (d0, d1: in STD_LOGIC_VECTOR(width-1 downto 0);
	s: in STD_LOGIC;
	y: out STD_LOGIC_VECTOR(width-1 downto 0));
end;

architecture mux2 of mux2 is
begin
	y <= d0 when s = '0' else d1;
end;

-------------------------------------------------------------------------------

library IEEE; 
use IEEE.STD_LOGIC_1164.all;
use IEEE.STD_LOGIC_UNSIGNED.all;
use work.p_MI0.all;
entity somador is -- Somador 32 bits
	port (a, b: in STD_LOGIC_VECTOR(31 downto 0);
	y: out STD_LOGIC_VECTOR(31 downto 0));
end;

architecture somador of somador is
begin
	y <= a + b;
end;

-----------------------------------------------------------------------------------------------------------------------

library IEEE; 
use IEEE.STD_LOGIC_1164.all;
use work.p_MI0.all;

entity hazard is -- Unidade de hazard
	port(rsD, rtD, rsE, rtE: in STD_LOGIC_VECTOR(4 downto 0);
	writeregM, writeregW: in STD_LOGIC_VECTOR(4 downto 0);
	regwriteM, regwriteW: in STD_LOGIC;
	memtoregE: in STD_LOGIC;
	forwardaE, forwardbE: out STD_LOGIC_VECTOR(1 downto 0);
	stallF, flushE: out STD_LOGIC;
	stallD: inout STD_LOGIC);
end;

architecture hazard of hazard is

	signal lwstallD: STD_LOGIC;
	
begin
	
	-- forwarding para estágio EXEC (ALU)
	process(rsE, rtE, writeregM, regwriteM, writeregW, regwriteW) begin
		forwardaE <= "00"; forwardbE <= "00";
		if (rsE /= "00000") then
			if ((rsE = writeregM) and (regwriteM = '1')) then
			forwardaE <= "10";
		elsif ((rsE = writeregW) and (regwriteW = '1')) then
			forwardaE <= "01";
		end if;
		end if;
		if (rtE /= "00000") then
			if ((rtE = writeregM) and (regwriteM = '1')) then
			forwardbE <= "10";
			elsif ((rtE = writeregW) and (regwriteW = '1')) then
			forwardbE <= "01";
			end if;
		end if;
	end process;
	
	-- stalls
	lwstallD <= '1' when ((memtoregE = '1') and ((rtE = rsD) or (rtE = rtD)))
	else '0';
	stallD <= (lwstallD) after 1 ns;
	stallF <= stallD after 1 ns; 
	flushE <= stallD after 1 ns; 
	
end;

----------------------------------------------------------------------------

library IEEE; 
use IEEE.STD_LOGIC_1164.all; 
use IEEE.STD_LOGIC_ARITH.all;
use work.p_MI0.all;

entity datapath is -- datapath
	port(clk, rst: in STD_LOGIC;
	memtoregE, memtoregM, memtoregW: in STD_LOGIC;
	alusrcE, regdstE: in STD_LOGIC;
	regwriteE, regwriteM, regwriteW: in STD_LOGIC;
	alucontrolE: in STD_LOGIC_VECTOR(2 downto 0);
	pcF: inout reg32;
	instrF: in reg32;
	aluoutM, writedataM: inout reg32;
	readdataM: in reg32;
	opD, functD: out STD_LOGIC_VECTOR(5 downto 0);
	flushE: inout STD_LOGIC);
end;

architecture datapath of datapath is

	signal forwardaE, forwardbE: STD_LOGIC_VECTOR(1 downto 0);
	signal stallF, stallFbar, stallD, stallDbar: STD_LOGIC;
	signal rsD, rtD, rdD, rsE, rtE, rdE: STD_LOGIC_VECTOR(4 downto 0);
	signal writeregE, writeregM, writeregW: STD_LOGIC_VECTOR(4 downto 0);
	signal flushD: STD_LOGIC;
	signal pcnext, pcplus4: reg32;
	signal signimmD, signimmE: reg32;
	signal srcaD, srca2D, srcaE, ALU_A: reg32;
	signal srcbD, srcb2D, srcbE, srcb2E, ALU_B: reg32;
	signal pcplus4D, instrD, instrE, instrM, instrW: reg32;
	signal aluoutE, aluoutW: reg32;
	signal readdataW, resultW: reg32;
	signal pcD, pcE, pcM, pcW: reg32;
	
begin

	-- Detecção de hazards
	h: entity work.hazard port map(rsD=>rsD, rtD=>rtD, rsE=>rsE, rtE=>rtE, writeregM=>writeregM,
	writeregW=>writeregW, regwriteM=>regwriteM, regwriteW=>regwriteW, memtoregE=>memtoregE,
	forwardaE=>forwardaE, forwardbE=>forwardbE, stallF=>stallF, stallD=>stallD, flushE=>flushE);

	-- Próximo PC
	pcnext <= pcplus4; -- Próximo PC
	
	-- Banco de registradores
	reg_file: entity work.reg_bank port map(ck=>clk, rst=>rst, wreg=>regwriteW, AdRs=>rsD, AdRt=>rtD, adRD=>writeregW, RD=>resultW, R1=>srcaD, R2=>srcbD); --Chamando o banco
	
	-- Estagio FETCH
	stallDbar <= (not stallD);
	stallFbar <= (not stallF);				
	pcreg: entity work.reg generic map(width=> 32) port map(ck=>clk, rst=>rst, ce=>stallFbar, 	
	clear=>'0', eh_pc=>'1', D=>pcnext, Q=>pcF, eh_banco=>'0');
	pcplus: entity work.somador port map(a=>pcF, b=> "00000000000000000000000000000100", y=>pcplus4); -- Determinando o PC
	
	-- Estágio DECODE
	pcD1: entity work.reg generic map(width=>32) port map(ck=>clk, rst=>rst, ce=>stallDbar, clear=>'0', eh_pc=>'0', D=>pcF, Q=>pcD, eh_banco=>'0'); --Registradores do pipeline
	instrD1:  entity work.reg generic map(width=>32) port map(ck=>clk, rst=>rst, ce=>stallDbar, clear=>flushD, eh_pc=>'0', D=>instrF, Q=>instrD, eh_banco=>'0');
	signimmD <= (X"FFFF" & instrD(15 downto 0)) when instrD(15) = '1' else (X"0000" & instrD(15 downto 0)); -- Extensor de sinal
	opD <= instrD(31 downto 26);
	functD <= instrD(5 downto 0);
	rsD <= instrD(25 downto 21);
	rtD <= instrD(20 downto 16);
	rdD <= instrD(15 downto 11);
		
	-- Estágio EXEC
	pcE1: entity work.reg generic map(width=>32) port map(ck=>clk, rst=>rst, ce=>stallDbar, clear=>'0', eh_pc=>'0', D=>pcD, Q=>pcE, eh_banco=>'0'); --Registradores do pipeline
	instrE1:  entity work.reg generic map(width=>32) port map(ck=>clk, rst=>rst, ce=>stallDbar, clear=>flushD, eh_pc=>'0', D=>instrD, Q=>instrE, eh_banco=>'0');
	r1E: entity work.reg generic map(width=>32) port map(ck=>clk, rst=>rst, ce=>'1', clear=>flushE, eh_pc=>'0', D=>srcaD, Q=>srcaE, eh_banco=>'0'); 
	r2E: entity work.reg generic map(width=>32) port map(ck=>clk, rst=>rst, ce=>'1', clear=>flushE, eh_pc=>'0', D=>srcbD, Q=>srcbE, eh_banco=>'0');
	r3E: entity work.reg generic map(width=>32) port map(ck=>clk, rst=>rst, ce=>'1', clear=>flushE, eh_pc=>'0', D=>signimmD, Q=>signimmE, eh_banco=>'0');
	r4E: entity work.reg generic map(width=>5) port map(ck=>clk, rst=>rst, ce=>'1', clear=>flushE, eh_pc=>'0', D=>rsD, Q=>rsE, eh_banco=>'0');
	r5E: entity work.reg generic map(width=>5) port map(ck=>clk, rst=>rst, ce=>'1', clear=>flushE, eh_pc=>'0', D=>rtD, Q=>rtE, eh_banco=>'0');
	r6E: entity work.reg generic map(width=>5) port map(ck=>clk, rst=>rst, ce=>'1', clear=>flushE, eh_pc=>'0', D=>rdD, Q=>rdE, eh_banco=>'0');
	forwardaemux: entity work.mux3 generic map(width=>32) port map(d0=>srcaE, d1=>resultW, d2=>aluoutM, s=>forwardaE, y=>ALU_A); --Mux de forward A
	forwardbemux: entity work.mux3 generic map(width=>32) port map(d0=>srcbE, d1=>resultW, d2=>aluoutM, s=>forwardbE, y=>srcb2E); --Mux de forward B
	srcbmux: entity work.mux2 generic map(width=>32) port map(d0=>srcb2E, d1=>signimmE, s=>alusrcE, y=>ALU_B); --Source B da ALU (registrador ou imediato)
	alu1: entity work.alu port map(alu_op1=>ALU_A, alu_op2=>ALU_B, alu_outalu=>aluoutE, alu_op_alu=>alucontrolE);
	wrmux: entity work.mux2 generic map(width=>5) port map(d0=>rtE, d1=>rdE, s=>regdstE, y=>writeregE); --Mux para ver o registrador a ser escrito

	-- Estágio MEM
	instrM1:  entity work.reg generic map(width=>32) port map(ck=>clk, rst=>rst, ce=>stallDbar, clear=>flushD, eh_pc=>'0', D=>instrE, Q=>instrM, eh_banco=>'0'); --Registradores do pipeline
	pcM1: entity work.reg generic map(width=>32) port map(ck=>clk, rst=>rst, ce=>stallDbar, clear=>'0', eh_pc=>'0', D=>pcE, Q=>pcM, eh_banco=>'0');
	r1M: entity work.reg generic map(width=>32) port map(ck=>clk, rst=>rst, ce=>'1', clear=>'0', eh_pc=>'0', D=>srcb2E, Q=>writedataM, eh_banco=>'0'); 
	r2M: entity work.reg generic map(width=>32) port map(ck=>clk, rst=>rst, ce=>'1', clear=>'0', eh_pc=>'0', D=>aluoutE, Q=>aluoutM, eh_banco=>'0');
	r3M: entity work.reg generic map(width=>5) port map(ck=>clk, rst=>rst, ce=>'1', clear=>'0', eh_pc=>'0', D=>writeregE, Q=>writeregM, eh_banco=>'0');
	
	-- Estágio WB
	instrW1:  entity work.reg generic map(width=>32) port map(ck=>clk, rst=>rst, ce=>stallDbar, clear=>flushD, eh_pc=>'0', D=>instrM, Q=>instrW, eh_banco=>'0'); --Registradores do pipeline
	pcW1: entity work.reg generic map(width=>32) port map(ck=>clk, rst=>rst, ce=>stallDbar, clear=>'0', eh_pc=>'0', D=>pcM, Q=>pcW, eh_banco=>'0');
	r1W: entity work.reg generic map(width=>32) port map(ck=>clk, rst=>rst, ce=>'1', clear=>'0', eh_pc=>'0', D=>aluoutM, Q=>aluoutW, eh_banco=>'0'); 
	r2W: entity work.reg generic map(width=>32) port map(ck=>clk, rst=>rst, ce=>'1', clear=>'0', eh_pc=>'0', D=>readdataM, Q=>readdataW, eh_banco=>'0');
	r3W: entity work.reg generic map(width=>5) port map(ck=>clk, rst=>rst, ce=>'1', clear=>'0', eh_pc=>'0', D=>writeregM, Q=>writeregW, eh_banco=>'0');
	resmux: entity work.mux2 generic map(width=>32) port map(d0=>aluoutW, d1=>readdataW, s=>memtoregW, y=>resultW); --Mux para verificar se o resultado vem da ALU ou memória
	
end;

---------------------------------------------------------------------------

library IEEE; 
use IEEE.STD_LOGIC_1164.all;
use work.p_MI0.all;

entity controle is -- Unidade de controle
		port(clk, rst: in STD_LOGIC;
		opD, functD: in STD_LOGIC_VECTOR(5 downto 0);
		flushE: in STD_LOGIC;
		memtoregE, memtoregM: inout STD_LOGIC;
		memtoregW, memwriteM, ce: out STD_LOGIC;
		alusrcE: out STD_LOGIC;
		regdstE: out STD_LOGIC;
		regwriteE: inout STD_LOGIC;
		regwriteM, regwriteW: inout STD_LOGIC;
		alucontrolE: out STD_LOGIC_VECTOR(2 downto 0));
end;

architecture controle of controle is

	signal aluopD: STD_LOGIC_VECTOR(1 downto 0);
	signal memtoregD, memwriteD, alusrcD, ceD, ceE: STD_LOGIC;
	signal regdstD, regwriteD: STD_LOGIC;
	signal alucontrolD: STD_LOGIC_VECTOR(2 downto 0);
	signal memwriteE: STD_LOGIC;
	signal controlsD: STD_LOGIC_VECTOR(7 downto 0);
	
	-- sinais internos para uso nos registradores de pipeline
	signal d_regE: STD_LOGIC_VECTOR(8 downto 0);
	signal q_regE: STD_LOGIC_VECTOR(8 downto 0);
	signal d_regM: STD_LOGIC_VECTOR(3 downto 0);
	signal q_regM: STD_LOGIC_VECTOR(3 downto 0);
	signal d_regW: STD_LOGIC_VECTOR(1 downto 0);
	signal q_regW: STD_LOGIC_VECTOR(1 downto 0);
	
begin

------------------------- MAPEANDO OS SINAIS DE CONTROLE --------------------

	process(opD) begin --Decodificando o opcode e setando os controles
	case opD is
		when "000000" => controlsD <= "01100010"; -- Rtype
		when "100011" => controlsD <= "11011100"; -- LW
		when "101011" => controlsD <= "10010000"; -- SW
		when "001101" => controlsD <= "01011011"; -- ORI
		
		when others => controlsD <= "--------"; -- Instrução inválida
	end case;
	end process;
	
	ceD <= controlsD(7);
	regwriteD <= controlsD(6);
	regdstD <= controlsD(5);
	alusrcD <= controlsD(4);
	memwriteD <= controlsD(3);
	memtoregD <= controlsD(2);
	aluopD <= controlsD(1 downto 0);

------------------------- MAPEANDO A ALU --------------------

	process(aluopD, functD) begin -- Decodificando o aluop e setando o alucontrol
	case aluopD is
		when "00" => alucontrolD <= "010"; -- add 
		when "01" => alucontrolD <= "110"; -- sub 
		when "11" => alucontrolD <= "001"; -- or 
		when others => case functD is -- R-type
			when "100001" => alucontrolD <= "010"; -- add
			when "100011" => alucontrolD <= "110"; -- sub
			when "100100" => alucontrolD <= "011"; -- and
			when "100101" => alucontrolD <= "001"; -- or
			when "100110" => alucontrolD <= "101"; --XXOR
			when "100111" => alucontrolD <= "111"; --NNOR
			when others => alucontrolD <= "---"; -- ???
		end case;
	end case;
	end process;
	
---------- Registradores do pipeline para guardar os sinais de controle

	regE: entity work.reg generic map(width=>9) port map (ck=>clk, rst=>rst, ce=>'1', clear=>flushE, eh_pc=>'0', D=>d_regE, Q=>q_regE, eh_banco=>'0');
	regM: entity work.reg generic map(width=>4) port map(ck=>clk, rst=>rst, ce=>'1', clear=>'0', eh_pc=>'0', D=>d_regM, Q=>q_regM, eh_banco=>'0');
	regW: entity work.reg generic map(width=>2) port map(ck=>clk, rst=>rst, ce=>'1', clear=>'0', eh_pc=>'0', D=>d_regW, Q=>q_regW, eh_banco=>'0');
	d_regE <= ceD & memtoregD & memwriteD & alusrcD & regdstD & regwriteD & alucontrolD;
	ceE <= q_regE(8);
	memtoregE <= q_regE(7);
	memwriteE <= q_regE(6);
	alusrcE <= q_regE(5);
	regdstE <= q_regE(4);
	regwriteE <= q_regE(3);
	alucontrolE <= q_regE(2 downto 0);
	d_regM <= ceE & memtoregE & memwriteE & regwriteE;
	ce <= q_regM(3);
	memtoregM <= q_regM(2);
	memwriteM <= q_regM(1);
	regwriteM <= q_regM(0);
	d_regW <= memtoregM & regwriteM;
	memtoregW <= q_regW(1);
	regwriteW <= q_regW(0);
	
end;

----------------------------------------------------------------------------

library IEEE; 
use IEEE.STD_LOGIC_1164.all;
use work.p_MI0.all;

entity mips is -- Topo da hierarquia do processador
	port(clk, rst: in STD_LOGIC;
	pcF: inout reg32;
	instrF: in reg32;
	memwriteM: out STD_LOGIC;
	ce: out STD_LOGIC;
	aluoutM, writedataM: inout reg32;
	readdataM: in reg32);
end;

architecture mips of mips is

	component controle
		port(clk, rst: in STD_LOGIC;
		opD, functD: in STD_LOGIC_VECTOR(5 downto 0);
		flushE: in STD_LOGIC;
		memtoregE, memtoregM: inout STD_LOGIC;
		memtoregW, memwriteM, ce: out STD_LOGIC;
		alusrcE: out STD_LOGIC;
		regdstE: out STD_LOGIC;
		regwriteE: inout STD_LOGIC;
		regwriteM, regwriteW: inout STD_LOGIC;
		alucontrolE: out STD_LOGIC_VECTOR(2 downto 0));
		end component;
		
	component datapath
		port(clk, rst: in STD_LOGIC;
		memtoregE, memtoregM, memtoregW: in STD_LOGIC;
		alusrcE, regdstE: in STD_LOGIC;
		regwriteE, regwriteM, regwriteW: in STD_LOGIC;
		alucontrolE: in STD_LOGIC_VECTOR(2 downto 0);
		pcF: inout reg32;
		instrF: in reg32;
		aluoutM, writedataM: inout reg32;
		readdataM: in reg32;
		opD, functD: out STD_LOGIC_VECTOR(5 downto 0);
		flushE: inout STD_LOGIC);
	end component;

	signal opD, functD: STD_LOGIC_VECTOR(5 downto 0);
	signal regdstE, alusrcE, memtoregE, memtoregM, memtoregW, regwriteE, regwriteM, regwriteW: STD_LOGIC;
	signal alucontrolE: STD_LOGIC_VECTOR(2 downto 0);
	signal flushE: STD_LOGIC;
	
begin
	c: controle port map(clk, rst, opD, functD, flushE, memtoregE, memtoregM, memtoregW, memwriteM, ce, alusrcE, 
	regdstE, regwriteE, regwriteM, regwriteW, alucontrolE);
	dp: datapath port map(clk, rst, memtoregE, memtoregM, memtoregW, alusrcE, regdstE, regwriteE, regwriteM, regwriteW,
	alucontrolE, pcF, instrF, aluoutM, writedataM, readdataM, opD, functD, flushE);

end;

---------------------------------------------------------------
