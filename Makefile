all:
	ghdl -a --ieee=synopsys -fexplicit MRstd.vhd 

	ghdl -a --ieee=synopsys -fexplicit MRstd_tb.vhd 

	ghdl -e --ieee=synopsys -fexplicit MRstd_tb

	./mrstd_tb --stop-time=1000ns --vcd=mrstd.vcd

	gtkwave mrstd.vcd signals.gtkw
clean:
	rm *.o
	
	rm mrstd.vcd

	rm mrstd_tb
	
	rm work-obj93.cf
