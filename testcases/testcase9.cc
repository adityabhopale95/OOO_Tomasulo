#include "sim_ooo.h"
#include <iostream>
#include <stdlib.h>
#include <sstream>

using namespace std;

/* Test case for pipelined simuator */ 
/* DO NOT MODIFY */

/* convert a float into an unsigned */
inline unsigned float2unsigned(float value){
        unsigned result;
        memcpy(&result, &value, sizeof value);
        return result;
}

/* convert an unsigned into a float */
inline float unsigned2float(unsigned value){
        float result;
        memcpy(&result, &value, sizeof value);
        return result;
}

int main(int argc, char **argv){

	// instantiates sim_ooo with a 1MB data memory
	sim_ooo *ooo = new sim_ooo(1024*1024,	//memory size 
				   6,           //rob size
				   3, 2, 2, 2,  //int, add, mult, load reservation stations
				   2); 		//issue width
			
	//initialize execution units
        ooo->init_exec_unit(INTEGER, 3, 2);
        ooo->init_exec_unit(ADDER, 3, 2);
        ooo->init_exec_unit(MULTIPLIER, 10, 1);
        ooo->init_exec_unit(DIVIDER, 40, 1);
        ooo->init_exec_unit(MEMORY, 5, 1);

	//loads program in instruction memory at address 0x00000000
	ooo->load_program("asm/sort.asm", 0x00000000);

	//initialize general purpose registers
	ooo->set_int_register(7, 0x80000000);

        //initialize data memory 
        ooo->write_memory(0xA000, float2unsigned(15.5));
        ooo->write_memory(0xA004, float2unsigned(3.1));
        ooo->write_memory(0xA008, float2unsigned(23.0));
        ooo->write_memory(0xA00C, float2unsigned(1.3));
        ooo->write_memory(0xA010, float2unsigned(4.4));
        ooo->write_memory(0xA014, float2unsigned(12.6));
        ooo->write_memory(0xA018, float2unsigned(0.0));
        ooo->write_memory(0xA01C, float2unsigned(-12.1));
        ooo->write_memory(0xA020, float2unsigned(30.2));
        ooo->write_memory(0xA024, float2unsigned(44.7));
        ooo->write_memory(0xA028, float2unsigned(41.5));
        ooo->write_memory(0xA02C, float2unsigned(-10.3));
	
	cout << "\nBEFORE PROGRAM EXECUTION..." << endl;
	cout << "======================================================================" << endl << endl;
	
	//prints the value of the memory and registers
	ooo->print_registers();
	ooo->print_memory(0xA000, 0xA030);
	ooo->print_memory(0xB000, 0xB030);

	// executes the program	
	cout << "\n*****************************" << endl;
	cout << "STARTING THE PROGRAM..." << endl;
	cout << "*****************************" << endl << endl;

	/* NO cycle-by-cycle execution for this test case
	cout << "First 30 clock cycles: inspecting the registers at each clock cycle..." << endl;
	cout << "======================================================================" << endl << endl;

	for (i=0; i<70; i++){
		cout << "CLOCK CYCLE #" << dec << i << endl;
		ooo->run(1);
		ooo->print_status();
		cout << endl;
	}
	*/

	// runs program to completion
	cout << "EXECUTING PROGRAM TO COMPLETION..." << endl << endl;
	ooo->run(); 

	cout << "PROGRAM TERMINATED\n";
	cout << "===================" << endl << endl;

	//prints the value of registers and data memory
	ooo->print_registers();
	ooo->print_memory(0xA000, 0xA030);
	ooo->print_memory(0xB000, 0xB030);
	cout << endl;

	//print the execution log
	ooo->print_log();
	
	cout << endl;

	// prints the number of instructions executed and IPC
	cout << "Instruction executed = " << dec << ooo->get_instructions_executed() << endl;
	cout << "Clock cycles = " << dec << ooo->get_clock_cycles() << endl;
	cout << "IPC = " << dec << ooo->get_IPC() << endl;
}
