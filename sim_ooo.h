#ifndef SIM_OO_H_
#define SIM_OO_H_

#include <stdio.h>
#include <stdbool.h>
#include <string>
#include <sstream>
#include <cstring>

using namespace std;

#define UNDEFINED 0xFFFFFFFF //constant used for initialization
#define NUM_GP_REGISTERS 32
#define NUM_FP_REGISTERS 32
#define NUM_OPCODES 28
#define NUM_STAGES 4

typedef enum {LW, SW, ADD, ADDI, SUB, SUBI, XOR, XORI, OR, ORI, AND, ANDI, MULT, DIV, BEQZ, BNEZ, BLTZ, BGTZ, BLEZ, BGEZ, JUMP, EOP, LWS, SWS, ADDS, SUBS, MULTS, DIVS} opcode_t;

typedef enum {INTEGER_RS, ADD_RS, MULT_RS, LOAD_B} res_station_t;

typedef enum {INTEGER, ADDER, MULTIPLIER, DIVIDER, MEMORY, NOTVALID} exe_unit_t;

typedef enum{ISSUE, EXECUTE, WRITE_RESULT, COMMIT, INVALID} stage_t;

typedef enum{INTEGER_RES,FLOAT_RES,NOTASSIGNED} res_type_t;


class sim_ooo{

	/* Add the data members required by your simulator's implementation here */

	//data memory - should be initialize to all 0xFF
	unsigned char *data_memory;

	//memory size in bytes
	unsigned base_add;
	unsigned data_memory_size;
	unsigned register_file[NUM_GP_REGISTERS];
	float float_reg_file[NUM_FP_REGISTERS];
	unsigned current_PC;
	unsigned head_ROB;
	unsigned current_R_int;
	unsigned current_R_mul;
	unsigned current_R_add;
	unsigned current_R_ld;
	unsigned current_B;
	unsigned check_branch;
	unsigned branch_cond;
	unsigned int_struc_hazard;
	unsigned load_done;
	unsigned add_struc_hazard;
	unsigned mul_struc_hazard;
	unsigned div_struc_hazard;
	unsigned issue_size;
	unsigned issue_success;
	unsigned num_cycles;
	unsigned load_complete;

public:

	/* Instantiates the simulator
          	Note: registers must be initialized to UNDEFINED value, and data memory to all 0xFF values
        */
	sim_ooo(unsigned mem_size, 		// size of data memory (in byte)
		unsigned rob_size, 		// number of ROB entries
                unsigned num_int_res_stations,	// number of integer reservation stations
                unsigned num_add_res_stations,	// number of ADD reservation stations
                unsigned num_mul_res_stations, 	// number of MULT/DIV reservation stations
                unsigned num_load_buffers,	// number of LOAD buffers
		unsigned issue_width=1		// issue width

        );

	//de-allocates the simulator
	~sim_ooo();

        // adds one or more execution units of a given type to the processor
        // - exec_unit: type of execution unit to be added
        // - latency: latency of the execution unit (in clock cycles)
        // - instances: number of execution units of this type to be added
        void init_exec_unit(exe_unit_t exec_unit, unsigned latency, unsigned instances=1);

	//loads the assembly program in file "filename" in instruction memory at the specified address
	void load_program(const char *filename, unsigned base_address=0x0);

	//runs the simulator for "cycles" clock cycles (run the program to completion if cycles=0)
	void run(unsigned cycles=0);

	//Issue stage of the Tomasulo's algorithm

	void clear_rob();

	void clear_res();

	void clear_reg();

	void issue(unsigned temp_PC);

	//Execute stage of the Tomasulo's algorithm
	void execute_ins();

	//Write result stage of the Tomasulo's algorithm
	void write_res();

	void commit_stage();

	//resets the state of the simulator
        /* Note:
	   - registers should be reset to UNDEFINED value
	   - data memory should be reset to all 0xFF values
	   - instruction window, reservation stations and rob should be cleaned
	*/
	void reset();

       //returns value of the specified integer general purpose register
        int get_int_register(unsigned reg);

        //set the value of the given integer general purpose register to "value"
        void set_int_register(unsigned reg, int value);

        //returns value of the specified floating point general purpose register
        float get_fp_register(unsigned reg);

        //set the value of the given floating point general purpose register to "value"
        void set_fp_register(unsigned reg, float value);

	// returns the index of the ROB entry that will write this integer register (UNDEFINED if the value of the register is not pending
	unsigned get_pending_int_register(unsigned reg);

	// returns the index of the ROB entry that will write this floating point register (UNDEFINED if the value of the register is not pending
	unsigned get_pending_fp_register(unsigned reg);

	//returns the IPC
	float get_IPC();

	//returns the number of instructions fully executed
	unsigned get_instructions_executed();

	//returns the number of clock cycles
	unsigned get_clock_cycles();

	//prints the content of the data memory within the specified address range
	void print_memory(unsigned start_address, unsigned end_address);

	// writes an integer value to data memory at the specified address (use little-endian format: https://en.wikipedia.org/wiki/Endianness)
	void write_memory(unsigned address, unsigned value);

	//prints the values of the registers
	void print_registers();

	//prints the status of processor excluding memory
	void print_status();

	// prints the content of the ROB
	void print_rob();

	//prints the content of the reservation stations
	void print_reservation_stations();

	//print the content of the instruction window
	void print_pending_instructions();

	//print the whole execution history
	void print_log();
};

#endif /*SIM_OOO_H_*/
