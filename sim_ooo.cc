#include "sim_ooo.h"
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <cstring>
#include <string>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <map>

using namespace std;

struct every_line
{
	string branch_array;
	string op_code;
	string mand_operand;
	string remainder_operand1;
	string remainder_operand2;
	unsigned instr_address;
	exe_unit_t classify_exe;
};

struct ROB_struc{
  unsigned entry;
  unsigned rob_busy;
  unsigned ready;
  unsigned rob_pc;
	unsigned rob_temp_pc;
	unsigned if_branch_rob;
  stage_t state;
  unsigned rob_dest;
  unsigned rob_val;
	res_type_t dest_type;
};

struct RES_Stat_struc{
  string res_name;
  unsigned res_busy;
  unsigned res_pc;
	int Vj;
  int Vk;
  unsigned Qj;
  unsigned Qk;
  unsigned res_dest;
  unsigned res_A;
	unsigned op_num;
	unsigned if_branch;
	unsigned use_ex;
	exe_unit_t op;
};

struct REGISTER_Stat{
  unsigned ROB_num;
  unsigned reg_busy;
};

struct exe_struc{
  int Vj;
  int Vk;
  int Add;
	unsigned res_stat_num;
  int status;
};

struct pack_of_exec
{
	exe_unit_t exe_unit;
	unsigned latency_exe;
	unsigned instance_exe;
};

std::map<string, int> opcode_map;

every_line every_instruct;

ROB_struc ROB_tab = {1,0,0,UNDEFINED,UNDEFINED,UNDEFINED,INVALID,UNDEFINED,UNDEFINED,NOTASSIGNED};
RES_Stat_struc RES_Temp = {"",0,UNDEFINED,UNDEFINED,UNDEFINED,UNDEFINED,UNDEFINED,UNDEFINED,UNDEFINED,UNDEFINED,UNDEFINED,0,NOTVALID};
exe_struc TempReg = {UNDEFINED,UNDEFINED,UNDEFINED,UNDEFINED,UNDEFINED};
pack_of_exec execution_structure;
std::vector<REGISTER_Stat> int_reg_stat;
std::vector<REGISTER_Stat> float_reg_stat;

std::vector<pack_of_exec> create_exe_unit;

std::vector<every_line> instruction_memory;

std::vector<RES_Stat_struc> RES_Stat_Int;
std::vector<RES_Stat_struc> RES_Stat_Load;
std::vector<RES_Stat_struc> RES_Stat_Mul;
std::vector<RES_Stat_struc> RES_Stat_Add;
std::vector<int> int_result;
std::vector<int> add_result;
std::vector<int> mul_result;
std::vector<int> mem_result;
std::vector<ROB_struc> ROB_table;

std::vector<exe_struc> big_int;
std::vector<exe_struc> big_add;
std::vector<exe_struc> big_mul;
std::vector<exe_struc> big_div;
std::vector<exe_struc> big_mem;

std::vector<int> num_latency_int;
std::vector<int> num_latency_add;
std::vector<int> num_latency_mul;
std::vector<int> num_latency_div;
std::vector<int> num_latency_mem;


//ROB_table.push_back(ROB_tab);

//used for debugging purposes
static const char *stage_names[NUM_STAGES] = {"ISSUE", "EXE", "WR", "COMMIT"};
static const char *opcode_name[NUM_OPCODES] = {"LW", "SW", "ADD", "ADDI", "SUB", "SUBI", "XOR", "XORI", "OR", "ORI", "AND", "ANDI", "MULT", "DIV", "BEQZ", "BNEZ", "BLTZ", "BGTZ", "BLEZ", "BGEZ", "JUMP", "EOP", "LWS", "SWS", "ADDS", "SUBS", "MULTS", "DIVS"};
static const char *res_station_names[5]={"Int", "Add", "Mult", "Load"};

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

/* convert integer into array of unsigned char - little endian */
inline void unsigned2char(unsigned value, unsigned char *buffer){
        buffer[0] = value & 0xFF;
        buffer[1] = (value >> 8) & 0xFF;
        buffer[2] = (value >> 16) & 0xFF;
        buffer[3] = (value >> 24) & 0xFF;
}

/* convert array of char into integer - little endian */
inline unsigned char2unsigned(unsigned char *buffer){
       return buffer[0] + (buffer[1] << 8) + (buffer[2] << 16) + (buffer[3] << 24);
}

sim_ooo::sim_ooo(unsigned mem_size,
                unsigned rob_size,
                unsigned num_int_res_stations,
                unsigned num_add_res_stations,
                unsigned num_mul_res_stations,
                unsigned num_load_res_stations,
		unsigned max_issue){
	//memory
	data_memory_size = mem_size;
	data_memory = new unsigned char[data_memory_size];
  //fill here
	issue_size = max_issue;

	for (unsigned a = 0; a < NUM_OPCODES; a++){
			opcode_map[opcode_name[a]] = a;
	}

  for(unsigned i = 0; i < NUM_GP_REGISTERS; i++){     //initializing integer registers
		register_file[i] = UNDEFINED;
		//std::cout << "Register" << i << register_file[i] << '\n';
	}

	for(unsigned i = 0; i < NUM_FP_REGISTERS; i++){    //initializing floating point registers
		float_reg_file[i] = UNDEFINED;
		//std::cout << "Register" << i << float_reg_file[i] << '\n';
	}

  for(unsigned j = 0; j < data_memory_size; j++){   //initializing data memory
    data_memory[j] = 0xFF;
  }

  ROB_table.push_back(ROB_tab);
  for(unsigned a = 1; a < rob_size; a++){           //initializing ROB
    ROB_tab.entry = a+1;
    ROB_table.push_back(ROB_tab);
  }

	RES_Temp.res_name = "Int";
  for(unsigned lint = 0; lint < num_int_res_stations; lint++){
    RES_Stat_Int.push_back(RES_Temp);
		int_result.push_back(UNDEFINED);
  }

	RES_Temp.res_name = "Mult";
  for(unsigned lmul = 0; lmul < num_mul_res_stations; lmul++){
    RES_Stat_Mul.push_back(RES_Temp);
		mul_result.push_back(UNDEFINED);
  }

	RES_Temp.res_name = "Add";
  for(unsigned ladd = 0; ladd < num_add_res_stations; ladd++){
    RES_Stat_Add.push_back(RES_Temp);
		add_result.push_back(UNDEFINED);
  }

	RES_Temp.res_name = "Load";
  for(unsigned lload = 0; lload < num_load_res_stations; lload++){
    RES_Stat_Load.push_back(RES_Temp);
		mem_result.push_back(UNDEFINED);
  }

  int_reg_stat.resize(NUM_GP_REGISTERS);
  float_reg_stat.resize(NUM_FP_REGISTERS);

  for(unsigned i = 0; i < NUM_GP_REGISTERS; i++){
    int_reg_stat[i].reg_busy = 0;
    int_reg_stat[i].ROB_num = UNDEFINED;
    float_reg_stat[i].reg_busy = 0;
    float_reg_stat[i].ROB_num = UNDEFINED;
  }
	head_ROB = 0;
	issue_success = 1;
}

sim_ooo::~sim_ooo(){
}

void sim_ooo::init_exec_unit(exe_unit_t exec_unit, unsigned latency, unsigned instances){
  execution_structure.exe_unit = exec_unit;
	execution_structure.latency_exe = latency + 1;
	execution_structure.instance_exe = instances;

  create_exe_unit.push_back(execution_structure);
	switch (create_exe_unit.back().exe_unit){
    case INTEGER:
				num_latency_int.resize(create_exe_unit.back().instance_exe);
				for (unsigned j = 0; j < num_latency_int.size(); j++){
					num_latency_int[j] = create_exe_unit.back().latency_exe;
				}

				for (unsigned l = 0; l < create_exe_unit.back().instance_exe; l++){
					big_int.push_back(TempReg);
				}
				break;

		case ADDER:
  			num_latency_add.resize(create_exe_unit.back().instance_exe);
  			for (unsigned j = 0; j < num_latency_add.size(); j++){
  				num_latency_add[j] = create_exe_unit.back().latency_exe;
  		  }

				for (unsigned l = 0; l < create_exe_unit.back().instance_exe; l++){
				  big_add.push_back(TempReg);
				}
		break;

		case MULTIPLIER:
			num_latency_mul.resize(create_exe_unit.back().instance_exe);
			for (unsigned j = 0; j < num_latency_mul.size(); j++){
				num_latency_mul[j] = create_exe_unit.back().latency_exe;
			}

      for (unsigned l = 0; l < create_exe_unit.back().instance_exe; l++){
				big_mul.push_back(TempReg);
			}
			break;

		case DIVIDER:
			num_latency_div.resize(create_exe_unit.back().instance_exe);
			for (unsigned j = 0; j < num_latency_div.size(); j++){
				num_latency_div[j] = create_exe_unit.back().latency_exe;
			}

			for (unsigned l = 0; l < create_exe_unit.back().instance_exe; l++){
				big_div.push_back(TempReg);
			}
			break;

    case MEMORY:
  		num_latency_mem.resize(create_exe_unit.back().instance_exe);
  		for (unsigned j = 0; j < num_latency_mem.size(); j++){
  			num_latency_mem[j] = create_exe_unit.back().latency_exe;
  		}

    	for (unsigned l = 0; l < create_exe_unit.back().instance_exe; l++){
  			big_mem.push_back(TempReg);
  		}
  		break;

		default:
			break;
    }
}

void sim_ooo::load_program(const char *filename, unsigned base_address){
  char *instr_temp;
	char temp_str[25];
	unsigned instr_scan = 0;
	unsigned word_count = 0;
	unsigned check_branch;

	string line;


	ifstream fp(filename);
	if (!fp.is_open()){
	std::cerr << "/* error: open file */" << filename << "failed!" <<endl;
		exit(-1);
	}

	while(getline(fp,line)){
		word_count = 0;
		check_branch = 0;
		every_instruct.instr_address = base_address + instr_scan * 4;
//		std::cout << hex << every_instruct.instr_address << '\n';
		strcpy(temp_str,line.c_str());
		instr_temp = strtok(temp_str,"\t,: ");
		for(unsigned i = 0; i < NUM_OPCODES; i++) {
			if(strcmp(opcode_name[i],instr_temp) == 0)
			{
				check_branch = 0;
				break;
			}
			else
			{
				check_branch = 1;
			}
		}
		if(check_branch == 1)
		{
//			std::cout << "Branch Instruction" << '\n';
			every_instruct.branch_array = instr_temp;
//			std::cout << every_instruct.branch_array << endl;
		}
		else
		{
			every_instruct.branch_array = "";
			every_instruct.op_code = instr_temp;
//			std::cout << every_instruct[instr_scan].op_code << endl;
		}
		while (instr_temp != NULL){
			//printf("%s\n",instr_temp);
//			std::cout << word_count << '\n';
			instr_temp = strtok(NULL,"\t ");
			if(check_branch == 1){
				if(word_count == 0){
					every_instruct.op_code = instr_temp;
//					std::cout << every_instruct[instr_scan].op_code << '\n';
				}
				else if(word_count == 1)
				{
					if(instr_temp == NULL)
					{
						every_instruct.mand_operand = "";
					}
					else{
						every_instruct.mand_operand = instr_temp;
//					std::cout << every_instruct[instr_scan].mand_operand << '\n';
					}
				}
				else if(word_count == 2)
				{
					if(instr_temp == NULL){
						every_instruct.remainder_operand1 = "";
					}
					else{
						every_instruct.remainder_operand1 = instr_temp;
//						std::cout << every_instruct[instr_scan].remainder_operand1 << '\n';
					}
				}
				else if(word_count == 3)
				{
					if(instr_temp == NULL){
						every_instruct.remainder_operand2 = "";
					}
					else{
						every_instruct.remainder_operand2 = instr_temp;
//					std::cout << every_instruct[instr_scan].remainder_operand2 << '\n';
				}
				}
			}
			else
			{
				if(word_count == 0)
				{
					if(instr_temp == NULL){
						every_instruct.mand_operand = "";
					}
					else{
					every_instruct.mand_operand = instr_temp;
//					std::cout << every_instruct[instr_scan].mand_operand << '\n';
					}
				}
				else if(word_count == 1)
				{
					if(instr_temp == NULL){
					every_instruct.remainder_operand1 = "";
					}
					else{
						every_instruct.remainder_operand1 = instr_temp;
//						std::cout << every_instruct[instr_scan].remainder_operand1 << '\n';
					}
				}
				else if(word_count == 2)
					{
					if(instr_temp == NULL){
						every_instruct.remainder_operand2 = "";
					}
					else{
						every_instruct.remainder_operand2 = instr_temp;
//						std::cout << every_instruct[instr_scan].remainder_operand2 << '\n';
					}
				}
			}
			word_count++;
		}
		every_instruct.classify_exe = NOTVALID;
		instruction_memory.push_back(every_instruct);
		instr_scan++;
		every_instruct.mand_operand = "";
		every_instruct.remainder_operand1 = "";
		every_instruct.remainder_operand2 = "";
		every_instruct.op_code = "";
		every_instruct.branch_array = "";
	}
	for (unsigned i = 0; i < instruction_memory.size(); i++){
  	std::cout << "OP Code: " << instruction_memory[i].op_code << '\n';
  	std::cout << "Branch label: " << instruction_memory[i].branch_array << '\n';
  	std::cout << "Address: " << instruction_memory[i].instr_address << '\n';
  	std::cout << "MandOP: " << instruction_memory[i].mand_operand << '\n';
  	std::cout << "OP1: " << instruction_memory[i].remainder_operand1 << '\n';
  	std::cout << "OP2: " << instruction_memory[i].remainder_operand2 << '\n';
  }
	//std::cout << "Checking MAP value: " << dec << opcode_map["EOP"] << '\n';
	base_add = base_address;
	current_PC = base_add;
}

void sim_ooo::run(unsigned cycles){
	unsigned PC_index;
/*	for(int i = 0; i < issue_size; i++){
		PC_index = (current_PC - base_add)/4;
		issue(PC_index);
		if(issue_success == 1){
			current_PC = current_PC + 4;
		}
		else{
			break;
		}
	}*/
}

void sim_ooo::issue(unsigned temp_PC){
  char operands_needed[10];
  unsigned hold_val;
  unsigned hold_wr_val = UNDEFINED;
  unsigned hold_val1;
  unsigned hold_val2;
  unsigned hold_val1_float;
  unsigned hold_val2_float;
	unsigned rob_yes;
	unsigned res_yes_int;
	unsigned res_yes_mul;
	unsigned res_yes_load;
	unsigned res_yes_add;
	unsigned imm;

	for(unsigned i = 0; i < ROB_table.size(); i++){
		if(ROB_table[i].rob_busy!=1){
			current_B = i+1;
			rob_yes = 1;
			break;
		}
		else{
			rob_yes = 0;
		}
	}

	for(unsigned i = 0; i < RES_Stat_Int.size(); i++){
		if(RES_Stat_Int[i].res_busy!=1){
			current_R_int = i;
			res_yes_int = 1;
			break;
		}
		else{
			res_yes_int = 0;
		}
	}

	for(unsigned i = 0; i < RES_Stat_Mul.size(); i++){
		if(RES_Stat_Mul[i].res_busy!=1){
			current_R_mul = i;
			res_yes_mul = 1;
			break;
		}
		else{
			res_yes_mul = 0;
		}
	}

	for(unsigned i = 0; i < RES_Stat_Add.size(); i++){
		if(RES_Stat_Add[i].res_busy!=1){
			current_R_add = i;
			res_yes_add = 1;
			break;
		}
		else{
			res_yes_add = 0;
		}
	}

	for(unsigned i = 0; i < RES_Stat_Load.size(); i++){
		if(RES_Stat_Load[i].res_busy!=1){
			current_R_ld = i;
			res_yes_load = 1;
			break;
		}
		else{
			res_yes_load = 0;
		}
	}

  switch (opcode_map[instruction_memory[temp_PC].op_code]) {
    case XOR:
  	case ADD:
  	case SUB:
  	case OR:
  	case AND:
			if(res_yes_int == 1 && rob_yes == 1){
				issue_success = 1;
      	strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand1.c_str());
	      hold_val1 = atoi(operands_needed+1);
	      strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand2.c_str());
	      hold_val2 = atoi(operands_needed+1);
	      if(int_reg_stat[hold_val1].reg_busy){
	        head_ROB = int_reg_stat[hold_val1].ROB_num;
					if(ROB_table[head_ROB].ready){
						RES_Stat_Int[current_R_int].Vj = ROB_table[head_ROB].rob_val;
						RES_Stat_Int[current_R_int].Qj = 0;
					}
					else{
						RES_Stat_Int[current_R_int].Qj = head_ROB+1;
					}
	      }
				else{
					RES_Stat_Int[current_R_int].Vj = register_file[hold_val1];
					RES_Stat_Int[current_R_int].Qj = 0;
				}
				RES_Stat_Int[current_R_int].res_busy = 1;
				RES_Stat_Int[current_R_int].res_dest = current_B;
				ROB_table[current_B].rob_busy = 1;
				ROB_table[current_B].rob_pc = temp_PC*4 + base_add;
				ROB_table[current_B].rob_temp_pc = temp_PC;
				ROB_table[current_B].state = ISSUE;
				if(int_reg_stat[hold_val2].reg_busy){
					head_ROB = int_reg_stat[hold_val2].ROB_num;
					if(ROB_table[head_ROB].ready){
						RES_Stat_Int[current_R_int].Vk = ROB_table[head_ROB].rob_val;
						RES_Stat_Int[current_R_int].Qk = 0;
					}
					else{
						RES_Stat_Int[current_R_int].Qk = head_ROB;
					}
				}
				else{
					RES_Stat_Int[current_R_int].Vk = register_file[hold_val2];
					RES_Stat_Int[current_R_int].Qk = 0;
				}
				strcpy(operands_needed,instruction_memory[temp_PC].mand_operand.c_str());
	      hold_val = atoi(operands_needed+1);
				int_reg_stat[hold_val].ROB_num = current_B;
				int_reg_stat[hold_val].reg_busy = 1;
				ROB_table[current_B].rob_dest = hold_val;
				ROB_table[current_B].dest_type = INTEGER_RES;
				RES_Stat_Int[current_R_int].op = INTEGER;
				RES_Stat_Int[current_R_int].op_num = temp_PC;
				RES_Stat_Int[current_R_int].res_pc = temp_PC*4 + base_add;
			}

			break;

		case MULT:
			if(res_yes_mul == 1 && rob_yes == 1){
				issue_success = 1;
				strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand1.c_str());
				hold_val1 = atoi(operands_needed+1);
				strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand2.c_str());
				hold_val2 = atoi(operands_needed+1);
				if(int_reg_stat[hold_val1].reg_busy){
					head_ROB = int_reg_stat[hold_val1].ROB_num;
					if(ROB_table[head_ROB].ready){
						RES_Stat_Mul[current_R_mul].Vj = ROB_table[head_ROB].rob_val;
						RES_Stat_Mul[current_R_mul].Qj = 0;
					}
					else{
						RES_Stat_Mul[current_R_mul].Qj = head_ROB;
					}
				}
				else{
					RES_Stat_Mul[current_R_mul].Vj = register_file[hold_val1];
					RES_Stat_Mul[current_R_mul].Qj = 0;
				}
				RES_Stat_Mul[current_R_mul].res_busy = 1;
				RES_Stat_Mul[current_R_mul].res_dest = current_B;
				ROB_table[current_B].rob_pc = temp_PC*4 + base_add;
				ROB_table[current_B].rob_temp_pc = temp_PC;
				ROB_table[current_B].state = ISSUE;

				if(int_reg_stat[hold_val2].reg_busy){
					head_ROB = int_reg_stat[hold_val2].ROB_num;
					if(ROB_table[head_ROB].ready){
						RES_Stat_Mul[current_R_mul].Vk = ROB_table[head_ROB].rob_val;
						RES_Stat_Mul[current_R_mul].Qk = 0;
					}
					else{
						RES_Stat_Mul[current_R_mul].Qk = head_ROB;
					}
				}
				else{
					RES_Stat_Mul[current_R_mul].Vk = register_file[hold_val2];
					RES_Stat_Mul[current_R_mul].Qk = 0;
				}
				strcpy(operands_needed,instruction_memory[temp_PC].mand_operand.c_str());
				hold_val = atoi(operands_needed+1);
				int_reg_stat[hold_val].ROB_num = current_B;
				int_reg_stat[hold_val].reg_busy = 1;
				ROB_table[current_B].rob_dest = hold_val;
				ROB_table[current_B].dest_type = INTEGER_RES;
				RES_Stat_Mul[current_R_mul].op = MULTIPLIER;
				RES_Stat_Mul[current_R_mul].op_num = temp_PC;
				RES_Stat_Mul[current_R_mul].res_pc = temp_PC*4 + base_add;
			}

			break;

		case DIV:
			if(res_yes_mul == 1 && rob_yes == 1){
				issue_success = 1;
				strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand1.c_str());
				hold_val1 = atoi(operands_needed+1);
				strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand2.c_str());
				hold_val2 = atoi(operands_needed+1);
				if(int_reg_stat[hold_val1].reg_busy){
					head_ROB = int_reg_stat[hold_val1].ROB_num;
					if(ROB_table[head_ROB].ready){
						RES_Stat_Mul[current_R_mul].Vj = ROB_table[head_ROB].rob_val;
						RES_Stat_Mul[current_R_mul].Qj = 0;
					}
					else{
						RES_Stat_Mul[current_R_mul].Qj = head_ROB;
					}
				}
				else{
					RES_Stat_Mul[current_R_mul].Vj = register_file[hold_val1];
					RES_Stat_Mul[current_R_mul].Qj = 0;
				}
				RES_Stat_Mul[current_R_mul].res_busy = 1;
				RES_Stat_Mul[current_R_mul].res_dest = current_B;
				ROB_table[current_B].rob_temp_pc = temp_PC;
				ROB_table[current_B].rob_pc = temp_PC*4 + base_add;
				ROB_table[current_B].state = ISSUE;

				if(int_reg_stat[hold_val2].reg_busy){
					head_ROB = int_reg_stat[hold_val2].ROB_num;
					if(ROB_table[head_ROB].ready){
						RES_Stat_Mul[current_R_mul].Vk = ROB_table[head_ROB].rob_val;
						RES_Stat_Mul[current_R_mul].Qk = 0;
					}
					else{
						RES_Stat_Mul[current_R_mul].Qk = head_ROB;
					}
				}
				else{
					RES_Stat_Mul[current_R_mul].Vk = register_file[hold_val2];
					RES_Stat_Mul[current_R_mul].Qk = 0;
				}
				strcpy(operands_needed,instruction_memory[temp_PC].mand_operand.c_str());
				hold_val = atoi(operands_needed+1);
				int_reg_stat[hold_val].ROB_num = current_B;
				int_reg_stat[hold_val].reg_busy = 1;
				ROB_table[current_B].rob_dest = hold_val;
				ROB_table[current_B].dest_type = INTEGER_RES;
				RES_Stat_Mul[current_R_mul].op = DIVIDER;
				RES_Stat_Mul[current_R_mul].op_num = temp_PC;
				RES_Stat_Mul[current_R_mul].res_pc = temp_PC*4 + base_add;
			}
			else{
				issue_success = 0;
			}

			break;

		case XORI:
		case ADDI:
		case SUBI:
		case ORI:
		case ANDI:
			char *endptr;
			if(res_yes_int == 1 && rob_yes == 1){
				issue_success = 1;
				strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand1.c_str());
	      hold_val1 = atoi(operands_needed+1);
	      if(int_reg_stat[hold_val1].reg_busy){
	        head_ROB = int_reg_stat[hold_val1].ROB_num;
					if(ROB_table[head_ROB].ready){
						RES_Stat_Int[current_R_int].Vj = ROB_table[head_ROB].rob_val;
						RES_Stat_Int[current_R_int].Qj = 0;
					}
					else{
						RES_Stat_Int[current_R_int].Qj = head_ROB;
					}
	      }
				else{
					RES_Stat_Int[current_R_int].Vj = register_file[hold_val1];
					RES_Stat_Int[current_R_int].Qj = 0;
				}
				RES_Stat_Int[current_R_int].res_busy = 1;
				RES_Stat_Int[current_R_int].res_dest = current_B;
				ROB_table[current_B].rob_pc = temp_PC*4 + base_add;
				ROB_table[current_B].rob_temp_pc = temp_PC;
				ROB_table[current_B].state = ISSUE;

				strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand2.c_str());
				imm = strtoul(operands_needed, &endptr, 0);
				RES_Stat_Int[current_R_int].res_A = imm;
				RES_Stat_Int[current_R_int].Qk = 0;

				strcpy(operands_needed,instruction_memory[temp_PC].mand_operand.c_str());
				hold_val = atoi(operands_needed+1);
				int_reg_stat[hold_val].ROB_num = current_B;
				int_reg_stat[hold_val].reg_busy = 1;
				ROB_table[current_B].rob_dest = hold_val;
				ROB_table[current_B].dest_type = INTEGER_RES;
				RES_Stat_Int[current_R_int].op = INTEGER;
				RES_Stat_Int[current_R_int].op_num = temp_PC;
				RES_Stat_Int[current_R_int].res_pc = temp_PC*4 + base_add;
			}
			else{
				issue_success = 0;
			}

			break;

		case ADDS:
		case SUBS:
			if(res_yes_add == 1 && rob_yes == 1){
				issue_success = 1;
				strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand1.c_str());
				hold_val1 = atoi(operands_needed+1);
				strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand2.c_str());
	      hold_val2 = atoi(operands_needed+1);
	      if(float_reg_stat[hold_val1].reg_busy){
	        head_ROB = float_reg_stat[hold_val1].ROB_num;
					if(ROB_table[head_ROB].ready){
						RES_Stat_Add[current_R_add].Vj = ROB_table[head_ROB].rob_val;
						RES_Stat_Add[current_R_add].Qj = 0;
					}
					else{
						RES_Stat_Add[current_R_add].Qj = head_ROB;
					}
	      }
				else{
					RES_Stat_Add[current_R_add].Vj = float2unsigned(float(float_reg_file[hold_val1]));
					RES_Stat_Add[current_R_add].Qj = 0;
				}
				RES_Stat_Add[current_R_add].res_busy = 1;
				RES_Stat_Add[current_R_add].res_dest = current_B;
				ROB_table[current_B].rob_pc = temp_PC*4 + base_add;
				ROB_table[current_B].rob_temp_pc = temp_PC;
				ROB_table[current_B].state = ISSUE;

				if(float_reg_stat[hold_val2].reg_busy){
					head_ROB = float_reg_stat[hold_val2].ROB_num;
					if(ROB_table[head_ROB].ready){
						RES_Stat_Add[current_R_add].Vk = ROB_table[head_ROB].rob_val;
						RES_Stat_Add[current_R_add].Qk = 0;
					}
					else{
						RES_Stat_Add[current_R_add].Qk = head_ROB;
					}
				}
				else{
					RES_Stat_Add[current_R_add].Vk = float2unsigned(float(float_reg_file[hold_val2]));
					RES_Stat_Add[current_R_add].Qk = 0;
				}

				strcpy(operands_needed,instruction_memory[temp_PC].mand_operand.c_str());
				hold_val = atoi(operands_needed+1);
				float_reg_stat[hold_val].ROB_num = current_B;
				float_reg_stat[hold_val].reg_busy = 1;
				ROB_table[current_B].rob_dest = hold_val;
				ROB_table[current_B].dest_type = FLOAT_RES;
				RES_Stat_Add[current_R_add].op = ADDER;
				RES_Stat_Add[current_R_add].op_num = temp_PC;
				RES_Stat_Add[current_R_add].res_pc = temp_PC*4 + base_add;
			}
			else{
				issue_success = 0;
			}
			break;

		case MULTS:
			if(res_yes_mul == 1 && rob_yes == 1){
				strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand1.c_str());
				hold_val1 = atoi(operands_needed+1);
				strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand2.c_str());
				hold_val2 = atoi(operands_needed+1);
				if(float_reg_stat[hold_val1].reg_busy){
					head_ROB = float_reg_stat[hold_val1].ROB_num;
					if(ROB_table[head_ROB].ready){
						RES_Stat_Mul[current_R_mul].Vj = ROB_table[head_ROB].rob_val;
						RES_Stat_Mul[current_R_mul].Qj = 0;
					}
					else{
						RES_Stat_Mul[current_R_mul].Qj = head_ROB;
					}
				}
				else{
					RES_Stat_Mul[current_R_mul].Vj = float2unsigned(float(float_reg_file[hold_val1]));
					RES_Stat_Mul[current_R_mul].Qj = 0;
				}
				RES_Stat_Mul[current_R_mul].res_busy = 1;
				RES_Stat_Mul[current_R_mul].res_dest = current_B;
				ROB_table[current_B].rob_pc = temp_PC*4 + base_add;
				ROB_table[current_B].rob_temp_pc = temp_PC;
				ROB_table[current_B].state = ISSUE;

				if(float_reg_stat[hold_val2].reg_busy){
					head_ROB = float_reg_stat[hold_val2].ROB_num;
					if(ROB_table[head_ROB].ready){
						RES_Stat_Mul[current_R_mul].Vk = ROB_table[head_ROB].rob_val;
						RES_Stat_Mul[current_R_mul].Qk = 0;
					}
					else{
						RES_Stat_Mul[current_R_mul].Qk = head_ROB;
					}
				}
				else{
					RES_Stat_Mul[current_R_mul].Vk = float2unsigned(float(float_reg_file[hold_val2]));
					RES_Stat_Mul[current_R_mul].Qk = 0;
				}
				strcpy(operands_needed,instruction_memory[temp_PC].mand_operand.c_str());
				hold_val = atoi(operands_needed+1);
				float_reg_stat[hold_val].ROB_num = current_B;
				float_reg_stat[hold_val].reg_busy = 1;
				ROB_table[current_B].rob_dest = hold_val;
				ROB_table[current_B].dest_type = FLOAT_RES;
				RES_Stat_Mul[current_R_mul].op = MULTIPLIER;
				RES_Stat_Mul[current_R_mul].op_num = temp_PC;
				RES_Stat_Mul[current_R_mul].res_pc = temp_PC*4 + base_add;
			}
			else{
				issue_success = 0;
			}
			break;

		case DIVS:
			if(res_yes_mul == 1 && rob_yes == 1){
				strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand1.c_str());
				hold_val1 = atoi(operands_needed+1);
				strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand2.c_str());
				hold_val2 = atoi(operands_needed+1);
				if(float_reg_stat[hold_val1].reg_busy){
					head_ROB = float_reg_stat[hold_val1].ROB_num;
					if(ROB_table[head_ROB].ready){
						RES_Stat_Mul[current_R_mul].Vj = ROB_table[head_ROB].rob_val;
						RES_Stat_Mul[current_R_mul].Qj = 0;
					}
					else{
						RES_Stat_Mul[current_R_mul].Qj = head_ROB;
					}
				}
				else{
					RES_Stat_Mul[current_R_mul].Vj = float2unsigned(float(float_reg_file[hold_val1]));
					RES_Stat_Mul[current_R_mul].Qj = 0;
				}
				RES_Stat_Mul[current_R_mul].res_busy = 1;
				RES_Stat_Mul[current_R_mul].res_dest = current_B;
				ROB_table[current_B].rob_pc = temp_PC*4 + base_add;
				ROB_table[current_B].rob_temp_pc = temp_PC;
				ROB_table[current_B].state = ISSUE;

				if(float_reg_stat[hold_val2].reg_busy){
					head_ROB = float_reg_stat[hold_val2].ROB_num;
					if(ROB_table[head_ROB].ready){
						RES_Stat_Mul[current_R_mul].Vk = ROB_table[head_ROB].rob_val;
						RES_Stat_Mul[current_R_mul].Qk = 0;
					}
					else{
						RES_Stat_Mul[current_R_mul].Qk = head_ROB;
					}
				}
				else{
					RES_Stat_Mul[current_R_mul].Vk = float2unsigned(float(float_reg_file[hold_val2]));
					RES_Stat_Mul[current_R_mul].Qk = 0;
				}
				strcpy(operands_needed,instruction_memory[temp_PC].mand_operand.c_str());
				hold_val = atoi(operands_needed+1);
				float_reg_stat[hold_val].ROB_num = current_B;
				float_reg_stat[hold_val].reg_busy = 1;
				ROB_table[current_B].rob_dest = hold_val;
				ROB_table[current_B].dest_type = FLOAT_RES;
				RES_Stat_Mul[current_R_mul].op = MULTIPLIER;
				RES_Stat_Mul[current_R_mul].op_num = temp_PC;
				RES_Stat_Mul[current_R_mul].res_pc = temp_PC*4 + base_add;
			}
			else{
				issue_success = 0;
			}
			break;

		case BEQZ:
		case BNEZ:
		case BLTZ:
		case BGTZ:
		case BLEZ:
		case BGEZ:
			if(res_yes_int == 1 && rob_yes == 1){
				strcpy(operands_needed,instruction_memory[temp_PC].mand_operand.c_str());
				hold_val1 = atoi(operands_needed+1);
				if(int_reg_stat[hold_val1].reg_busy){
	        head_ROB = int_reg_stat[hold_val1].ROB_num;
					if(ROB_table[head_ROB].ready){
						RES_Stat_Int[current_R_int].Vj = ROB_table[head_ROB].rob_val;
						RES_Stat_Int[current_R_int].Qj = 0;
					}
					else{
						RES_Stat_Int[current_R_int].Qj = head_ROB;
					}
	      }
				else{
					RES_Stat_Int[current_R_int].Vj = register_file[hold_val1];
					RES_Stat_Int[current_R_int].Qj = 0;
				}
				RES_Stat_Int[current_R_int].res_busy = 1;
				RES_Stat_Int[current_R_int].res_dest = current_B;
				ROB_table[current_B].rob_pc = temp_PC*4 + base_add;
				ROB_table[current_B].rob_temp_pc = temp_PC;
				ROB_table[current_B].state = ISSUE;

				for(unsigned i = 0; i < instruction_memory.size(); i++){
					if(instruction_memory[temp_PC].remainder_operand1 == instruction_memory[i].branch_array){
						imm = instruction_memory[i].instr_address;
						check_branch = 1;
						RES_Stat_Int[current_R_int].res_A = imm;
					}
					else{
						check_branch = 0;
					}
				}
				RES_Stat_Int[current_R_int].op = INTEGER;
				RES_Stat_Int[current_R_int].op_num = temp_PC;
				RES_Stat_Int[current_R_int].res_pc = temp_PC*4 + base_add;
			}
			else{
				issue_success = 0;
			}

			break;

		case LW:
			unsigned hold_val_imm;
			char *pch;
			if(res_yes_load == 1 && rob_yes == 1){
				strcpy(operands_needed, instruction_memory[temp_PC].remainder_operand1.c_str());
				pch = strtok(operands_needed, " ( ");
				hold_val_imm = atoi(pch);

				RES_Stat_Load[current_R_ld].res_A = hold_val_imm;

				while(pch != NULL)
				{
					hold_val1 = atoi(pch);
					pch = strtok(NULL," R,( ");
				}

				if(int_reg_stat[hold_val1].reg_busy){
	        head_ROB = int_reg_stat[hold_val1].ROB_num;
					if(ROB_table[head_ROB].ready){
						RES_Stat_Load[current_R_ld].Vj = ROB_table[head_ROB].rob_val;
						RES_Stat_Load[current_R_ld].Qj = 0;
					}
					else{
						RES_Stat_Load[current_R_ld].Qj = head_ROB;
					}
	      }
				else{
					RES_Stat_Load[current_R_ld].Vj = register_file[hold_val1];
					RES_Stat_Load[current_R_ld].Qj = 0;
				}
				RES_Stat_Load[current_R_ld].res_busy = 1;
				RES_Stat_Load[current_R_ld].res_dest = current_B;
				ROB_table[current_B].rob_pc = temp_PC*4 + base_add;
				ROB_table[current_B].rob_temp_pc = temp_PC;
				ROB_table[current_B].state = ISSUE;

				strcpy(operands_needed, instruction_memory[temp_PC].mand_operand.c_str());
				hold_val = atoi(operands_needed+1);

				int_reg_stat[hold_val].ROB_num = current_B;
				int_reg_stat[hold_val].reg_busy = 1;
				ROB_table[current_B].rob_dest = hold_val;
				ROB_table[current_B].dest_type = INTEGER_RES;
				RES_Stat_Load[current_R_ld].op = MEMORY;
				RES_Stat_Load[current_R_ld].op_num = temp_PC;
				RES_Stat_Load[current_R_ld].res_pc = temp_PC*4 + base_add;
			}
			else{
				issue_success = 0;
			}
			break;

		case LWS:
			unsigned hold_val_imm1;
			char *pch1;
			if(res_yes_load == 1 && rob_yes == 1){
				strcpy(operands_needed, instruction_memory[temp_PC].remainder_operand1.c_str());
				pch1 = strtok(operands_needed, " ( ");
				hold_val_imm1 = atoi(pch1);

				RES_Stat_Load[current_R_ld].res_A = hold_val_imm1;

				while(pch1 != NULL)
				{
					hold_val1 = atoi(pch1);
					pch1 = strtok(NULL," R,( ");
				}
				if(int_reg_stat[hold_val1].reg_busy){
	        head_ROB = int_reg_stat[hold_val1].ROB_num;
					if(ROB_table[head_ROB].ready){
						RES_Stat_Load[current_R_ld].Vj = ROB_table[head_ROB].rob_val;
						RES_Stat_Load[current_R_ld].Qj = 0;
					}
					else{
						RES_Stat_Load[current_R_ld].Qj = head_ROB;
					}
	      }

				else{
					RES_Stat_Load[current_R_ld].Vj = register_file[hold_val1];
					RES_Stat_Load[current_R_ld].Qj = 0;
				}
				RES_Stat_Load[current_R_ld].res_busy = 1;
				RES_Stat_Load[current_R_ld].res_dest = current_B;
				ROB_table[current_B].rob_pc = temp_PC*4 + base_add;
				ROB_table[current_B].rob_temp_pc = temp_PC;
				ROB_table[current_B].state = ISSUE;

				strcpy(operands_needed, instruction_memory[temp_PC].mand_operand.c_str());
				hold_val = atoi(operands_needed+1);

				float_reg_stat[hold_val].ROB_num = current_B;
				float_reg_stat[hold_val].reg_busy = 1;
				ROB_table[current_B].rob_dest = hold_val;
				ROB_table[current_B].dest_type = FLOAT_RES;
				RES_Stat_Load[current_R_ld].op = MEMORY;
				RES_Stat_Load[current_R_ld].op_num = temp_PC;
				RES_Stat_Load[current_R_ld].res_pc = temp_PC*4 + base_add;
			}
			else{
				issue_success = 0;
			}
			break;

		case SW:
			unsigned hold_val_imm2;
			char *pch2;
			if(res_yes_load == 1 && rob_yes == 1){
				strcpy(operands_needed, instruction_memory[temp_PC].remainder_operand1.c_str());
				pch2 = strtok(operands_needed, " ( ");
				hold_val_imm2 = atoi(pch2);

				RES_Stat_Load[current_R_ld].res_A = hold_val_imm2;

				while(pch2 != NULL)
				{
					hold_val1 = atoi(pch2);
					pch2 = strtok(NULL," R,( ");
				}

				if(int_reg_stat[hold_val1].reg_busy){
	        head_ROB = int_reg_stat[hold_val1].ROB_num;
					if(ROB_table[head_ROB].ready){
						RES_Stat_Load[current_R_ld].Vj = ROB_table[head_ROB].rob_val;
						RES_Stat_Load[current_R_ld].Qj = 0;
					}
					else{
						RES_Stat_Load[current_R_ld].Qj = head_ROB;
					}
	      }
				else{
					RES_Stat_Load[current_R_ld].Vj = register_file[hold_val1];
					RES_Stat_Load[current_R_ld].Qj = 0;
				}
				RES_Stat_Load[current_R_ld].res_busy = 1;
				RES_Stat_Load[current_R_ld].res_dest = current_B;
				ROB_table[current_B].rob_pc = temp_PC*4 + base_add;
				ROB_table[current_B].rob_temp_pc = temp_PC;
				ROB_table[current_B].state = ISSUE;

				strcpy(operands_needed, instruction_memory[temp_PC].mand_operand.c_str());
				hold_val2 = atoi(operands_needed+1);
				if(int_reg_stat[hold_val2].reg_busy){
					head_ROB = int_reg_stat[hold_val2].ROB_num;
					if(ROB_table[head_ROB].ready){
						RES_Stat_Load[current_R_ld].Vk = ROB_table[head_ROB].rob_val;
						RES_Stat_Load[current_R_ld].Qk = 0;
					}
					else{
						RES_Stat_Load[current_R_ld].Qk = head_ROB;
					}
				}
				else{
					RES_Stat_Load[current_R_ld].Vk = register_file[hold_val2];
					RES_Stat_Load[current_R_ld].Qk = 0;
				}
				RES_Stat_Load[current_R_ld].op = MEMORY;
				RES_Stat_Load[current_R_ld].op_num = temp_PC;
				RES_Stat_Load[current_R_ld].res_pc = temp_PC*4 + base_add;
			}
			else{
				issue_success = 0;
			}
			break;

		case SWS:
			unsigned hold_val_imm3;
			char *pch3;

			if(res_yes_load == 1 && rob_yes == 1){
				strcpy(operands_needed, instruction_memory[temp_PC].remainder_operand1.c_str());
				pch3 = strtok(operands_needed, " ( ");
				hold_val_imm3 = atoi(pch3);

				RES_Stat_Load[current_R_ld].res_A = hold_val_imm3;

				while(pch3 != NULL)
				{
					hold_val1 = atoi(pch3);
					pch3 = strtok(NULL," R,( ");
				}

				if(int_reg_stat[hold_val1].reg_busy){
	        head_ROB = int_reg_stat[hold_val1].ROB_num;
					if(ROB_table[head_ROB].ready){
						RES_Stat_Load[current_R_ld].Vj = ROB_table[head_ROB].rob_val;
						RES_Stat_Load[current_R_ld].Qj = 0;
					}
					else{
						RES_Stat_Load[current_R_ld].Qj = head_ROB;
					}
	      }
				else{
					RES_Stat_Load[current_R_ld].Vj = register_file[hold_val1];
					RES_Stat_Load[current_R_ld].Qj = 0;
				}
				RES_Stat_Load[current_R_ld].res_busy = 1;
				RES_Stat_Load[current_R_ld].res_dest = current_B;
				ROB_table[current_B].rob_pc = temp_PC*4 + base_add;
				ROB_table[current_B].rob_temp_pc = temp_PC;
				ROB_table[current_B].state = ISSUE;

				strcpy(operands_needed, instruction_memory[temp_PC].mand_operand.c_str());
				hold_val2 = atoi(operands_needed+1);
				if(float_reg_stat[hold_val2].reg_busy){
					head_ROB = float_reg_stat[hold_val2].ROB_num;
					if(ROB_table[head_ROB].ready){
						RES_Stat_Load[current_R_ld].Vk = ROB_table[head_ROB].rob_val;
						RES_Stat_Load[current_R_ld].Qk = 0;
					}
					else{
						RES_Stat_Load[current_R_ld].Qk = head_ROB;
					}
				}
				else{
					RES_Stat_Load[current_R_ld].Vk = float_reg_file[hold_val2];
					RES_Stat_Load[current_R_ld].Qk = 0;
				}
				RES_Stat_Load[current_R_ld].op = MEMORY;
				RES_Stat_Load[current_R_ld].op_num = temp_PC*4 + base_add;
				RES_Stat_Load[current_R_ld].res_pc = temp_PC;
			}
			else{
				issue_success = 0;
			}
  }
}

void sim_ooo::execute_ins(){
	unsigned hold_int;
	unsigned hold_add;
	unsigned hold_mul;
	unsigned hold_div;
	float temp_adds;
	float temp_divs;
	float temp_muls;

	for(unsigned i = 0 ; i < RES_Stat_Int.size(); i++){
		if(RES_Stat_Int[i].res_busy == 1){
			if((RES_Stat_Int[i].Qj == 0) && (RES_Stat_Int[i].Qk == 0)){
				switch (RES_Stat_Int[i].op) {
					case INTEGER:
						for(unsigned sel_int_unit = 0; sel_int_unit < big_int.size(); sel_int_unit++){
							if((big_int[sel_int_unit].status == 1) && (ROB_table[RES_Stat_Int[i].res_dest].state == ISSUE)){
								ROB_table[RES_Stat_Int[i].res_dest].state = EXECUTE;
								RES_Stat_Int[i].use_ex = sel_int_unit;
								big_int[sel_int_unit].status = 0;
								big_int[sel_int_unit].res_stat_num = i;
								//num_latency_int[sel_int_unit]--;
								break;
							}
						}
						break;

					default:
						break;
				}
			}
		}
	}

	for(unsigned sel_int_unit = 0; sel_int_unit < big_int.size(); sel_int_unit++){
		if(big_int[sel_int_unit].status == 0){
			if(num_latency_int[sel_int_unit] > 1){
				num_latency_int[sel_int_unit]--;
			}
			else{
				switch(opcode_map[instruction_memory[RES_Stat_Int[big_int[sel_int_unit].res_stat_num].op_num].op_code]){
				case AND:
						int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vj & RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vk;
					break;

				case ADD:
					int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vj + RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vk;
					break;

				case OR:
					int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vj | RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vk;
					break;

				case XOR:
					int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vj ^ RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vk;
					break;

				case SUB:
					int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vj - RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vk;
					break;

				case ADDI:
					int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vj + RES_Stat_Int[big_int[sel_int_unit].res_stat_num].res_A;
					break;

				case SUBI:
					int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vj - RES_Stat_Int[big_int[sel_int_unit].res_stat_num].res_A;
					break;

				case XORI:
					int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vj ^ RES_Stat_Int[big_int[sel_int_unit].res_stat_num].res_A;
					break;

				case ORI:
					int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vj | RES_Stat_Int[big_int[sel_int_unit].res_stat_num].res_A;
					break;

				case ANDI:
					int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vj & RES_Stat_Int[big_int[sel_int_unit].res_stat_num].res_A;
					break;

				case BEQZ:
					if(RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vj == 0){
						RES_Stat_Int[big_int[sel_int_unit].res_stat_num].if_branch = 1;
						int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].res_A;
					}
					else{
						RES_Stat_Int[big_int[sel_int_unit].res_stat_num].if_branch = 0;
						branch_cond = 0;
						int_result[big_int[sel_int_unit].res_stat_num] = current_PC + 4;
					}
					break;

				case BNEZ:
					if(RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vj != 0){
						RES_Stat_Int[big_int[sel_int_unit].res_stat_num].if_branch = 1;
						int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].res_A;
					}
					else{
						RES_Stat_Int[big_int[sel_int_unit].res_stat_num].if_branch = 0;
						branch_cond = 0;
						int_result[big_int[sel_int_unit].res_stat_num] = current_PC + 4;
					}
					break;

				case BLTZ:
					if(RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vj < 0){
						RES_Stat_Int[big_int[sel_int_unit].res_stat_num].if_branch = 1;
						int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].res_A;
					}
					else{
						RES_Stat_Int[big_int[sel_int_unit].res_stat_num].if_branch = 0;
						int_result[big_int[sel_int_unit].res_stat_num] = current_PC + 4;
					}
					break;

				case BGTZ:
					if(RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vj > 0){
						RES_Stat_Int[big_int[sel_int_unit].res_stat_num].if_branch = 1;
						int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].res_A;
					}
					else{
						RES_Stat_Int[big_int[sel_int_unit].res_stat_num].if_branch = 0;
						int_result[big_int[sel_int_unit].res_stat_num] = current_PC + 4;
					}
					break;

				case BLEZ:
					if(RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vj <= 0){
						int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].res_A;
					}
					else{
						RES_Stat_Int[big_int[sel_int_unit].res_stat_num].if_branch = 0;
						int_result[big_int[sel_int_unit].res_stat_num] = current_PC + 4;
					}
					break;

				case BGEZ:
					if(RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vj >= 0){
						RES_Stat_Int[big_int[sel_int_unit].res_stat_num].if_branch = 1;
						int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].res_A;
					}
					else{
						RES_Stat_Int[big_int[sel_int_unit].res_stat_num].if_branch = 0;
						int_result[big_int[sel_int_unit].res_stat_num] = current_PC + 4;
					}
					break;

				case JUMP:
					RES_Stat_Int[big_int[sel_int_unit].res_stat_num].if_branch = 1;
					int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].res_A;
					break;

				default:
					break;
				}

				for(unsigned a = 0; a < create_exe_unit.size(); a++){
					if(create_exe_unit[a].exe_unit == INTEGER){
						hold_int = a;
					}
				}

				num_latency_int[sel_int_unit] = create_exe_unit[hold_int].latency_exe;
				big_int[sel_int_unit].status = 1;
				RES_Stat_Int[big_int[sel_int_unit].res_stat_num].use_ex = UNDEFINED;
			}
			break;
		}
		else{
			int_struc_hazard = 1;
		}
	}

	for(unsigned i = 0; i < RES_Stat_Add.size(); i++){
		if(RES_Stat_Add[i].res_busy == 1){
			if((RES_Stat_Add[i].Qj == 0) && (RES_Stat_Add[i].Qk == 0)){
				switch (RES_Stat_Add[i].op){
					case ADDER:
						for(unsigned sel_add_unit = 0; sel_add_unit < big_add.size(); sel_add_unit++){
							if((big_int[sel_add_unit].status == 1) && (ROB_table[RES_Stat_Add[i].res_dest].state == ISSUE)){
								ROB_table[RES_Stat_Add[i].res_dest].state = EXECUTE;
								big_int[sel_add_unit].status = 0;
								big_int[sel_add_unit].res_stat_num = i;
								break;
							}
						}
						break;

					default:
						break;
				}
			}
		}
	}

	for(unsigned sel_add_unit = 0; sel_add_unit < big_add.size(); sel_add_unit++){
		if(big_add[sel_add_unit].status == 1){
			if(num_latency_add[sel_add_unit] > 1){
				num_latency_add[sel_add_unit]--;
			}
			else{
				switch (opcode_map[instruction_memory[RES_Stat_Add[big_add[sel_add_unit].res_stat_num].op_num].op_code]) {
					case ADDS:
							temp_adds = unsigned2float(RES_Stat_Add[big_add[sel_add_unit].res_stat_num].Vj) + unsigned2float(RES_Stat_Add[big_add[sel_add_unit].res_stat_num].Vk);
							add_result[big_add[sel_add_unit].res_stat_num] = float2unsigned(float(temp_adds));
							break;

						case SUBS:
							temp_adds = unsigned2float(RES_Stat_Add[big_add[sel_add_unit].res_stat_num].Vj) - unsigned2float(RES_Stat_Add[big_add[sel_add_unit].res_stat_num].Vk);
							add_result[big_add[sel_add_unit].res_stat_num] = float2unsigned(float(temp_adds));
							break;
					}
					for(unsigned a = 0; a < create_exe_unit.size(); a++){
						if(create_exe_unit[a].exe_unit == ADDER){
							hold_add = a;
						}
					}
					num_latency_add[sel_add_unit] = create_exe_unit[hold_add].latency_exe;
					big_add[sel_add_unit].status = 1;
				}
			}
	}

	for(unsigned i = 0; i < RES_Stat_Mul.size(); i++){
		if(RES_Stat_Mul[i].res_busy == 1){
			if((RES_Stat_Mul[i].Qj == 0) && (RES_Stat_Mul[i].Qk == 0)){
				switch (RES_Stat_Mul[i].op) {
					case MULTIPLIER:
						for(unsigned sel_mul_unit = 0; sel_mul_unit < big_mul.size(); sel_mul_unit++){
							if((big_mul[sel_mul_unit].status == 1) && (ROB_table[RES_Stat_Mul[i].res_dest].state == ISSUE)){
								big_mul[sel_mul_unit].status = 0;
								ROB_table[RES_Stat_Mul[i].res_dest].state = EXECUTE;
								big_mul[sel_mul_unit].res_stat_num = i;
								break;
							}
						}
						break;

					case DIVIDER:
						for(unsigned sel_div_unit = 0; sel_div_unit < big_div.size(); sel_div_unit++){
							if(big_div[sel_div_unit].status == 1){
								big_div[sel_div_unit].status = 0;
								ROB_table[RES_Stat_Mul[i].res_dest].state = EXECUTE;
								big_div[sel_div_unit].res_stat_num = i;
								break;
							}
						}
						break;

					default:
						break;
							}
						}
				}
			}


	for(unsigned sel_mul_unit = 0; sel_mul_unit < big_mul.size(); sel_mul_unit++){
		if(big_mul[sel_mul_unit].status == 0){
			if(num_latency_mul[sel_mul_unit] > 1){
				num_latency_mul[sel_mul_unit]--;
			}
			else{
				switch (opcode_map[instruction_memory[RES_Stat_Mul[big_mul[sel_mul_unit].res_stat_num].op_num].op_code]) {
					case MULT:
						mul_result[big_mul[sel_mul_unit].res_stat_num] = RES_Stat_Mul[big_mul[sel_mul_unit].res_stat_num].Vj * RES_Stat_Mul[big_mul[sel_mul_unit].res_stat_num].Vk;
						break;

			  	case MULTS:
						temp_muls = float2unsigned(float(RES_Stat_Mul[big_mul[sel_mul_unit].res_stat_num].Vj)) + float2unsigned(float(RES_Stat_Mul[big_mul[sel_mul_unit].res_stat_num].Vk));
						mul_result[big_mul[sel_mul_unit].res_stat_num] = unsigned2float(temp_muls);
						break;
					}
					for(unsigned a = 0; a < create_exe_unit.size(); a++){
						if(create_exe_unit[a].exe_unit == MULTIPLIER){
							hold_mul = a;
						}
					}
					num_latency_mul[sel_mul_unit] = create_exe_unit[hold_mul].latency_exe;
				  big_mul[sel_mul_unit].status = 1;
			}
		}
	}
	for(unsigned sel_div_unit = 0; sel_div_unit < big_div.size(); sel_div_unit++){
		if(big_div[sel_div_unit].status == 0){
			if(num_latency_div[sel_div_unit] > 1){
				num_latency_div[sel_div_unit]--;
			}
			else{
				switch (opcode_map[instruction_memory[RES_Stat_Mul[big_div[sel_div_unit].res_stat_num].op_num].op_code]) {
					case DIV:
						mul_result[big_div[sel_div_unit].res_stat_num] = RES_Stat_Mul[big_div[sel_div_unit].res_stat_num].Vj / RES_Stat_Mul[big_div[sel_div_unit].res_stat_num].Vk;
						break;

					case DIVS:
						temp_divs = float2unsigned(float(RES_Stat_Mul[big_div[sel_div_unit].res_stat_num].Vj)) / float2unsigned(float(RES_Stat_Mul[big_div[sel_div_unit].res_stat_num].Vk));
						mul_result[big_div[sel_div_unit].res_stat_num] = unsigned2float(temp_divs);
						break;
				}
				for(unsigned a = 0; a < create_exe_unit.size(); a++){
					if(create_exe_unit[a].exe_unit == DIVIDER){
						hold_div = a;
					}
				}
				num_latency_div[sel_div_unit] = create_exe_unit[hold_div].latency_exe;
				big_div[sel_div_unit].status = 1;
			}
		}
	}
	unsigned store_check;
	unsigned sw_check_head;

	for(unsigned i = 0; i < RES_Stat_Load.size(); i++){
		if(RES_Stat_Load[i].res_busy == 1){
			if((RES_Stat_Load[i].Qj == 0) && (RES_Stat_Load[i].Qk == 0)){
				switch(RES_Stat_Load[i].op){
					case MEMORY:
						for(unsigned sel_ld_unit = 0; sel_ld_unit < big_mem.size(); sel_ld_unit++){
							if(big_mem[sel_ld_unit].status == 1 && ROB_table[RES_Stat_Load[i].res_dest].state == ISSUE){
								big_mem[sel_ld_unit].status = 0;
								ROB_table[RES_Stat_Load[i].res_dest].state == EXECUTE;
								big_mem[sel_ld_unit].res_stat_num = i;
								break;
							}
						}
						break;

					default:
						break;
				}
			}
		}
	}
	for(unsigned sel_ld_unit = 0; sel_ld_unit < big_mem.size(); sel_ld_unit++){
		if(big_mem[sel_ld_unit].status == 0){
			if(num_latency_mem[sel_ld_unit] > 1){
				num_latency_mem[sel_ld_unit]--;
			}
			else{
				switch (opcode_map[instruction_memory[RES_Stat_Load[big_mem[sel_ld_unit].res_stat_num].op_num].op_code]) {
					case LW:
					case LWS:
						for(unsigned iter_ld = big_mem[sel_ld_unit].res_stat_num-1; iter_ld > 0; iter_ld--){
							if((opcode_map[instruction_memory[RES_Stat_Load[iter_ld].op_num].op_code] == SW)
							|| (opcode_map[instruction_memory[RES_Stat_Load[iter_ld].op_num].op_code] == SWS)) {
								store_check = 1;
								break;
							}
							else{
								store_check = 0;
							}
						}
						if((RES_Stat_Load[big_mem[sel_ld_unit].res_stat_num].Qj == 0) && (store_check != 1)){
							RES_Stat_Load[big_mem[sel_ld_unit].res_stat_num].res_A = RES_Stat_Load[big_mem[sel_ld_unit].res_stat_num].Vj + RES_Stat_Load[big_mem[sel_ld_unit].res_stat_num].res_A;
							load_done = 1;
						}
						break;

					case SWS:
					case SW:
						if((opcode_map[instruction_memory[ROB_table[head_ROB].rob_pc].op_code] == SW)
						||(opcode_map[instruction_memory[ROB_table[head_ROB].rob_pc].op_code] == SWS)){
							sw_check_head = 1;
						}
						else{
							sw_check_head = 0;
						}
						if((RES_Stat_Load[big_mem[sel_ld_unit].res_stat_num].Qj == 0) && (sw_check_head == 1)){
							mem_result[big_mem[sel_ld_unit].res_stat_num] = RES_Stat_Load[big_mem[sel_ld_unit].res_stat_num].Vj + RES_Stat_Load[big_mem[sel_ld_unit].res_stat_num].res_A;
						}
						break;
				}
			}
		}
	}
}

void sim_ooo::write_res(){
	unsigned b_rob;

	for(unsigned i = 0; i < RES_Stat_Int.size(); i++){
		ROB_table[RES_Stat_Int[i].res_dest].rob_val = int_result[i];
		ROB_table[RES_Stat_Int[i].res_dest].if_branch_rob = RES_Stat_Int[i].if_branch;
	}

	for(unsigned i = 0; i < RES_Stat_Add.size(); i++){
		ROB_table[RES_Stat_Add[i].res_dest].rob_val = add_result[i];
	}

	for(unsigned i = 0; i < RES_Stat_Mul.size(); i++){
		ROB_table[RES_Stat_Mul[i].res_dest].rob_val = mul_result[i];
	}

	for(unsigned i = 0; i < RES_Stat_Load.size(); i++){
		ROB_table[RES_Stat_Load[i].res_dest].rob_val = mem_result[i];
	}

	for(unsigned i = 0; i < RES_Stat_Int.size(); i++){
			b_rob = RES_Stat_Int[i].res_dest;
			RES_Stat_Int[i].res_busy = 0;
			if(RES_Stat_Int[i].Qj == b_rob){
				RES_Stat_Int[i].Vj = ROB_table[b_rob].rob_val;
				RES_Stat_Int[i].Qj = 0;
			}
			if(RES_Stat_Int[i].Qk == b_rob){
				RES_Stat_Int[i].Vk = ROB_table[b_rob].rob_val;
				RES_Stat_Int[i].Qk = 0;
			}
			ROB_table[b_rob].ready = 1;
			ROB_table[b_rob].state = WRITE_RESULT;
	}

	for(unsigned i = 0; i < RES_Stat_Add.size(); i++){
			b_rob = RES_Stat_Add[i].res_dest;
			RES_Stat_Add[i].res_busy = 0;
			if(RES_Stat_Add[i].Qj == b_rob){
				RES_Stat_Add[i].Vj = ROB_table[b_rob].rob_val;
				RES_Stat_Add[i].Qj = 0;
			}
			if(RES_Stat_Add[i].Qk == b_rob){
				RES_Stat_Add[i].Vk = ROB_table[b_rob].rob_val;
				RES_Stat_Add[i].Qk = 0;
			}
			ROB_table[b_rob].ready = 1;
			ROB_table[b_rob].state = WRITE_RESULT;
	}

	for(unsigned i = 0; i < RES_Stat_Mul.size(); i++){
			b_rob = RES_Stat_Mul[i].res_dest;
			RES_Stat_Mul[i].res_busy = 0;
			if(RES_Stat_Mul[i].Qj == b_rob){
				RES_Stat_Mul[i].Vj = ROB_table[b_rob].rob_val;
				RES_Stat_Mul[i].Qj = 0;
			}
			if(RES_Stat_Mul[i].Qk == b_rob){
				RES_Stat_Mul[i].Vk = ROB_table[b_rob].rob_val;
				RES_Stat_Mul[i].Qk = 0;
			}
			ROB_table[b_rob].ready = 1;
			ROB_table[b_rob].state = WRITE_RESULT;
	}

	unsigned if_addr_same;
	for(unsigned i = 0; i < RES_Stat_Load.size(); i++){
		switch (opcode_map[instruction_memory[RES_Stat_Load[i].op_num].op_code]){
			case LW:
			case LWS:
				for(unsigned iter_mem = 0; iter_mem < RES_Stat_Load.size(); iter_mem++){
					if(mem_result[iter_mem] == RES_Stat_Load[i].res_A){
						if_addr_same = 1;
						break;
					}
					else{
						if_addr_same = 0;
					}
				}
				if((load_done == 1) && (if_addr_same!=1)){
					ROB_table[RES_Stat_Load[i].res_dest].rob_val = char2unsigned(data_memory + RES_Stat_Load[i].res_A);
				}
				break;

			case SW:
			case SWS:
				if(RES_Stat_Load[i].Qk == 0){
					ROB_table[head_ROB].rob_val = RES_Stat_Load[i].Vk;
				}
				break;
		}
	}
}

void sim_ooo::commit_stage(){
	unsigned d;
	if(ROB_table[head_ROB].ready == 1){
		d = ROB_table[head_ROB].rob_dest;
		switch(opcode_map[instruction_memory[ROB_table[head_ROB].rob_pc].op_code]){
			case BEQZ:
			case BNEZ:
			case BLTZ:
			case BGTZ:
			case BLEZ:
			case BGEZ:
				if(ROB_table[head_ROB].if_branch_rob == 0){
					current_PC = ROB_table[head_ROB].rob_val;
					clear_rob();
					clear_res();
					clear_reg();
				}
				break;

			case SW:
			case SWS:
				unsigned2char(ROB_table[head_ROB].rob_val,data_memory+ROB_table[head_ROB].rob_dest);
				break;

			case XOR:
			case ADD:
			case SUB:
			case OR:
			case AND:
			case MULT:
			case DIV:
			case XORI:
			case ADDI:
			case SUBI:
			case ORI:
			case ANDI:
				register_file[d] = ROB_table[head_ROB].rob_val;
				break;

			case ADDS:
			case SUBS:
			case MULTS:
			case DIVS:
				float_reg_file[d] = ROB_table[head_ROB].rob_val;
				break;
		}
		ROB_table[head_ROB].rob_busy = false;
		ROB_table[head_ROB].ready = false;
		ROB_table[head_ROB].rob_pc = UNDEFINED;
		ROB_table[head_ROB].state = INVALID;
		ROB_table[head_ROB].rob_dest = UNDEFINED;
		ROB_table[head_ROB].dest_type = NOTASSIGNED;
		if(int_reg_stat[d].ROB_num == head_ROB){
			int_reg_stat[d].reg_busy = 0;
		}

		if(float_reg_stat[d].ROB_num == head_ROB){

			float_reg_stat[d].reg_busy = 0;
		}
	}
}
//reset the state of the sim_oooulator

void sim_ooo::clear_rob(){
	for(unsigned a = 0; a < ROB_table.size(); a++){           //initializing ROB
		ROB_tab.entry = a+1;
		ROB_table[a] = ROB_tab;
		head_ROB = 0;
	}
}

void sim_ooo::clear_res(){
	RES_Temp.res_name = "Int";
	for(unsigned lint = 0; lint < RES_Stat_Int.size(); lint++){
    RES_Stat_Int[lint] = RES_Temp;
		int_result[lint] = UNDEFINED;
  }

	RES_Temp.res_name = "Mult";
  for(unsigned lmul = 0; lmul < RES_Stat_Mul.size(); lmul++){
		RES_Stat_Mul[lmul] = RES_Temp;
		mul_result[lmul] = UNDEFINED;
  }

	RES_Temp.res_name = "Add";
  for(unsigned ladd = 0; ladd < RES_Stat_Add.size(); ladd++){
		RES_Stat_Add[ladd] = RES_Temp;
		add_result[ladd] = UNDEFINED;
	}

	RES_Temp.res_name = "Load";
  for(unsigned lload = 0; lload < RES_Stat_Load.size(); lload++){
		RES_Stat_Load[lload] = RES_Temp;
		add_result[lload] = UNDEFINED;
	}
}

void sim_ooo::clear_reg(){
	for(unsigned i = 0; i < NUM_GP_REGISTERS; i++){
    int_reg_stat[i].reg_busy = 0;
    int_reg_stat[i].ROB_num = UNDEFINED;
    float_reg_stat[i].reg_busy = 0;
    float_reg_stat[i].ROB_num = UNDEFINED;
  }
}

void sim_ooo::reset(){
}

int sim_ooo::get_int_register(unsigned reg){
	return register_file[reg]; //fill here
}

void sim_ooo::set_int_register(unsigned reg, int value){
  register_file[reg] = value;
}

float sim_ooo::get_fp_register(unsigned reg){
	return float_reg_file[reg]; //fill here
}

void sim_ooo::set_fp_register(unsigned reg, float value){
  float_reg_file[reg] = value;
}

unsigned sim_ooo::get_pending_int_register(unsigned reg){
	return UNDEFINED; //fill here
}

unsigned sim_ooo::get_pending_fp_register(unsigned reg){
	return UNDEFINED; //fill here
}

void sim_ooo::print_status(){
	print_pending_instructions();
	print_rob();
	print_reservation_stations();
	print_registers();
}

void sim_ooo::print_memory(unsigned start_address, unsigned end_address){
	cout << "DATA MEMORY[0x" << hex << setw(8) << setfill('0') << start_address << ":0x" << hex << setw(8) << setfill('0') <<  end_address << "]" << endl;
	for (unsigned i=start_address; i<end_address; i++){
		if (i%4 == 0) cout << "0x" << hex << setw(8) << setfill('0') << i << ": ";
		cout << hex << setw(2) << setfill('0') << int(data_memory[i]) << " ";
		if (i%4 == 3){
			cout << endl;
		}
	}
}

void sim_ooo::write_memory(unsigned address, unsigned value){
	unsigned2char(value,data_memory+address);
}

void sim_ooo::print_registers(){
        unsigned i;
	cout << "GENERAL PURPOSE REGISTERS" << endl;
	cout << setfill(' ') << setw(8) << "Register" << setw(22) << "Value" << setw(5) << "ROB" << endl;
        for (i=0; i< NUM_GP_REGISTERS; i++){
                if (get_pending_int_register(i)!=UNDEFINED)
			cout << setfill(' ') << setw(7) << "R" << dec << i << setw(22) << "-" << setw(5) << get_pending_int_register(i) << endl;
                else if (get_int_register(i)!=(int)UNDEFINED)
			cout << setfill(' ') << setw(7) << "R" << dec << i << setw(11) << get_int_register(i) << hex << "/0x" << setw(8) << setfill('0') << get_int_register(i) << setfill(' ') << setw(5) << "-" << endl;
        }
	for (i=0; i< NUM_GP_REGISTERS; i++){
                if (get_pending_fp_register(i)!=UNDEFINED)
			cout << setfill(' ') << setw(7) << "F" << dec << i << setw(22) << "-" << setw(5) << get_pending_fp_register(i) << endl;
                else if (get_fp_register(i)!=UNDEFINED)
			cout << setfill(' ') << setw(7) << "F" << dec << i << setw(11) << get_fp_register(i) << hex << "/0x" << setw(8) << setfill('0') << float2unsigned(get_fp_register(i)) << setfill(' ') << setw(5) << "-" << endl;
	}
	cout << endl;
}

void sim_ooo::print_rob(){
	cout << "REORDER BUFFER" << endl;
	cout << setfill(' ') << setw(5) << "Entry" << setw(6) << "Busy" << setw(7) << "Ready" << setw(12) << "PC" << setw(10) << "State" << setw(6) << "Dest" << setw(12) << "Value" << endl;
	//fill here
	for(unsigned i = 0; i < ROB_table.size(); i++){
		std::cout << setfill(' ') << setw(5) << (i+1);
		(ROB_table[i].rob_busy)?std::cout << setw(6) << "yes":std::cout << setw(6) << "no";
		(ROB_table[i].ready)?std::cout << setw(7) << "yes":std::cout << setw(7) << "no";
		if(ROB_table[i].state == INVALID){
			std::cout << setw(12) << "-";
		}
		else{
			std::cout << setw(4) << hex << "0x" << setw(8) << setfill('0') << ROB_table[i].rob_pc;
		}
		switch (ROB_table[i].state) {
			case ISSUE:
				std::cout << setfill(' ') << setw(10) << "ISSUE";
				break;
			case EXECUTE:
				std::cout << setfill(' ') << setw(10) << "EXE";
				break;
			case WRITE_RESULT:
				std::cout << setfill(' ') << setw(10) << "WR";
				break;
			case COMMIT:
				std::cout << setfill(' ') << setw(10) << "-";
				break;
			case INVALID:
				std::cout << setfill(' ') << setw(10) << "-";
				break;
		}
		if(ROB_table[i].state == INVALID)std::cout << setw(6) << "-";
		else std::cout << setfill(' ') << setw(5) << "F" << ROB_table[i].rob_dest;
		if(ROB_table[i].state == WRITE_RESULT)std::cout << setw(12) << ROB_table[i].rob_val << '\n';
		else std::cout << setw(12) << "-" << '\n';
	}

	cout << endl;
}

void sim_ooo::print_reservation_stations(){
	cout << "RESERVATION STATIONS" << endl;
	cout  << setfill(' ');
	cout << setw(7) << "Name" << setw(6) << "Busy" << setw(12) << "PC" << setw(12) << "Vj" << setw(12) << "Vk" << setw(6) << "Qj" << setw(6) << "Qk" << setw(6) << "Dest" << setw(12) << "Address" << endl;

	// fill here
	for(unsigned i = 0; i < RES_Stat_Int.size(); i++){
		std::cout << setfill(' ') << setw(6) << RES_Stat_Int[i].res_name << (i+1);
		if(RES_Stat_Int[i].res_busy == 1)std::cout << setw(6) << "YES";
		else std::cout << setw(6) << "NO";
		if(RES_Stat_Int[i].res_busy == 1)std::cout << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Int[i].res_pc;
		else std::cout << setw(12) << "-";
		if((RES_Stat_Int[i].res_busy == 1) && (RES_Stat_Int[i].Vj!=0))std::cout << setfill(' ') << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Int[i].Vj;
		else std::cout << setw(12) << "-";
		if((RES_Stat_Int[i].res_busy == 1) && (RES_Stat_Int[i].Vk!=0))std::cout << setfill(' ') << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Int[i].Vk;
		else std::cout << setfill(' ') << setw(12) << "-";
		if((RES_Stat_Int[i].res_busy == 1) && (RES_Stat_Int[i].Qj!=0))std::cout << setfill(' ') << setw(6) << RES_Stat_Int[i].Qj;
		else std::cout << setfill(' ') << setw(6) << "-";
		if((RES_Stat_Int[i].res_busy == 1) && (RES_Stat_Int[i].Qk!=0))std::cout << setfill(' ') << setw(6) << RES_Stat_Int[i].Qk;
		else std::cout << setfill(' ') << setw(6) << "-";
		if(RES_Stat_Int[i].res_busy == 1)std::cout << setw(6) << RES_Stat_Int[i].res_dest;
		else std::cout << setfill(' ') << setw(6) << "-";
		std::cout << setfill(' ') << setw(12) << "-" << '\n';
	}

	for(unsigned i = 0; i < RES_Stat_Load.size(); i++){
		std::cout << setfill(' ') << setw(6) << RES_Stat_Mul[i].res_name << (i+1);
		if(RES_Stat_Load[i].res_busy == 1)std::cout << setw(6) << "YES";
		else std::cout << setw(6) << "NO";
		if(RES_Stat_Load[i].res_busy == 1)std::cout << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Load[i].res_pc;
		else std::cout << setw(12) << "-";
		if((RES_Stat_Load[i].res_busy == 1) && (RES_Stat_Load[i].Vj!=0))std::cout << setfill(' ') << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Load[i].Vj;
		else std::cout << setw(12) << "-";
		if((RES_Stat_Load[i].res_busy == 1) && (RES_Stat_Load[i].Vk!=0))std::cout << setfill(' ') << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Load[i].Vk;
		else std::cout << setfill(' ') << setw(12) << "-";
		if((RES_Stat_Load[i].res_busy == 1) && (RES_Stat_Load[i].Qj!=0))std::cout << setfill(' ') << setw(6) << RES_Stat_Load[i].Qj;
		else std::cout << setfill(' ') << setw(6) << "-";
		if((RES_Stat_Load[i].res_busy == 1) && (RES_Stat_Load[i].Qk!=0))std::cout << setfill(' ') << setw(6) << RES_Stat_Load[i].Qk;
		else std::cout << setfill(' ') << setw(6) << "-";
		if(RES_Stat_Load[i].res_busy == 1)std::cout << setw(6) << RES_Stat_Load[i].res_dest;
		else std::cout << setfill(' ') << setw(6) << "-";
		std::cout << setfill(' ') << setw(12) << "-" << '\n';
	}

	for(unsigned i = 0; i < RES_Stat_Add.size(); i++){
		std::cout << setfill(' ') << setw(6) << RES_Stat_Add[i].res_name << (i+1);
		if(RES_Stat_Add[i].res_busy == 1)std::cout << setw(6) << "YES";
		else std::cout << setw(6) << "NO";
		if(RES_Stat_Add[i].res_busy == 1)std::cout << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Add[i].res_pc;
		else std::cout << setw(12) << "-";
		if((RES_Stat_Add[i].res_busy == 1) && (RES_Stat_Add[i].Vj!=0))std::cout << setfill(' ') << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Add[i].Vj;
		else std::cout << setw(12) << "-";
		if((RES_Stat_Add[i].res_busy == 1) && (RES_Stat_Add[i].Vk!=0))std::cout << setfill(' ') << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Add[i].Vk;
		else std::cout << setfill(' ') << setw(12) << "-";
		if((RES_Stat_Add[i].res_busy == 1) && (RES_Stat_Add[i].Qj!=0))std::cout << setfill(' ') << setw(6) << RES_Stat_Add[i].Qj;
		else std::cout << setfill(' ') << setw(6) << "-";
		if((RES_Stat_Add[i].res_busy == 1) && (RES_Stat_Add[i].Qk!=0))std::cout << setfill(' ') << setw(6) << RES_Stat_Add[i].Qk;
		else std::cout << setfill(' ') << setw(6) << "-";
		if(RES_Stat_Add[i].res_busy == 1)std::cout << setw(6) << RES_Stat_Add[i].res_dest;
		else std::cout << setfill(' ') << setw(6) << "-";
		std::cout << setfill(' ') << setw(12) << "-" << '\n';
	}

	for(unsigned i = 0; i < RES_Stat_Mul.size(); i++){
		std::cout << setfill(' ') << setw(6) << RES_Stat_Mul[i].res_name << (i+1);
		if(RES_Stat_Mul[i].res_busy == 1)std::cout << setw(6) << "YES";
		else std::cout << setw(6) << "NO";
		if(RES_Stat_Mul[i].res_busy == 1)std::cout << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Mul[i].res_pc;
		else std::cout << setw(12) << "-";
		if((RES_Stat_Mul[i].res_busy == 1) && (RES_Stat_Mul[i].Vj!=0))std::cout << setfill(' ') << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Mul[i].Vj;
		else std::cout << setw(12) << "-";
		if((RES_Stat_Mul[i].res_busy == 1) && (RES_Stat_Mul[i].Vk!=0))std::cout << setfill(' ') << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Mul[i].Vk;
		else std::cout << setfill(' ') << setw(12) << "-";
		if((RES_Stat_Mul[i].res_busy == 1) && (RES_Stat_Mul[i].Qj!=0))std::cout << setfill(' ') << setw(6) << RES_Stat_Mul[i].Qj;
		else std::cout << setfill(' ') << setw(6) << "-";
		if((RES_Stat_Mul[i].res_busy == 1) && (RES_Stat_Mul[i].Qk!=0))std::cout << setfill(' ') << setw(6) << RES_Stat_Mul[i].Qk;
		else std::cout << setfill(' ') << setw(6) << "-";
		if(RES_Stat_Mul[i].res_busy == 1)std::cout << setw(6) << RES_Stat_Mul[i].res_dest;
		else std::cout << setfill(' ') << setw(6) << "-";
		std::cout << setfill(' ') << setw(12) << "-" << '\n';
	}

	cout << endl;
}

void sim_ooo::print_pending_instructions(){
	cout << "PENDING INSTRUCTIONS STATUS" << endl;
	cout << setfill(' ');
	cout << setw(10) << "PC" << setw(7) << "Issue" << setw(7) << "Exe" << setw(7) << "WR" << setw(7) << "Commit";
	cout << endl;
}

void sim_ooo::print_log(){
}

float sim_ooo::get_IPC(){
	return UNDEFINED; //fill here
}

unsigned sim_ooo::get_instructions_executed(){
	return UNDEFINED; //fill here
}

unsigned sim_ooo::get_clock_cycles(){
	return UNDEFINED; //fill here
}
