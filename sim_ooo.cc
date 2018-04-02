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
  stage_t state;
  char rob_dest[10];
  unsigned rob_val;
};

struct RES_Stat_struc{
  unsigned res_name;
  unsigned res_busy;
  unsigned res_pc;
  int Vj;
  int Vk;
  unsigned Qj;
  unsigned Qk;
  unsigned res_dest;
  unsigned res_A;
};

struct REGISTER_Stat{
  unsigned ROB_num;
  unsigned reg_busy;
};

struct exe_struc{
  int Vj;
  int Vk;
  int Add;
  int status;
};

struct pack_of_exec
{
	exe_unit_t exe_unit;
	unsigned latency_exe;
	unsigned instance_exe;
};

every_line every_instruct;

ROB_struc ROB_tab = {1,0,0,UNDEFINED,INVALID,"",UNDEFINED};
RES_Stat_struc RES_Temp = {UNDEFINED,0,UNDEFINED,UNDEFINED,UNDEFINED,UNDEFINED,UNDEFINED,UNDEFINED,UNDEFINED};
exe_struc TempReg = {UNDEFINED,UNDEFINED,UNDEFINED,UNDEFINED};
pack_of_exec execution_structure;
std::vector<REGISTER_Stat> int_reg_stat;
std::vector<REGISTER_Stat> float_reg_stat;

std::vector<pack_of_exec> create_exe_unit;

std::vector<every_line> instruction_memory;

std::vector<RES_Stat_struc> RES_Stat_Int;
std::vector<RES_Stat_struc> RES_Stat_Load;
std::vector<RES_Stat_struc> RES_Stat_Mul;
std::vector<RES_Stat_struc> RES_Stat_Add;

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

std::map<string, int> opcode_map;

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

  for(unsigned lint = 0; lint < num_int_res_stations; lint++){
    RES_Temp.res_name = lint + 1;
    RES_Stat_Int.push_back(RES_Temp);
  }

  for(unsigned lmul = 0; lmul < num_mul_res_stations; lmul++){
    RES_Temp.res_name = lmul + 1;
    RES_Stat_Mul.push_back(RES_Temp);
  }

  for(unsigned ladd = 0; ladd < num_add_res_stations; ladd++){
    RES_Temp.res_name = ladd + 1;
    RES_Stat_Add.push_back(RES_Temp);
  }

  for(unsigned lload = 0; lload < num_load_res_stations; lload++){
    RES_Temp.res_name = lload + 1;
    RES_Stat_Load.push_back(RES_Temp);
  }

  int_reg_stat.resize(NUM_GP_REGISTERS);
  float_reg_stat.resize(NUM_FP_REGISTERS);

  for(unsigned i = 0; i < NUM_GP_REGISTERS; i++){
    int_reg_stat[i].reg_busy = 0;
    int_reg_stat[i].ROB_num = UNDEFINED;
    float_reg_stat[i].reg_busy = 0;
    float_reg_stat[i].ROB_num = UNDEFINED;
  }
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
}

void sim_ooo::issue(){
  char operands_needed[10];
  unsigned hold_val;
  unsigned hold_wr_val = UNDEFINED;
  unsigned hold_val1;
  unsigned hold_val2;
  unsigned hold_val1_float;
  unsigned hold_val2_float;
	unsigned res_yes;
	unsigned rob_yes;
	unsigned imm;
  switch (opcode_map[instruction_memory[0].op_code]) {
    case XOR:
  	case ADD:
  	case SUB:
  	case OR:
  	case AND:
			for(int i = 0; i < RES_Stat_Int.size(); i++){
				if(RES_Stat_Int[i].res_busy!=1){
					current_R_int = i;
					res_yes = 1;
					break;
				}
				else{
					res_yes = 0;
				}
			}
			for(int i = 0; i < ROB_table.size(); i++){
				if(ROB_table[i].rob_busy!=1){
					current_B = i;
					rob_yes = 1;
					break;
				}
				else{
					rob_yes = 0;
				}
			}
			if(res_yes == 1 && rob_yes == 1){
      	strcpy(operands_needed,instruction_memory[0].remainder_operand1.c_str());
	      hold_val1 = atoi(operands_needed+1);
	      strcpy(operands_needed,instruction_memory[0].remainder_operand2.c_str());
	      hold_val2 = atoi(operands_needed+1);
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
				ROB_table[current_B].rob_pc = 0;

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
				strcpy(operands_needed,instruction_memory[0].mand_operand.c_str());
	      hold_val = atoi(operands_needed+1);
				int_reg_stat[hold_val].ROB_num = current_B;
				int_reg_stat[hold_val].reg_busy = 1;
				strcpy(ROB_table[current_B].rob_dest,operands_needed);
			}

			break;

		case MULT:
		case DIV:
			for(int i = 0; i < RES_Stat_Mul.size(); i++){
				if(RES_Stat_Mul[i].res_busy!=1){
					current_R_mul = i;
					res_yes = 1;
					break;
				}
				else{
					res_yes = 0;
				}
			}
			for(int i = 0; i < ROB_table.size(); i++){
				if(ROB_table[i].rob_busy!=1){
					current_B = i;
					rob_yes = 1;
					break;
				}
				else{
					rob_yes = 0;
				}
			}
			if(res_yes == 1 && rob_yes == 1){
				strcpy(operands_needed,instruction_memory[0].remainder_operand1.c_str());
				hold_val1 = atoi(operands_needed+1);
				strcpy(operands_needed,instruction_memory[0].remainder_operand2.c_str());
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
				ROB_table[current_B].rob_pc = 0;

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
				strcpy(operands_needed,instruction_memory[0].mand_operand.c_str());
				hold_val = atoi(operands_needed+1);
				int_reg_stat[hold_val].ROB_num = current_B;
				int_reg_stat[hold_val].reg_busy = 1;
				strcpy(ROB_table[current_B].rob_dest,operands_needed);
			}

			break;

		case XORI:
		case ADDI:
		case SUBI:
		case ORI:
		case ANDI:
			char *endptr;
			for(int i = 0; i < RES_Stat_Int.size(); i++){
				if(RES_Stat_Int[i].res_busy!=1){
					current_R_int = i;
					res_yes = 1;
					break;
				}
				else{
					res_yes = 0;
				}
			}
			for(int i = 0; i < ROB_table.size(); i++){
				if(ROB_table[i].rob_busy!=1){
					current_B = i;
					rob_yes = 1;
					break;
				}
				else{
					rob_yes = 0;
				}
			}

			if(res_yes == 1 && rob_yes == 1){
				strcpy(operands_needed,instruction_memory[0].remainder_operand1.c_str());
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
				ROB_table[current_B].rob_pc = 0;

				strcpy(operands_needed,instruction_memory[0].remainder_operand2.c_str());
				imm = strtoul(operands_needed, &endptr, 0);
				RES_Stat_Int[current_R_int].res_A = imm;

				strcpy(operands_needed,instruction_memory[0].mand_operand.c_str());
				hold_val = atoi(operands_needed+1);
				int_reg_stat[hold_val].ROB_num = current_B;
				int_reg_stat[hold_val].reg_busy = 1;
				strcpy(ROB_table[current_B].rob_dest,operands_needed);
			}

			break;

			case ADDS:
			case SUBS:
				for(int i = 0; i < RES_Stat_Add.size(); i++){
					if(RES_Stat_Add[i].res_busy!=1){
						current_R_add = i;
						res_yes = 1;
						break;
					}
					else{
						res_yes = 0;
					}
				}
				for(int i = 0; i < ROB_table.size(); i++){
					if(ROB_table[i].rob_busy!=1){
						current_B = i;
						rob_yes = 1;
						break;
					}
					else{
						rob_yes = 0;
					}
				}

				if(res_yes == 1 && rob_yes == 1){
					strcpy(operands_needed,instruction_memory[0].remainder_operand1.c_str());
					hold_val1 = atoi(operands_needed+1);
					strcpy(operands_needed,instruction_memory[0].remainder_operand2.c_str());
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
						RES_Stat_Add[current_R_add].Vj = float_reg_file[hold_val1];
						RES_Stat_Add[current_R_add].Qj = 0;
					}
					RES_Stat_Add[current_R_add].res_busy = 1;
					RES_Stat_Add[current_R_add].res_dest = current_B;
					ROB_table[current_B].rob_pc = 0;

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
						RES_Stat_Add[current_R_add].Vk = float_reg_file[hold_val2];
						RES_Stat_Add[current_R_add].Qk = 0;
					}

					strcpy(operands_needed,instruction_memory[0].mand_operand.c_str());
					hold_val = atoi(operands_needed+1);
					float_reg_stat[hold_val].ROB_num = current_B;
					float_reg_stat[hold_val].reg_busy = 1;
					strcpy(ROB_table[current_B].rob_dest,operands_needed);
				}
  }
}
//reset the state of the sim_oooulator
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

	cout << endl;
}

void sim_ooo::print_reservation_stations(){
	cout << "RESERVATION STATIONS" << endl;
	cout  << setfill(' ');
	cout << setw(7) << "Name" << setw(6) << "Busy" << setw(12) << "PC" << setw(12) << "Vj" << setw(12) << "Vk" << setw(6) << "Qj" << setw(6) << "Qk" << setw(6) << "Dest" << setw(12) << "Address" << endl;

	// fill here

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
