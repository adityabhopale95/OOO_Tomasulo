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
	unsigned if_commit;
	unsigned add_reg;
	unsigned str_add;
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
	unsigned target_address;
	unsigned if_depend_Vj;
	unsigned if_depend_Vk;
	unsigned wr_done;
	unsigned imm_val;
	unsigned address_raw;
	unsigned from_forward;
  unsigned execution_unit_used;
	res_type_t res_dest_type;
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
 	unsigned status;
};

struct pack_of_exec
{
	exe_unit_t exe_unit;
	unsigned latency_exe;
	unsigned instance_exe;
};

std::map<string, int> opcode_map;

every_line every_instruct;

ROB_struc ROB_tab = {1,0,0,UNDEFINED,UNDEFINED,UNDEFINED,INVALID,UNDEFINED,UNDEFINED,NOTASSIGNED,0,UNDEFINED,UNDEFINED};
RES_Stat_struc RES_Temp = {"",0,UNDEFINED,UNDEFINED,UNDEFINED,UNDEFINED,UNDEFINED,UNDEFINED,UNDEFINED,UNDEFINED,UNDEFINED,0,0,0,0,0,UNDEFINED,UNDEFINED,UNDEFINED,UNDEFINED,NOTASSIGNED,NOTVALID};
exe_struc TempReg = {UNDEFINED,UNDEFINED,UNDEFINED,UNDEFINED,1};
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
std::vector<int> flag_int;
std::vector<int> flag_add;
std::vector<int> flag_mem;
std::vector<int> flag_mul;
std::vector<int> int_ex_done;
std::vector<int> add_ex_done;
std::vector<int> mul_ex_done;
std::vector<int> div_ex_done;
std::vector<int> ld_ex_done;
std::vector<int> temp_res_add;
std::vector<int> needs_ex_ld;
std::vector<int> ld_step1;

std::vector<int> ld_current_exe;

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
		flag_int.push_back(0);
  }

	RES_Temp.res_name = "Mult";
  for(unsigned lmul = 0; lmul < num_mul_res_stations; lmul++){
    RES_Stat_Mul.push_back(RES_Temp);
		mul_result.push_back(UNDEFINED);
		flag_mul.push_back(0);
  }

	RES_Temp.res_name = "Add";
  for(unsigned ladd = 0; ladd < num_add_res_stations; ladd++){
    RES_Stat_Add.push_back(RES_Temp);
		add_result.push_back(UNDEFINED);
		flag_add.push_back(0);
  }

	RES_Temp.res_name = "Load";
  for(unsigned lload = 0; lload < num_load_res_stations; lload++){
    RES_Stat_Load.push_back(RES_Temp);
		mem_result.push_back(UNDEFINED);
		flag_mem.push_back(0);
		needs_ex_ld.push_back(UNDEFINED);
		ld_step1.push_back(0);
		ld_current_exe.push_back(0);
		temp_res_add.push_back(UNDEFINED);
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
	num_cycles = 0;
	check_end = 0;
	num_instructions = 0;
	is_cleared = 0;
	is_store_ex = 0;
  is_load_ex = 0;
}

sim_ooo::~sim_ooo(){
}

void sim_ooo::init_exec_unit(exe_unit_t exec_unit, unsigned latency, unsigned instances){
  execution_structure.exe_unit = exec_unit;
	execution_structure.latency_exe = latency;
	execution_structure.instance_exe = instances;

  create_exe_unit.push_back(execution_structure);
	switch (create_exe_unit.back().exe_unit){
    case INTEGER:
				num_latency_int.resize(create_exe_unit.back().instance_exe);
				int_ex_done.resize(create_exe_unit.back().instance_exe);
				for (unsigned j = 0; j < num_latency_int.size(); j++){
					num_latency_int[j] = create_exe_unit.back().latency_exe;
					int_ex_done[j] = 0;
				}

				for (unsigned l = 0; l < create_exe_unit.back().instance_exe; l++){
					big_int.push_back(TempReg);
				}
				break;

		case ADDER:
  			num_latency_add.resize(create_exe_unit.back().instance_exe);
				add_ex_done.resize(create_exe_unit.back().instance_exe);
				for (unsigned j = 0; j < num_latency_add.size(); j++){
  				num_latency_add[j] = create_exe_unit.back().latency_exe;
					add_ex_done[j] = 0;
  		  }

				for (unsigned l = 0; l < create_exe_unit.back().instance_exe; l++){
				  big_add.push_back(TempReg);
				}
		break;

		case MULTIPLIER:
			num_latency_mul.resize(create_exe_unit.back().instance_exe);
			mul_ex_done.resize(create_exe_unit.back().instance_exe);
			for (unsigned j = 0; j < num_latency_mul.size(); j++){
				num_latency_mul[j] = create_exe_unit.back().latency_exe;
				mul_ex_done[j] = 0;
			}

      for (unsigned l = 0; l < create_exe_unit.back().instance_exe; l++){
				big_mul.push_back(TempReg);
			}
			break;

		case DIVIDER:
			num_latency_div.resize(create_exe_unit.back().instance_exe);
			div_ex_done.resize(create_exe_unit.back().instance_exe);
			for (unsigned j = 0; j < num_latency_div.size(); j++){
				num_latency_div[j] = create_exe_unit.back().latency_exe;
				div_ex_done[j] = 0;
			}

			for (unsigned l = 0; l < create_exe_unit.back().instance_exe; l++){
				big_div.push_back(TempReg);
			}
			break;

    case MEMORY:
  		num_latency_mem.resize(create_exe_unit.back().instance_exe);
			ld_ex_done.resize(create_exe_unit.back().instance_exe);
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
	if(cycles!=0){
		while (cycles!=0) {
			std::cout << "Cycle #" << dec << num_cycles << '\n';
			num_cycles++;

			commit_stage();

			write_res();

			execute_ins();

			if(is_cleared != 1 ){
				for(int i = 0; i < issue_size; i++){
					PC_index = (current_PC - base_add)/4;
					issue(PC_index);
					std::cout << "ISSUE SUCCESS: " << issue_success << '\n';
					if(issue_success == 1){
						current_PC = current_PC + 4;
					}
					else{
						break;
					}
				}
			}
      clear_flags();
			cycles--;
		}
	}
	else{
		while(cycles==0){
			if(eop_end){
				break;
			}
			else{
				std::cout << "Cycle #" << dec << num_cycles << '\n';
				num_cycles++;
				commit_stage();

				write_res();

				execute_ins();

				if(is_cleared != 1){
					for(int i = 0; i < issue_size; i++){
						PC_index = (current_PC - base_add)/4;
						issue(PC_index);
						if(issue_success == 1){
							current_PC = current_PC + 4;
						}
						else{
							break;
						}
					}
				}
        clear_flags();
			}
		}
	}
}

void sim_ooo::clear_flags(){
  for(unsigned i = 0; i < int_ex_done.size(); i++){
    int_ex_done[i] = 0;
  }

  for(unsigned i = 0; i < mul_ex_done.size(); i++){
    mul_ex_done[i] = 0;
  }

  for(unsigned i = 0; i < div_ex_done.size(); i++){
    div_ex_done[i] = 0;
  }

  for(unsigned i = 0; i < add_ex_done.size(); i++){
    add_ex_done[i] = 0;
  }

  for(unsigned i = 0; i < ld_ex_done.size(); i++){
    ld_ex_done[i] = 0;
  }
}

void sim_ooo::issue(unsigned temp_PC){
	std::cout << "************ISSUE***************" << '\n';
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
	unsigned h_val;
	int fake_head;
	unsigned current_B = UNDEFINED;


  /*for(unsigned i = 0; i < ld_ex_done.size(); i++){
    if(ld_ex_done[i]){
      ld_ex_done[i] = 0;
    }
  }*/
	std::cout << "Value of HEAD: " << head_ROB << '\n';
	for(unsigned i = head_ROB; i < ROB_table.size(); i++){
		std::cout << "ROB_BUSY: " << ROB_table[i].rob_busy << '\n';
		if(ROB_table[i].rob_busy != 1){
			std::cout << "Assigning ROB: " << i << '\n';
			/*if(ROB_table[i].if_commit == 1){
				ROB_table[i].if_commit = 0;
				rob_yes = 0;
				continue;
			}*/
			/*if(head_ROB == 0){
				if(i == (ROB_table.size()-1)){
					if(ROB_table[i].if_commit == 1){
						ROB_table[i].if_commit = 0;
						rob_yes = 0;
						continue;
					}
				}
			}*/
			current_B = i;
			rob_yes = 1;
			break;
		}
		else{
			rob_yes = 0;
		}
	}

	if(rob_yes == 0){
		for(unsigned j = 0; j < head_ROB; j++){
			if(ROB_table[j].rob_busy != 1){
				/*if(ROB_table[j].if_commit == 1){
					ROB_table[j].if_commit = 0;
					rob_yes = 0;
					continue;
				}*/
				/*if(j == (head_ROB-1)){
					if(ROB_table[(head_ROB-1)].if_commit == 1){
						std::cout << "Cannot issue ROB" << '\n';
						ROB_table[(head_ROB-1)].if_commit = 0;
						rob_yes = 0;
						//issue_success = 0;
						continue;
					}
				}*/
				current_B = j;
				rob_yes = 1;
				break;
			}
			else{
				rob_yes = 0;
			}
		}
	}

	for(unsigned i = 0; i < ROB_table.size(); i++){
		if(ROB_table[i].if_commit == 1){
			ROB_table[i].if_commit = 0;
			if(current_B == i){
				current_B = UNDEFINED;
				rob_yes = 0;
			}
			//break;
		}
	}

  for(unsigned iter = 0; iter < int_ex_done.size(); iter++){
    if(int_ex_done[iter] == 1){
      int_ex_done[iter] = 0;
    }
  }

  for(unsigned iter = 0; iter < add_ex_done.size(); iter++){
    if(add_ex_done[iter] == 1){
      add_ex_done[iter] = 0;
    }
  }

  for(unsigned iter = 0; iter < mul_ex_done.size(); iter++){
    if(mul_ex_done[iter] == 1){
      mul_ex_done[iter] = 0;
    }
  }

  for(unsigned iter = 0; iter < ld_ex_done.size(); iter++){
    if(mul_ex_done[iter] == 1){
      mul_ex_done[iter] = 0;
    }
  }

	for(unsigned it = 1; it < RES_Stat_Int.size(); it++){
		if(RES_Stat_Int[it].res_busy!=1){
			if(RES_Stat_Int[it].wr_done == 1){
				RES_Stat_Int[it].wr_done = 0;
				std::cout << "Wasted a life here for i: " << it << '\n';
				res_yes_int = 0;
				continue;
			}
			current_R_int = it;
			res_yes_int = 1;
			break;
		}
		else{
			res_yes_int = 0;
		}
	}

	for(unsigned i = 0; i < RES_Stat_Int.size(); i++){
		if(RES_Stat_Int[i].res_busy!=1){
			if(RES_Stat_Int[i].use_ex == 1){
				RES_Stat_Int[i].use_ex = 0;
				res_yes_int = 0;
				continue;
			}
			current_R_int = i;
			res_yes_int = 1;
			break;
		}
		else{
			res_yes_int = 0;
		}
	}

	for(unsigned it = 1; it < RES_Stat_Mul.size(); it++){
		if(RES_Stat_Mul[it].res_busy!=1){
			if(RES_Stat_Mul[it].wr_done == 1){
				RES_Stat_Mul[it].wr_done = 0;
				res_yes_mul = 0;
				continue;
			}
			current_R_mul = it;
			res_yes_mul = 1;
		}
		else{
			res_yes_mul = 0;
		}
	}

	for(unsigned i = 0; i < RES_Stat_Mul.size(); i++){
		if(RES_Stat_Mul[i].res_busy!=1){
			if(RES_Stat_Mul[i].use_ex == 1){
				RES_Stat_Mul[i].use_ex = 0;
				res_yes_mul = 0;
				continue;
			}
			current_R_mul = i;
			res_yes_mul = 1;
			break;
		}
		else{
			res_yes_mul = 0;
		}
	}

	for(unsigned it = 1; it < RES_Stat_Add.size(); it++){
		if(RES_Stat_Add[it].res_busy!=1){
			if(RES_Stat_Add[it].wr_done == 1){
				RES_Stat_Add[it].wr_done = 0;
				res_yes_add = 0;
				continue;
			}
			current_R_add = it;
			res_yes_add = 1;
		}
		else{
			res_yes_add = 0;
		}
	}

	for(unsigned i = 0; i < RES_Stat_Add.size(); i++){
		if(RES_Stat_Add[i].res_busy!=1){
			if(RES_Stat_Add[i].use_ex == 1){
				RES_Stat_Add[i].use_ex = 0;
				res_yes_add = 0;
				continue;
			}
			current_R_add = i;
			res_yes_add = 1;
			break;
		}
		else{
			res_yes_add = 0;
		}
	}

	for(unsigned it = 1; it < RES_Stat_Load.size(); it++){
		if(RES_Stat_Load[it].res_busy!=1){
			if(RES_Stat_Load[it].wr_done == 1){
				RES_Stat_Load[it].wr_done = 0;
				res_yes_load = 0;
				continue;
			}
			current_R_ld = it;
			res_yes_load = 1;
		}
		else{
			res_yes_load = 0;
		}
	}

	for(unsigned i = 0; i < RES_Stat_Load.size(); i++){
		if(RES_Stat_Load[i].res_busy!=1){
			if(RES_Stat_Load[i].use_ex == 1){
				RES_Stat_Load[i].use_ex = 0;
				res_yes_load = 0;
				continue;
			}
			current_R_ld = i;
			res_yes_load = 1;
			break;
		}
		else{
			res_yes_load = 0;
		}
	}
	std::cout << "ROB_Available: " << rob_yes << " at position: " << current_B <<'\n';
	std::cout << "RESERVATION STATION Available for INT: " << res_yes_int << '\n';
	std::cout << "RESERVATION STATION Available for MUL: " << res_yes_mul << '\n';
	std::cout << "RESERVATION STATION Available for ADD: " << res_yes_add << '\n';
	std::cout << "RESERVATION STATION Available for MEM: " << res_yes_load << '\n';

	std::cout << "OPCODE in ISSUE:" << instruction_memory[temp_PC].op_code << '\n';
	std::cout << "TEMP_PC: " << temp_PC << '\n';
  switch (opcode_map[instruction_memory[temp_PC].op_code]) {
    case XOR:
  	case ADD:
  	case SUB:
  	case OR:
  	case AND:
			if(res_yes_int == 1 && rob_yes == 1){
				issue_success = 1;
				for(unsigned iter = 0; iter < int_ex_done.size(); iter++){
					if(int_ex_done[iter] == 1){
						int_ex_done[iter] = 0;
					}
				}
      	strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand1.c_str());
	      hold_val1 = atoi(operands_needed+1);
	      strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand2.c_str());
	      hold_val2 = atoi(operands_needed+1);
	      if(int_reg_stat[hold_val1].reg_busy){
	        h_val = int_reg_stat[hold_val1].ROB_num;
					if(ROB_table[h_val].ready){
						RES_Stat_Int[current_R_int].Vj = ROB_table[h_val].rob_val;
						RES_Stat_Int[current_R_int].Qj = 0;
						RES_Stat_Int[current_R_int].if_depend_Vj = 0;
					}
					else{
						RES_Stat_Int[current_R_int].if_depend_Vj = 1;
						RES_Stat_Int[current_R_int].Qj = h_val+1;
					}
	      }
				else{
					std::cout << "Value loaded: " << register_file[hold_val1] << '\n';
					RES_Stat_Int[current_R_int].Vj = register_file[hold_val1];
					std::cout << "Vj: " << RES_Stat_Int[current_R_int].Vj << '\n';
					RES_Stat_Int[current_R_int].Qj = 0;
					RES_Stat_Int[current_R_int].if_depend_Vj = 0;
				}
				RES_Stat_Int[current_R_int].res_busy = 1;
				RES_Stat_Int[current_R_int].res_dest = current_B;
				ROB_table[current_B].rob_busy = 1;
				ROB_table[current_B].rob_pc = temp_PC*4 + base_add;
				ROB_table[current_B].rob_temp_pc = temp_PC;
				ROB_table[current_B].state = ISSUE;
				if(int_reg_stat[hold_val2].reg_busy){
					h_val = int_reg_stat[hold_val2].ROB_num;
					if(ROB_table[h_val].ready){
						RES_Stat_Int[current_R_int].Vk = ROB_table[h_val].rob_val;
						RES_Stat_Int[current_R_int].Qk = 0;
						RES_Stat_Int[current_R_int].if_depend_Vk = 0;
					}
					else{
						RES_Stat_Int[current_R_int].if_depend_Vk = 1;
						RES_Stat_Int[current_R_int].Qk = h_val+1;
					}
				}
				else{
					RES_Stat_Int[current_R_int].Vk = register_file[hold_val2];
					std::cout << "Vk: " << RES_Stat_Int[current_R_int].Vk << '\n';
					RES_Stat_Int[current_R_int].Qk = 0;
					RES_Stat_Int[current_R_int].if_depend_Vk = 0;
				}
				strcpy(operands_needed,instruction_memory[temp_PC].mand_operand.c_str());
	      hold_val = atoi(operands_needed+1);
				int_reg_stat[hold_val].ROB_num = current_B;
				int_reg_stat[hold_val].reg_busy = 1;
				ROB_table[current_B].rob_dest = hold_val;
				ROB_table[current_B].dest_type = INTEGER_RES;
				RES_Stat_Int[current_R_int].res_dest_type = INTEGER_RES;
				RES_Stat_Int[current_R_int].op = INTEGER;
				RES_Stat_Int[current_R_int].op_num = temp_PC;
				RES_Stat_Int[current_R_int].res_pc = temp_PC*4 + base_add;
			}
			else{
				issue_success = 0;
			}

			break;

		case MULT:
			if(res_yes_mul == 1 && rob_yes == 1){
				issue_success = 1;
				for(unsigned iter = 0; iter < mul_ex_done.size(); iter++){
					if(mul_ex_done[iter] == 1){
						mul_ex_done[iter] = 0;
					}
				}
				strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand1.c_str());
				hold_val1 = atoi(operands_needed+1);ROB_table[current_B].rob_busy = 1;
				strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand2.c_str());
				hold_val2 = atoi(operands_needed+1);
				if(int_reg_stat[hold_val1].reg_busy){
					h_val = int_reg_stat[hold_val1].ROB_num;
					if(ROB_table[h_val].ready){
						RES_Stat_Mul[current_R_mul].Vj = ROB_table[h_val].rob_val;
						RES_Stat_Mul[current_R_mul].Qj = 0;
						RES_Stat_Mul[current_R_mul].if_depend_Vj = 0;
					}
					else{
						RES_Stat_Mul[current_R_mul].Qj = h_val+1;
						RES_Stat_Mul[current_R_mul].if_depend_Vj = 1;
					}
				}
				else{
					RES_Stat_Mul[current_R_mul].Vj = register_file[hold_val1];
					RES_Stat_Mul[current_R_mul].Qj = 0;
					RES_Stat_Mul[current_R_mul].if_depend_Vj = 0;
				}
				RES_Stat_Mul[current_R_mul].res_busy = 1;
				RES_Stat_Mul[current_R_mul].res_dest = current_B;
				ROB_table[current_B].rob_pc = temp_PC*4 + base_add;
				ROB_table[current_B].rob_temp_pc = temp_PC;
				ROB_table[current_B].state = ISSUE;
				ROB_table[current_B].rob_busy = 1;

				if(int_reg_stat[hold_val2].reg_busy){
					h_val = int_reg_stat[hold_val2].ROB_num;
					if(ROB_table[h_val].ready){
						RES_Stat_Mul[current_R_mul].Vk = ROB_table[h_val].rob_val;
						RES_Stat_Mul[current_R_mul].Qk = 0;
						RES_Stat_Mul[current_R_mul].if_depend_Vk = 0;
					}
					else{
						RES_Stat_Mul[current_R_mul].Qk = h_val+1;
						RES_Stat_Mul[current_R_mul].if_depend_Vk = 1;
					}
				}
				else{
					RES_Stat_Mul[current_R_mul].Vk = register_file[hold_val2];
					RES_Stat_Mul[current_R_mul].Qk = 0;
					RES_Stat_Mul[current_R_mul].if_depend_Vk = 0;
				}
				strcpy(operands_needed,instruction_memory[temp_PC].mand_operand.c_str());
				hold_val = atoi(operands_needed+1);
				int_reg_stat[hold_val].ROB_num = current_B;
				int_reg_stat[hold_val].reg_busy = 1;
				ROB_table[current_B].rob_dest = hold_val;
				ROB_table[current_B].dest_type = INTEGER_RES;
				RES_Stat_Mul[current_R_mul].res_dest_type = INTEGER_RES;
				RES_Stat_Mul[current_R_mul].op = MULTIPLIER;
				RES_Stat_Mul[current_R_mul].op_num = temp_PC;
				RES_Stat_Mul[current_R_mul].res_pc = temp_PC*4 + base_add;
			}
			else{
				issue_success = 0;
			}

			break;

		case DIV:
			if(res_yes_mul == 1 && rob_yes == 1){
				issue_success = 1;
				for(unsigned iter = 0; iter < div_ex_done.size(); iter++){
					if(div_ex_done[iter] == 1){
						div_ex_done[iter] = 0;
					}
				}
				strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand1.c_str());
				hold_val1 = atoi(operands_needed+1);
				strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand2.c_str());
				hold_val2 = atoi(operands_needed+1);
				if(int_reg_stat[hold_val1].reg_busy){
					h_val = int_reg_stat[hold_val1].ROB_num;
					if(ROB_table[h_val].ready){
						RES_Stat_Mul[current_R_mul].Vj = ROB_table[h_val].rob_val;
						RES_Stat_Mul[current_R_mul].Qj = 0;
						RES_Stat_Mul[current_R_mul].if_depend_Vj = 0;
					}
					else{
						RES_Stat_Mul[current_R_mul].Qj = h_val+1;
						RES_Stat_Mul[current_R_mul].if_depend_Vj = 1;
					}
				}
				else{
					RES_Stat_Mul[current_R_mul].Vj = register_file[hold_val1];
					RES_Stat_Mul[current_R_mul].Qj = 0;
					RES_Stat_Mul[current_R_mul].if_depend_Vj = 0;
				}
				RES_Stat_Mul[current_R_mul].res_busy = 1;
				RES_Stat_Mul[current_R_mul].res_dest = current_B;
				ROB_table[current_B].rob_temp_pc = temp_PC;
				ROB_table[current_B].rob_pc = temp_PC*4 + base_add;
				ROB_table[current_B].state = ISSUE;
				ROB_table[current_B].rob_busy = 1;

				if(int_reg_stat[hold_val2].reg_busy){
					h_val = int_reg_stat[hold_val2].ROB_num;
					if(ROB_table[h_val].ready){
						RES_Stat_Mul[current_R_mul].Vk = ROB_table[h_val].rob_val;
						RES_Stat_Mul[current_R_mul].Qk = 0;
						RES_Stat_Mul[current_R_mul].if_depend_Vk = 0;
					}
					else{
						RES_Stat_Mul[current_R_mul].Qk = h_val+1;
						RES_Stat_Mul[current_R_mul].if_depend_Vk = 1;
					}
				}
				else{
					RES_Stat_Mul[current_R_mul].Vk = register_file[hold_val2];
					RES_Stat_Mul[current_R_mul].Qk = 0;
					RES_Stat_Mul[current_R_mul].if_depend_Vk = 0;
				}
				strcpy(operands_needed,instruction_memory[temp_PC].mand_operand.c_str());
				hold_val = atoi(operands_needed+1);
				int_reg_stat[hold_val].ROB_num = current_B;
				int_reg_stat[hold_val].reg_busy = 1;
				ROB_table[current_B].rob_dest = hold_val;
				ROB_table[current_B].dest_type = INTEGER_RES;
				RES_Stat_Mul[current_R_mul].res_dest_type = INTEGER_RES;
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
				for(unsigned iter = 0; iter < int_ex_done.size(); iter++){
					if(int_ex_done[iter] == 1){
						std::cout << "Made the ex unit available" << '\n';
						int_ex_done[iter] = 0;
					}
				}
				issue_success = 1;
				strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand1.c_str());
	      hold_val1 = atoi(operands_needed+1);
	      if(int_reg_stat[hold_val1].reg_busy){
	        h_val = int_reg_stat[hold_val1].ROB_num;
					if(ROB_table[h_val].ready){
						RES_Stat_Int[current_R_int].Vj = ROB_table[h_val].rob_val;
						RES_Stat_Int[current_R_int].Qj = 0;
						RES_Stat_Int[current_R_int].if_depend_Vj = 0;
					}
					else{
						RES_Stat_Int[current_R_int].Qj = h_val+1;
						RES_Stat_Int[current_R_int].if_depend_Vj = 1;
					}
	      }
				else{
					RES_Stat_Int[current_R_int].Vj = register_file[hold_val1];
					RES_Stat_Int[current_R_int].Qj = 0;
					RES_Stat_Int[current_R_int].if_depend_Vj = 0;
				}
				RES_Stat_Int[current_R_int].res_busy = 1;
				RES_Stat_Int[current_R_int].res_dest = current_B;
				ROB_table[current_B].rob_pc = temp_PC*4 + base_add;
				ROB_table[current_B].rob_temp_pc = temp_PC;
				ROB_table[current_B].state = ISSUE;
				ROB_table[current_B].rob_busy = 1;

				strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand2.c_str());
				imm = strtoul(operands_needed, &endptr, 0);
				RES_Stat_Int[current_R_int].imm_val = imm;
				RES_Stat_Int[current_R_int].Qk = 0;
				RES_Stat_Int[current_R_int].if_depend_Vk = 0;

				strcpy(operands_needed,instruction_memory[temp_PC].mand_operand.c_str());
				hold_val = atoi(operands_needed+1);
				int_reg_stat[hold_val].ROB_num = current_B;
				int_reg_stat[hold_val].reg_busy = 1;
				ROB_table[current_B].rob_dest = hold_val;
				ROB_table[current_B].dest_type = INTEGER_RES;
				RES_Stat_Int[current_R_int].op = INTEGER;
				RES_Stat_Int[current_R_int].res_dest_type = INTEGER_RES;
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
				for(unsigned iter = 0; iter < add_ex_done.size(); iter++){
					if(add_ex_done[iter] == 1){
						add_ex_done[iter] = 0;
					}
				}
				strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand1.c_str());
				hold_val1 = atoi(operands_needed+1);
				strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand2.c_str());
	      hold_val2 = atoi(operands_needed+1);
	      if(float_reg_stat[hold_val1].reg_busy){
	        h_val = float_reg_stat[hold_val1].ROB_num;
					if(ROB_table[h_val].ready){
						RES_Stat_Add[current_R_add].Vj = ROB_table[h_val].rob_val;
						RES_Stat_Add[current_R_add].Qj = 0;
						RES_Stat_Add[current_R_add].if_depend_Vj = 0;
					}
					else{
						RES_Stat_Add[current_R_add].Qj = h_val+1;
						RES_Stat_Add[current_R_add].if_depend_Vj = 1;
					}
	      }
				else{
					RES_Stat_Add[current_R_add].Vj = float2unsigned(float(float_reg_file[hold_val1]));
					RES_Stat_Add[current_R_add].Qj = 0;
					RES_Stat_Add[current_R_add].if_depend_Vj = 0;
				}
				RES_Stat_Add[current_R_add].res_busy = 1;
				RES_Stat_Add[current_R_add].res_dest = current_B;
				ROB_table[current_B].rob_pc = temp_PC*4 + base_add;
				ROB_table[current_B].rob_temp_pc = temp_PC;
				ROB_table[current_B].state = ISSUE;
				ROB_table[current_B].rob_busy = 1;

				if(float_reg_stat[hold_val2].reg_busy){
					std::cout << "The register was busy" << '\n';
					h_val = float_reg_stat[hold_val2].ROB_num;
					std::cout << "The register is dependent on ROB: " << h_val << '\n';
					if(ROB_table[h_val].ready){
						RES_Stat_Add[current_R_add].Vk = ROB_table[h_val].rob_val;
						RES_Stat_Add[current_R_add].Qk = 0;
						RES_Stat_Add[current_R_add].if_depend_Vk = 0;
					}
					else{
						RES_Stat_Add[current_R_add].Qk = h_val+1;
						RES_Stat_Add[current_R_add].if_depend_Vk = 1;
					}
				}
				else{
					RES_Stat_Add[current_R_add].Vk = float2unsigned(float(float_reg_file[hold_val2]));
					RES_Stat_Add[current_R_add].Qk = 0;
					RES_Stat_Add[current_R_add].if_depend_Vk = 0;
				}

				strcpy(operands_needed,instruction_memory[temp_PC].mand_operand.c_str());
				hold_val = atoi(operands_needed+1);
				float_reg_stat[hold_val].ROB_num = current_B;
				float_reg_stat[hold_val].reg_busy = 1;
				ROB_table[current_B].rob_dest = hold_val;
				ROB_table[current_B].dest_type = FLOAT_RES;
				RES_Stat_Add[current_R_add].res_dest_type = FLOAT_RES;
				RES_Stat_Add[current_R_add].op = ADDER;
				RES_Stat_Add[current_R_add].op_num = temp_PC;
				RES_Stat_Add[current_R_add].res_pc = temp_PC*4 + base_add;
			}
			else{
				issue_success = 0;
			}
			break;

		case MULTS:
			std::cout << "Came here in MULTS" << '\n';
			if(res_yes_mul == 1 && rob_yes == 1){
				issue_success = 1;
				for(unsigned iter = 0; iter < mul_ex_done.size(); iter++){
					if(mul_ex_done[iter] == 1){
						mul_ex_done[iter] = 0;
					}
				}
				strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand1.c_str());
				hold_val1 = atoi(operands_needed+1);
				strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand2.c_str());
				hold_val2 = atoi(operands_needed+1);
				if(float_reg_stat[hold_val1].reg_busy){
					h_val = float_reg_stat[hold_val1].ROB_num;
					if(ROB_table[h_val].ready){
						RES_Stat_Mul[current_R_mul].Vj = ROB_table[h_val].rob_val;
						RES_Stat_Mul[current_R_mul].Qj = 0;
						RES_Stat_Mul[current_R_mul].if_depend_Vj = 0;
					}
					else{
						RES_Stat_Mul[current_R_mul].Qj = h_val+1;
						RES_Stat_Mul[current_R_mul].if_depend_Vj = 1;
					}
				}
				else{
					RES_Stat_Mul[current_R_mul].Vj = float2unsigned(float(float_reg_file[hold_val1]));
					RES_Stat_Mul[current_R_mul].Qj = 0;
					RES_Stat_Mul[current_R_mul].if_depend_Vj = 0;
				}
				RES_Stat_Mul[current_R_mul].res_busy = 1;
				RES_Stat_Mul[current_R_mul].res_dest = current_B;
				ROB_table[current_B].rob_pc = temp_PC*4 + base_add;
				ROB_table[current_B].rob_temp_pc = temp_PC;
				ROB_table[current_B].state = ISSUE;
				ROB_table[current_B].rob_busy = 1;

				if(float_reg_stat[hold_val2].reg_busy){
					h_val = float_reg_stat[hold_val2].ROB_num;
					if(ROB_table[h_val].ready){
						RES_Stat_Mul[current_R_mul].Vk = ROB_table[h_val].rob_val;
						RES_Stat_Mul[current_R_mul].Qk = 0;
						RES_Stat_Mul[current_R_mul].if_depend_Vk = 0;
					}
					else{
						RES_Stat_Mul[current_R_mul].Qk = h_val+1;
						RES_Stat_Mul[current_R_mul].if_depend_Vk = 1;
					}
				}
				else{
					RES_Stat_Mul[current_R_mul].Vk = float2unsigned(float(float_reg_file[hold_val2]));
					RES_Stat_Mul[current_R_mul].Qk = 0;
					RES_Stat_Mul[current_R_mul].if_depend_Vk = 0;
				}
				strcpy(operands_needed,instruction_memory[temp_PC].mand_operand.c_str());
				hold_val = atoi(operands_needed+1);
				float_reg_stat[hold_val].ROB_num = current_B;
				float_reg_stat[hold_val].reg_busy = 1;
				ROB_table[current_B].rob_dest = hold_val;
				ROB_table[current_B].dest_type = FLOAT_RES;
				RES_Stat_Mul[current_R_mul].op = MULTIPLIER;
				RES_Stat_Mul[current_R_mul].res_dest_type = FLOAT_RES;
				RES_Stat_Mul[current_R_mul].op_num = temp_PC;
				RES_Stat_Mul[current_R_mul].res_pc = temp_PC*4 + base_add;
			}
			else{
				issue_success = 0;
			}
			break;

		case DIVS:
			if(res_yes_mul == 1 && rob_yes == 1){
				issue_success = 1;
				for(unsigned iter = 0; iter < div_ex_done.size(); iter++){
					if(div_ex_done[iter] == 1){
						div_ex_done[iter] = 0;
					}
				}
				strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand1.c_str());
				hold_val1 = atoi(operands_needed+1);
				strcpy(operands_needed,instruction_memory[temp_PC].remainder_operand2.c_str());
				hold_val2 = atoi(operands_needed+1);
				if(float_reg_stat[hold_val1].reg_busy){
					h_val = float_reg_stat[hold_val1].ROB_num;
					if(ROB_table[h_val].ready){
						RES_Stat_Mul[current_R_mul].Vj = ROB_table[h_val].rob_val;
						RES_Stat_Mul[current_R_mul].Qj = 0;
						RES_Stat_Mul[current_R_mul].if_depend_Vj = 0;
					}
					else{
						RES_Stat_Mul[current_R_mul].Qj = h_val+1;
						RES_Stat_Mul[current_R_mul].if_depend_Vj = 1;
					}
				}
				else{
					RES_Stat_Mul[current_R_mul].Vj = float2unsigned(float(float_reg_file[hold_val1]));
					RES_Stat_Mul[current_R_mul].Qj = 0;
					RES_Stat_Mul[current_R_mul].if_depend_Vj = 0;
				}
				RES_Stat_Mul[current_R_mul].res_busy = 1;
				RES_Stat_Mul[current_R_mul].res_dest = current_B;
				RES_Stat_Mul[current_R_mul].res_dest_type = FLOAT_RES;
				ROB_table[current_B].rob_pc = temp_PC*4 + base_add;
				ROB_table[current_B].rob_temp_pc = temp_PC;
				ROB_table[current_B].state = ISSUE;
				ROB_table[current_B].rob_busy = 1;

				if(float_reg_stat[hold_val2].reg_busy){
					h_val = float_reg_stat[hold_val2].ROB_num;
					if(ROB_table[h_val].ready){
						RES_Stat_Mul[current_R_mul].Vk = ROB_table[h_val].rob_val;
						RES_Stat_Mul[current_R_mul].Qk = 0;
						RES_Stat_Mul[current_R_mul].if_depend_Vk = 0;
					}
					else{
						RES_Stat_Mul[current_R_mul].Qk = h_val+1;
						RES_Stat_Mul[current_R_mul].if_depend_Vk = 1;
					}
				}
				else{
					RES_Stat_Mul[current_R_mul].Vk = float2unsigned(float(float_reg_file[hold_val2]));
					RES_Stat_Mul[current_R_mul].Qk = 0;
					RES_Stat_Mul[current_R_mul].if_depend_Vk = 0;
				}
				strcpy(operands_needed,instruction_memory[temp_PC].mand_operand.c_str());
				hold_val = atoi(operands_needed+1);
				float_reg_stat[hold_val].ROB_num = current_B;
				float_reg_stat[hold_val].reg_busy = 1;
				ROB_table[current_B].rob_dest = hold_val;
				ROB_table[current_B].dest_type = FLOAT_RES;
				RES_Stat_Mul[current_R_mul].op = DIVIDER;
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
			std::cout << "RES_YES_INT: " << res_yes_int << '\n';
			if(res_yes_int == 1 && rob_yes == 1){
				for(unsigned iter = 0; iter < int_ex_done.size(); iter++){
					if(int_ex_done[iter] == 1){
						int_ex_done[iter] = 0;
					}
				}
				issue_success = 1;
				strcpy(operands_needed,instruction_memory[temp_PC].mand_operand.c_str());
				hold_val1 = atoi(operands_needed+1);
				if(int_reg_stat[hold_val1].reg_busy){
	        h_val = int_reg_stat[hold_val1].ROB_num;
					if(ROB_table[h_val].ready){
						RES_Stat_Int[current_R_int].Vj = ROB_table[h_val].rob_val;
						RES_Stat_Int[current_R_int].Qj = 0;
						RES_Stat_Int[current_R_int].if_depend_Vj = 0;
					}
					else{
						RES_Stat_Int[current_R_int].Qj = h_val+1;
						RES_Stat_Int[current_R_int].if_depend_Vj = 1;
					}
	      }
				else{
					RES_Stat_Int[current_R_int].Vj = register_file[hold_val1];
					RES_Stat_Int[current_R_int].Qj = 0;
					RES_Stat_Int[current_R_int].if_depend_Vj = 0;
				}
				RES_Stat_Int[current_R_int].res_busy = 1;
				RES_Stat_Int[current_R_int].res_dest = current_B;
				RES_Stat_Int[current_R_int].Qk = 0;
				RES_Stat_Int[current_R_int].if_depend_Vk = 0;
				ROB_table[current_B].rob_pc = temp_PC*4 + base_add;
				ROB_table[current_B].rob_temp_pc = temp_PC;
				RES_Stat_Int[current_R_int].res_dest_type = FLOAT_RES;
				ROB_table[current_B].state = ISSUE;
				ROB_table[current_B].rob_busy = 1;

				for(unsigned i = 0; i < instruction_memory.size(); i++){
					if(instruction_memory[temp_PC].remainder_operand1 == instruction_memory[i].branch_array){
						imm = instruction_memory[i].instr_address;
						check_branch = 1;
						RES_Stat_Int[current_R_int].target_address = imm;
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
				for(unsigned iter = 0; iter < ld_ex_done.size(); iter++){
					if(ld_ex_done[iter] == 1){
						ld_ex_done[iter] = 0;
					}
				}
				issue_success = 1;
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
	        h_val = int_reg_stat[hold_val1].ROB_num;
					if(ROB_table[h_val].ready){
						RES_Stat_Load[current_R_ld].Vj = ROB_table[h_val].rob_val;
						RES_Stat_Load[current_R_ld].Qj = 0;
						RES_Stat_Load[current_R_ld].if_depend_Vj = 0;
					}
					else{
						RES_Stat_Load[current_R_ld].Qj = h_val+1;
						RES_Stat_Load[current_R_ld].if_depend_Vj = 1;
					}
	      }
				else{
					RES_Stat_Load[current_R_ld].Vj = register_file[hold_val1];
					RES_Stat_Load[current_R_ld].Qj = 0;
					RES_Stat_Load[current_R_ld].if_depend_Vj = 0;
				}
				RES_Stat_Load[current_R_ld].Qk = 0;
				RES_Stat_Load[current_R_ld].if_depend_Vk = 0;
				RES_Stat_Load[current_R_ld].res_busy = 1;
				RES_Stat_Load[current_R_ld].res_dest = current_B;
				ROB_table[current_B].rob_busy = 1;
				ROB_table[current_B].rob_pc = temp_PC*4 + base_add;
				ROB_table[current_B].rob_temp_pc = temp_PC;
				ROB_table[current_B].state = ISSUE;

				strcpy(operands_needed, instruction_memory[temp_PC].mand_operand.c_str());
				hold_val = atoi(operands_needed+1);

				int_reg_stat[hold_val].ROB_num = current_B;
				int_reg_stat[hold_val].reg_busy = 1;
				ROB_table[current_B].rob_dest = hold_val;
				RES_Stat_Load[current_R_ld].res_dest_type = INTEGER_RES;
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
				issue_success = 1;
				for(unsigned iter = 0; iter < ld_ex_done.size(); iter++){
					if(ld_ex_done[iter] == 1){
						ld_ex_done[iter] = 0;
					}
				}
				strcpy(operands_needed, instruction_memory[temp_PC].remainder_operand1.c_str());
				pch1 = strtok(operands_needed, " ( ");
				hold_val_imm1 = atoi(pch1);
				std::cout << "hold_val_imm1: " << hold_val_imm1 << '\n';
				RES_Stat_Load[current_R_ld].res_A = hold_val_imm1;

				while(pch1 != NULL)
				{
					hold_val1 = atoi(pch1);
					pch1 = strtok(NULL," R,( ");
				}
				if(int_reg_stat[hold_val1].reg_busy){
	        h_val = int_reg_stat[hold_val1].ROB_num;
					if(ROB_table[h_val].ready){
						RES_Stat_Load[current_R_ld].Vj = ROB_table[h_val].rob_val;
						RES_Stat_Load[current_R_ld].Qj = 0;
						RES_Stat_Load[current_R_ld].if_depend_Vj = 0;
					}
					else{
						RES_Stat_Load[current_R_ld].Qj = h_val+1;
						RES_Stat_Load[current_R_ld].if_depend_Vj = 1;
					}
	      }

				else{
					RES_Stat_Load[current_R_ld].Vj = register_file[hold_val1];
					RES_Stat_Load[current_R_ld].Qj = 0;
					RES_Stat_Load[current_R_ld].if_depend_Vj = 0;
				}
				RES_Stat_Load[current_R_ld].Qk = 0;
				RES_Stat_Load[current_R_ld].if_depend_Vk = 0;
				RES_Stat_Load[current_R_ld].res_busy = 1;
				RES_Stat_Load[current_R_ld].res_dest = current_B;
				ROB_table[current_B].rob_pc = temp_PC*4 + base_add;
				ROB_table[current_B].rob_temp_pc = temp_PC;
				ROB_table[current_B].state = ISSUE;
				ROB_table[current_B].rob_busy = 1;

				strcpy(operands_needed, instruction_memory[temp_PC].mand_operand.c_str());
				hold_val = atoi(operands_needed+1);

				float_reg_stat[hold_val].ROB_num = current_B;
				float_reg_stat[hold_val].reg_busy = 1;
				ROB_table[current_B].rob_dest = hold_val;
				RES_Stat_Load[current_R_ld].res_dest_type = FLOAT_RES;
				ROB_table[current_B].dest_type = FLOAT_RES;
				RES_Stat_Load[current_R_ld].op = MEMORY;
				RES_Stat_Load[current_R_ld].op_num = temp_PC;
				std::cout << "Reservation station opcode number: " << RES_Stat_Load[current_R_ld].op_num << '\n';
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
				for(unsigned iter = 0; iter < ld_ex_done.size(); iter++){
					if(ld_ex_done[iter] == 1){
						ld_ex_done[iter] = 0;
					}
				}

				issue_success = 1;
				strcpy(operands_needed, instruction_memory[temp_PC].remainder_operand1.c_str());
				pch2 = strtok(operands_needed, " ( ");
				hold_val_imm2 = atoi(pch2);

				RES_Stat_Load[current_R_ld].res_A = hold_val_imm2;

				while(pch2 != NULL)
				{
					hold_val1 = atoi(pch2);
					pch2 = strtok(NULL," R,( ");
				}

				ROB_table[current_B].add_reg = hold_val1;

				if(int_reg_stat[hold_val1].reg_busy){
	        h_val = int_reg_stat[hold_val1].ROB_num;
					if(ROB_table[h_val].ready){
						RES_Stat_Load[current_R_ld].Vk = ROB_table[h_val].rob_val;
						RES_Stat_Load[current_R_ld].Qk = 0;
						RES_Stat_Load[current_R_ld].if_depend_Vk = 0;
						ROB_table[current_B].str_add = RES_Stat_Load[current_R_ld].Vk + RES_Stat_Load[current_R_ld].res_A;
					}
					else{
						RES_Stat_Load[current_R_ld].Vk = UNDEFINED;
						RES_Stat_Load[current_R_ld].Qk = h_val+1;
						RES_Stat_Load[current_R_ld].if_depend_Vk = 1;
						ROB_table[current_B].str_add = UNDEFINED;
					}
	      }
				else{
					RES_Stat_Load[current_R_ld].Vk = register_file[hold_val1];
					RES_Stat_Load[current_R_ld].Qk = 0;
					RES_Stat_Load[current_R_ld].if_depend_Vk = 0;
					ROB_table[current_B].str_add = RES_Stat_Load[current_R_ld].Vk + RES_Stat_Load[current_R_ld].res_A;
				}
				RES_Stat_Load[current_R_ld].res_busy = 1;
				RES_Stat_Load[current_R_ld].res_dest = current_B;
				ROB_table[current_B].rob_pc = temp_PC*4 + base_add;
				ROB_table[current_B].rob_temp_pc = temp_PC;
				ROB_table[current_B].state = ISSUE;
				ROB_table[current_B].rob_busy = 1;

				strcpy(operands_needed, instruction_memory[temp_PC].mand_operand.c_str());
				hold_val2 = atoi(operands_needed+1);
				if(int_reg_stat[hold_val2].reg_busy){
					h_val = int_reg_stat[hold_val2].ROB_num;
					if(ROB_table[h_val].ready){
						RES_Stat_Load[current_R_ld].Vj = ROB_table[h_val].rob_val;
						RES_Stat_Load[current_R_ld].Qj = 0;
						RES_Stat_Load[current_R_ld].if_depend_Vj = 0;
					}
					else{
						RES_Stat_Load[current_R_ld].Vj = UNDEFINED;
						RES_Stat_Load[current_R_ld].Qj = h_val+1;
						RES_Stat_Load[current_R_ld].if_depend_Vj = 1;
					}
				}
				else{
					RES_Stat_Load[current_R_ld].Vj = register_file[hold_val2];
					RES_Stat_Load[current_R_ld].Qj = 0;
					RES_Stat_Load[current_R_ld].if_depend_Vj = 0;
				}
				RES_Stat_Load[current_R_ld].res_dest_type = INTEGER_RES;
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
				issue_success = 1;
				for(unsigned iter = 0; iter < ld_ex_done.size(); iter++){
					if(ld_ex_done[iter] == 1){
						ld_ex_done[iter] = 0;
					}
				}
				strcpy(operands_needed, instruction_memory[temp_PC].remainder_operand1.c_str());
				pch3 = strtok(operands_needed, " ( ");
				hold_val_imm3 = atoi(pch3);

				RES_Stat_Load[current_R_ld].res_A = hold_val_imm3;

				while(pch3 != NULL)
				{
					hold_val1 = atoi(pch3);
					pch3 = strtok(NULL," R,( ");
				}

				ROB_table[current_B].add_reg = hold_val1;

				if(int_reg_stat[hold_val1].reg_busy){
	        h_val = int_reg_stat[hold_val1].ROB_num;
					if(ROB_table[h_val].ready){
						RES_Stat_Load[current_R_ld].Vk = ROB_table[h_val].rob_val;
						RES_Stat_Load[current_R_ld].Qk = 0;
						RES_Stat_Load[current_R_ld].if_depend_Vk = 0;
						ROB_table[current_B].str_add = RES_Stat_Load[current_R_ld].Vk + RES_Stat_Load[current_R_ld].res_A;
					}
					else{
						RES_Stat_Load[current_R_ld].Vk = UNDEFINED;
						RES_Stat_Load[current_R_ld].Qk = h_val+1;
						RES_Stat_Load[current_R_ld].if_depend_Vk = 1;
						ROB_table[current_B].str_add = UNDEFINED;
					}
	      }
				else{
					RES_Stat_Load[current_R_ld].Vk = register_file[hold_val1];
					RES_Stat_Load[current_R_ld].Qk = 0;
					RES_Stat_Load[current_R_ld].if_depend_Vk = 0;
					ROB_table[current_B].str_add = RES_Stat_Load[current_R_ld].Vk + RES_Stat_Load[current_R_ld].res_A;
				}
				RES_Stat_Load[current_R_ld].res_busy = 1;
				RES_Stat_Load[current_R_ld].res_dest = current_B;
				ROB_table[current_B].rob_pc = temp_PC*4 + base_add;
				ROB_table[current_B].rob_temp_pc = temp_PC;
				ROB_table[current_B].state = ISSUE;
				ROB_table[current_B].rob_busy = 1;

				strcpy(operands_needed, instruction_memory[temp_PC].mand_operand.c_str());
				hold_val2 = atoi(operands_needed+1);
				if(float_reg_stat[hold_val2].reg_busy){
					h_val = float_reg_stat[hold_val2].ROB_num;
					if(ROB_table[h_val].ready){
						RES_Stat_Load[current_R_ld].Vj = ROB_table[h_val].rob_val;
						RES_Stat_Load[current_R_ld].Qj = 0;
						RES_Stat_Load[current_R_ld].if_depend_Vj = 0;
					}
					else{
						RES_Stat_Load[current_R_ld].Vj = UNDEFINED;
						RES_Stat_Load[current_R_ld].Qj = h_val+1;
						RES_Stat_Load[current_R_ld].if_depend_Vj = 1;
					}
				}
				else{
					RES_Stat_Load[current_R_ld].Vj = float_reg_file[hold_val2];
					RES_Stat_Load[current_R_ld].Qj = 0;
					RES_Stat_Load[current_R_ld].if_depend_Vj = 0;
				}
				RES_Stat_Load[current_R_ld].res_dest_type = FLOAT_RES;
				RES_Stat_Load[current_R_ld].op = MEMORY;
				RES_Stat_Load[current_R_ld].op_num = temp_PC;
				RES_Stat_Load[current_R_ld].res_pc = temp_PC*4 + base_add;
			}

			else{
				issue_success = 0;
			}
			break;

		case EOP:
			check_end = 1;
			issue_success = 0;
			break;

		default:
			break;
  }
}

void sim_ooo::execute_ins(){
	std::cout << "****************EXECUTE*****************" << '\n';
	unsigned hold_int;
	unsigned hold_add;
	unsigned hold_mul;
	unsigned hold_div;
	unsigned hold_mem;
	float temp_adds;
	float temp_divs;
	float temp_muls;

	for(unsigned i = 0 ; i < RES_Stat_Int.size(); i++){
		if(RES_Stat_Int[i].res_busy == 1){
			if((RES_Stat_Int[i].Qj == 0) && (RES_Stat_Int[i].Qk == 0)){
				if(RES_Stat_Int[i].if_depend_Vj == 1 || RES_Stat_Int[i].if_depend_Vk == 1){
					RES_Stat_Int[i].if_depend_Vj = 0;
					RES_Stat_Int[i].if_depend_Vk = 0;
					for(unsigned it = 0; it < int_ex_done.size(); it++){
						if(int_ex_done[it] == 1){
							int_ex_done[it] = 0;
						}
					}
					continue;
				}
				switch (RES_Stat_Int[i].op) {
					case INTEGER:
						for(unsigned sel_int_unit = 0; sel_int_unit < big_int.size(); sel_int_unit++){
							if((big_int[sel_int_unit].status == 1) && (ROB_table[RES_Stat_Int[i].res_dest].state == ISSUE)){
								if(int_ex_done[sel_int_unit] == 1){
									int_ex_done[sel_int_unit] = 0;
									continue;
								}
								ROB_table[RES_Stat_Int[i].res_dest].state = EXECUTE;
								RES_Stat_Int[i].use_ex = 1;
                RES_Stat_Int[i].execution_unit_used = sel_int_unit;
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
				std::cout << "Latency of Int: " << num_latency_int[sel_int_unit] << '\n';
				num_latency_int[sel_int_unit]--;
				flag_int[big_int[sel_int_unit].res_stat_num] = 0;
			}
			else{
				flag_int[big_int[sel_int_unit].res_stat_num] = 1;
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
					int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vj + RES_Stat_Int[big_int[sel_int_unit].res_stat_num].imm_val;
					break;

				case SUBI:
					int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vj - RES_Stat_Int[big_int[sel_int_unit].res_stat_num].imm_val;
					break;

				case XORI:
					int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vj ^ RES_Stat_Int[big_int[sel_int_unit].res_stat_num].imm_val;
					break;

				case ORI:
					int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vj | RES_Stat_Int[big_int[sel_int_unit].res_stat_num].imm_val;
					break;

				case ANDI:
					int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vj & RES_Stat_Int[big_int[sel_int_unit].res_stat_num].imm_val;
					break;

				case BEQZ:
					if(RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vj == 0){
						RES_Stat_Int[big_int[sel_int_unit].res_stat_num].if_branch = 1;
						int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].target_address;
					}
					else{
						RES_Stat_Int[big_int[sel_int_unit].res_stat_num].if_branch = 0;
						branch_cond = 0;
						int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].res_pc + 4;
					}
					break;

				case BNEZ:
					if(RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vj != 0){
						RES_Stat_Int[big_int[sel_int_unit].res_stat_num].if_branch = 1;
						int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].target_address;
					}
					else{
						RES_Stat_Int[big_int[sel_int_unit].res_stat_num].if_branch = 0;
						branch_cond = 0;
						int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].res_pc + 4;
					}
					break;

				case BLTZ:
					if(RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vj < 0){
						RES_Stat_Int[big_int[sel_int_unit].res_stat_num].if_branch = 1;
						int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].target_address;
					}
					else{
						RES_Stat_Int[big_int[sel_int_unit].res_stat_num].if_branch = 0;
						int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].res_pc + 4;
					}
					break;

				case BGTZ:
					if(RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vj > 0){
						RES_Stat_Int[big_int[sel_int_unit].res_stat_num].if_branch = 1;
						int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].target_address;
					}
					else{
						RES_Stat_Int[big_int[sel_int_unit].res_stat_num].if_branch = 0;
						int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].res_pc + 4;
					}
					break;

				case BLEZ:
					if(RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vj <= 0){
						int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].target_address;
					}
					else{
						RES_Stat_Int[big_int[sel_int_unit].res_stat_num].if_branch = 0;
						int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].res_pc + 4;
					}
					break;

				case BGEZ:
					if(RES_Stat_Int[big_int[sel_int_unit].res_stat_num].Vj >= 0){
						RES_Stat_Int[big_int[sel_int_unit].res_stat_num].if_branch = 1;
						int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].target_address;
					}
					else{
						RES_Stat_Int[big_int[sel_int_unit].res_stat_num].if_branch = 0;
						int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].res_pc + 4;
					}
					break;

				case JUMP:
					RES_Stat_Int[big_int[sel_int_unit].res_stat_num].if_branch = 1;
					int_result[big_int[sel_int_unit].res_stat_num] = RES_Stat_Int[big_int[sel_int_unit].res_stat_num].target_address;
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
				int_ex_done[sel_int_unit] = 1;
				//RES_Stat_Int[big_int[sel_int_unit].res_stat_num].use_ex = UNDEFINED;
			}
			//break;
		}
		else{
			int_struc_hazard = 1;
		}
	}
	for(unsigned i = 0; i < RES_Stat_Add.size(); i++){
		std::cout << "Iterator ADD: " << i << '\n';
		if(RES_Stat_Add[i].res_busy == 1){
			if((RES_Stat_Add[i].Qj == 0) && (RES_Stat_Add[i].Qk == 0)){
				if(RES_Stat_Add[i].if_depend_Vj == 1 || RES_Stat_Add[i].if_depend_Vk == 1){
					RES_Stat_Add[i].if_depend_Vj = 0;
					RES_Stat_Add[i].if_depend_Vk = 0;
					for(unsigned it = 0; it < add_ex_done.size(); it++){
						if(add_ex_done[it] == 1){
							add_ex_done[it] = 0;
						}
					}
					std::cout << "Waited for one clock cycle" << '\n';
					continue;
				}
				switch (RES_Stat_Add[i].op){
					case ADDER:
						for(unsigned sel_add_unit = 0; sel_add_unit < big_add.size(); sel_add_unit++){
							std::cout << "Sel_Add_Unit: " << sel_add_unit << '\n';
							std::cout << "Status: " << big_add[sel_add_unit].status << '\n';
							if((big_add[sel_add_unit].status == 1) && (ROB_table[RES_Stat_Add[i].res_dest].state == ISSUE)){
								if(add_ex_done[sel_add_unit] == 1){
									std::cout << "Also waited here for one clock cycle" << '\n';
									add_ex_done[sel_add_unit] = 0;
									continue;
								}

								std::cout << "Into Execute" << '\n';
                RES_Stat_Add[i].execution_unit_used = sel_add_unit;
								ROB_table[RES_Stat_Add[i].res_dest].state = EXECUTE;
								RES_Stat_Add[i].use_ex = 1;
								big_add[sel_add_unit].status = 0;
								big_add[sel_add_unit].res_stat_num = i;
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
		std::cout << "Execution loop of add: " << sel_add_unit << '\n';
		std::cout << "Status of adder unit: " << big_add[sel_add_unit].status << '\n';
		if(big_add[sel_add_unit].status == 0){
			if(num_latency_add[sel_add_unit] > 1){
				std::cout << "Latency of add unit: " << num_latency_add[sel_add_unit] << '\n';
				num_latency_add[sel_add_unit]--;
				flag_add[big_add[sel_add_unit].res_stat_num] = 0;
			}
			else{
				std::cout << "Executing the adder unit" << '\n';
				flag_add[big_add[sel_add_unit].res_stat_num] = 1;
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
					add_ex_done[sel_add_unit] = 1;
				}
			}
	}

	for(unsigned i = 0; i < RES_Stat_Mul.size(); i++){
		std::cout << "Iterating in Mul res station for assignment" << '\n';
		if(RES_Stat_Mul[i].res_busy == 1){
			std::cout << "Found this busy" << '\n';
			if((RES_Stat_Mul[i].Qj == 0) && (RES_Stat_Mul[i].Qk == 0)){
				if(RES_Stat_Mul[i].if_depend_Vj == 1 || RES_Stat_Mul[i].if_depend_Vk == 1){
					RES_Stat_Mul[i].if_depend_Vj = 0;
					RES_Stat_Mul[i].if_depend_Vk = 0;
					for(unsigned it = 0; it < mul_ex_done.size(); it++){
						if(mul_ex_done[it] == 1){
							mul_ex_done[it] = 0;
						}
					}
					continue;
				}
				std::cout << "No dependencies whatsoever" << '\n';
				std::cout << "Divider or Mult??" << RES_Stat_Mul[i].op << '\n';
				switch (RES_Stat_Mul[i].op) {
					case MULTIPLIER:
						for(unsigned sel_mul_unit = 0; sel_mul_unit < big_mul.size(); sel_mul_unit++){
							if((big_mul[sel_mul_unit].status == 1) && (ROB_table[RES_Stat_Mul[i].res_dest].state == ISSUE)){
								if(mul_ex_done[sel_mul_unit] == 1){
									mul_ex_done[sel_mul_unit] = 0;
									continue;
								}

								big_mul[sel_mul_unit].status = 0;
								ROB_table[RES_Stat_Mul[i].res_dest].state = EXECUTE;
								RES_Stat_Mul[i].use_ex = 1;
								big_mul[sel_mul_unit].res_stat_num = i;
                RES_Stat_Mul[i].execution_unit_used = sel_mul_unit;
								break;
							}
						}
						break;

					case DIVIDER:
						for(unsigned sel_div_unit = 0; sel_div_unit < big_div.size(); sel_div_unit++){
							std::cout << "In Div iterator" << '\n';
							if(big_div[sel_div_unit].status == 1 && ROB_table[RES_Stat_Mul[i].res_dest].state == ISSUE){
								std::cout << "Checking if div assignment possible" << '\n';
								if(div_ex_done[sel_div_unit] == 1){
									div_ex_done[sel_div_unit] = 0;
									break;
								}

								std::cout << "No problems in assigning" << '\n';
								big_div[sel_div_unit].status = 0;
								ROB_table[RES_Stat_Mul[i].res_dest].state = EXECUTE;
								big_div[sel_div_unit].res_stat_num = i;
								continue;
							}
              RES_Stat_Mul[i].execution_unit_used = sel_div_unit;
							big_div[sel_div_unit].status = 0;
							ROB_table[RES_Stat_Mul[i].res_dest].state = EXECUTE;
							RES_Stat_Mul[i].use_ex = 1;
							big_div[sel_div_unit].res_stat_num = i;
							break;
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
				flag_mul[big_mul[sel_mul_unit].res_stat_num] = 0;
			}
			else{
				std::cout << "In for execution in multiplier" << '\n';
				flag_mul[big_mul[sel_mul_unit].res_stat_num] = 1;
				switch (opcode_map[instruction_memory[RES_Stat_Mul[big_mul[sel_mul_unit].res_stat_num].op_num].op_code]) {
					case MULT:
						mul_result[big_mul[sel_mul_unit].res_stat_num] = RES_Stat_Mul[big_mul[sel_mul_unit].res_stat_num].Vj * RES_Stat_Mul[big_mul[sel_mul_unit].res_stat_num].Vk;
						break;

			  	case MULTS:
						temp_muls = unsigned2float(RES_Stat_Mul[big_mul[sel_mul_unit].res_stat_num].Vj) * unsigned2float(RES_Stat_Mul[big_mul[sel_mul_unit].res_stat_num].Vk);
						mul_result[big_mul[sel_mul_unit].res_stat_num] = float2unsigned(float(temp_muls));
						break;
					}
					for(unsigned a = 0; a < create_exe_unit.size(); a++){
						if(create_exe_unit[a].exe_unit == MULTIPLIER){
							hold_mul = a;
						}
					}
					num_latency_mul[sel_mul_unit] = create_exe_unit[hold_mul].latency_exe;
				  big_mul[sel_mul_unit].status = 1;
					mul_ex_done[sel_mul_unit] = 1;
			}
		}
	}
	for(unsigned sel_div_unit = 0; sel_div_unit < big_div.size(); sel_div_unit++){
		if(big_div[sel_div_unit].status == 0){
			if(num_latency_div[sel_div_unit] > 1){
				num_latency_div[sel_div_unit]--;
				flag_mul[big_div[sel_div_unit].res_stat_num] = 0;
			}
			else{
				flag_mul[big_div[sel_div_unit].res_stat_num] = 1;
				switch (opcode_map[instruction_memory[RES_Stat_Mul[big_div[sel_div_unit].res_stat_num].op_num].op_code]) {
					case DIV:
						mul_result[big_div[sel_div_unit].res_stat_num] = RES_Stat_Mul[big_div[sel_div_unit].res_stat_num].Vj / RES_Stat_Mul[big_div[sel_div_unit].res_stat_num].Vk;
						break;

					case DIVS:
						temp_divs = unsigned2float(RES_Stat_Mul[big_div[sel_div_unit].res_stat_num].Vj) / unsigned2float(RES_Stat_Mul[big_div[sel_div_unit].res_stat_num].Vk);
						mul_result[big_div[sel_div_unit].res_stat_num] = float2unsigned(float(temp_divs));
						break;
				}
				for(unsigned a = 0; a < create_exe_unit.size(); a++){
					if(create_exe_unit[a].exe_unit == DIVIDER){
						hold_div = a;
					}
				}
				num_latency_div[sel_div_unit] = create_exe_unit[hold_div].latency_exe;
				big_div[sel_div_unit].status = 1;
				div_ex_done[sel_div_unit] = 1;
			}
		}
	}
	unsigned store_check = 0;
	//unsigned ld_step1;
	unsigned sw_check_head;

	for(unsigned i = 0; i < RES_Stat_Load.size(); i++){
		if(RES_Stat_Load[i].res_busy == 1){
			if(ld_step1[i] == 0){
				switch (opcode_map[instruction_memory[RES_Stat_Load[i].op_num].op_code]) {
					case LW:
					case LWS:
						std::cout << "Calculating PC: " << RES_Stat_Load[i].res_pc << '\n';
						if((ROB_table[RES_Stat_Load[i].res_dest].entry - 1) >= (head_ROB)){
							for(int a = (ROB_table[RES_Stat_Load[i].res_dest].entry - 1); a >= head_ROB ; a--){
								//std::cout << "Value of a in this condition: " << a << '\n';
								if((opcode_map[instruction_memory[ROB_table[a].rob_temp_pc].op_code] == SW)
								||(opcode_map[instruction_memory[ROB_table[a].rob_temp_pc].op_code] == SWS)){
									//std::cout << "There is a store" << '\n';
									if(int_reg_stat[ROB_table[a].add_reg].reg_busy != 1){
										store_check = 1;
										std::cout << "Store detected and it does not have indeterminate address" << '\n';
										break;
									}
									else{
										std::cout << "Busy stores present in Load Step 1" << '\n';
										store_check = 0;
									}
                  break;
								}
								else{
									std::cout << "No stores in Load Step 1" << '\n';
									store_check = 1;
								}
							}
						}
						else{
							for(int a = (ROB_table[RES_Stat_Load[i].res_dest].entry - 1); a >= 0; a--){
								//std::cout << "Value of a: " << a << '\n';
								if((opcode_map[instruction_memory[ROB_table[a].rob_temp_pc].op_code] == SW)
								||(opcode_map[instruction_memory[ROB_table[a].rob_temp_pc].op_code] == SWS)){
									//std::cout << "There is a store" << '\n';
									if(int_reg_stat[ROB_table[a].add_reg].reg_busy != 1){
										std::cout << "Store detected and it does not have indeterminate address" << '\n';
										store_check = 1;
										break;
									}
									else{
										std::cout << "Busy stores detected in Load Step 1" << '\n';
										store_check = 0;
									}
									break;
								}
								else{
									std::cout << "No stores in Load Step 1" << '\n';
									store_check = 1;
								}
							}
							if(store_check == 0){
								for(int a = (ROB_table.size() - 1) ; a >= head_ROB; a--){
									if((opcode_map[instruction_memory[ROB_table[a].rob_temp_pc].op_code] == SW)
									||(opcode_map[instruction_memory[ROB_table[a].rob_temp_pc].op_code] == SWS)){
										std::cout << "Store detected first up" << '\n';
										if(int_reg_stat[ROB_table[a].add_reg].reg_busy != 1){
                      std::cout << "Store detected and it does not have indeterminate address" << '\n';
                      store_check = 1;
											break;
										}
										else{
                      std::cout << "Busy stores" << '\n';
											store_check = 0;
										}
									}
									else{
										std::cout << "Store not detected" << '\n';
										store_check = 1;
									}
								}
						}
					}

					if(RES_Stat_Load[i].Qj == 0 && store_check == 1){
						std::cout << "This condition was satisfied" << '\n';
						temp_res_add[i] = RES_Stat_Load[i].Vj + RES_Stat_Load[i].res_A;
						std::cout << "Address Load: " << hex << temp_res_add[i] << '\n';
						ld_step1[i] = 1;
						ld_current_exe[i] = 1;
					}
					break;
				}
			}
		}
	}

	for(unsigned i = 0; i < RES_Stat_Load.size(); i++){
		if(ld_step1[i] == 1){
			std::cout << "first step of load was satisfied" << '\n';
			std::cout << "Value of i: " << i << '\n';
			std::cout << "Value of entry: " << RES_Stat_Load[i].res_dest << '\n';
			std::cout << "Calculating PC after step 1 is over: " << dec << RES_Stat_Load[i].res_pc << '\n';
			if((ROB_table[RES_Stat_Load[i].res_dest].entry - 1) >= (head_ROB)){
				for(int a = (ROB_table[RES_Stat_Load[i].res_dest].entry - 1); a >= head_ROB ; a--){
					if((opcode_map[instruction_memory[ROB_table[a].rob_temp_pc].op_code] == SW)
					||(opcode_map[instruction_memory[ROB_table[a].rob_temp_pc].op_code] == SWS)){
						std::cout << "Store detected for load step 2" << '\n';
						std::cout << "The address at store ROB table: " << ROB_table[a].str_add << '\n';
						std::cout << "The address for load: " << temp_res_add[i] << '\n';
						if(temp_res_add[i] == ROB_table[a].str_add){
							needs_ex_ld[i] = 0;
							RES_Stat_Load[i].address_raw = a+1;
							std::cout << "There is an address raw" << '\n';
							break;
						}
						else{
							needs_ex_ld[i] = 1;
							RES_Stat_Load[i].address_raw = 0;
						}
					}
					else{
						std::cout << "Store not detected" << '\n';
						needs_ex_ld[i] = 1;
						RES_Stat_Load[i].address_raw = 0;
					}
				}
			}
			else{
				for(int a = (ROB_table[RES_Stat_Load[i].res_dest].entry - 1); a >= 0; a--){
					if((opcode_map[instruction_memory[ROB_table[a].rob_temp_pc].op_code] == SW)
					||(opcode_map[instruction_memory[ROB_table[a].rob_temp_pc].op_code] == SWS)){
						std::cout << "The address at store ROB table: " << ROB_table[a].str_add << '\n';
						std::cout << "The address for load: " << temp_res_add[i] << '\n';
						if(temp_res_add[i] == ROB_table[a].str_add){
							needs_ex_ld[i] = 0;
							RES_Stat_Load[i].address_raw = a+1;
							break;
						}
						else{
							RES_Stat_Load[i].address_raw = 0;
							needs_ex_ld[i] = 1;
						}
					}
					else{
						needs_ex_ld[i] = 1;
						RES_Stat_Load[i].address_raw = 0;
					}
				}
				if(needs_ex_ld[i] == 1){
					for(int a = (ROB_table[RES_Stat_Load[i].res_dest].entry - 1); a >= 0; a--){
						if((opcode_map[instruction_memory[ROB_table[a].rob_temp_pc].op_code] == SW)
						||(opcode_map[instruction_memory[ROB_table[a].rob_temp_pc].op_code] == SWS)){
							if(temp_res_add[i] == ROB_table[a].str_add){
								needs_ex_ld[i] = 0;
								RES_Stat_Load[i].address_raw = a+1;
								break;
							}
							else{
								RES_Stat_Load[i].address_raw = 0;
								needs_ex_ld[i] = 1;
							}
						}
					}
				}
			}
		}
	}

	for(unsigned i = 0; i < RES_Stat_Load.size(); i++){
		if(needs_ex_ld[i] == 1){
			if(RES_Stat_Load[i].if_depend_Vj == 1 || RES_Stat_Load[i].if_depend_Vk == 1){
				RES_Stat_Load[i].if_depend_Vj = 0;
				RES_Stat_Load[i].if_depend_Vk = 0;
				for(unsigned it = 0; it < ld_ex_done.size(); it++){
					if(ld_ex_done[it] == 1){
						ld_ex_done[it] = 0;
					}
				}
				continue;
			}
			switch (opcode_map[instruction_memory[RES_Stat_Load[i].op_num].op_code]) {
				case LW:
				case LWS:
					std::cout << "Ikde aala re baba" << '\n';
						for(unsigned sel_ld_unit = 0; sel_ld_unit < big_mem.size(); sel_ld_unit++){
							if(big_mem[sel_ld_unit].status == 1 && ROB_table[RES_Stat_Load[i].res_dest].state == ISSUE){
								if(ld_ex_done[sel_ld_unit] == 1){
									ld_ex_done[sel_ld_unit] = 0;
									continue;
								}
                std::cout << "Allocating execution unit for LOAD" << '\n';
								big_mem[sel_ld_unit].status = 0;
								ROB_table[RES_Stat_Load[i].res_dest].state = EXECUTE;
								RES_Stat_Load[i].use_ex = 1;
                RES_Stat_Load[i].execution_unit_used = sel_ld_unit;
								big_mem[sel_ld_unit].res_stat_num = i;
								RES_Stat_Load[i].res_A = temp_res_add[i];
                is_load_ex = 1;
								break;
							}
						}
					break;

				default:
					break;
			}
		}
		else if(needs_ex_ld[i] == 0){
			std::cout << "Mem execution unit not required and waiting for store to bypass" << '\n';
			std::cout << "Dependency at: " << i << '\n';

			if(RES_Stat_Load[i].address_raw == 0){
				std::cout << "Came to take the bypassed value" << '\n';
				if(ld_current_exe[i] == 1){
					ld_current_exe[i] = 0;
					continue;
				}
				RES_Stat_Load[i].Vk = RES_Stat_Load[i].from_forward;
				mem_result[i] = RES_Stat_Load[i].from_forward;
				ld_current_exe[i] = 0;
				ld_step1[i] = 0;
				ROB_table[RES_Stat_Load[i].res_dest].state = EXECUTE;
				flag_mem[i] = 1;
				RES_Stat_Load[i].res_A = temp_res_add[i];
				needs_ex_ld[i] = UNDEFINED;
			}
		}
	}

	for(unsigned sel_ld_unit = 0; sel_ld_unit < big_mem.size(); sel_ld_unit++){
    std::cout << "Store " << '\n';
		if(is_store_ex == 0){
			std::cout << "Is load coming here for execution?" << '\n';
			if(big_mem[sel_ld_unit].status == 0){
				std::cout << "Mem latency unit is busy" << '\n';
				std::cout << "Latency of mem before decrementing: " << num_latency_mem[sel_ld_unit] << '\n';
				if(num_latency_mem[sel_ld_unit] > 1){
					std::cout << "Latency of Mem Unit: " << num_latency_mem[sel_ld_unit] << '\n';
					num_latency_mem[sel_ld_unit]--;
					flag_mem[big_mem[sel_ld_unit].res_stat_num] = 0;
				}
				else{
					switch (opcode_map[instruction_memory[RES_Stat_Load[big_mem[sel_ld_unit].res_stat_num].op_num].op_code]) {
						case LW:
						case LWS:
							flag_mem[big_mem[sel_ld_unit].res_stat_num] = 1;
							mem_result[big_mem[sel_ld_unit].res_stat_num] = char2unsigned(data_memory + RES_Stat_Load[big_mem[sel_ld_unit].res_stat_num].res_A);
							break;
						}
						for(unsigned a = 0; a < create_exe_unit.size(); a++){
							if(create_exe_unit[a].exe_unit == MEMORY){
								hold_mem = a;
							}
						}
						ld_ex_done[sel_ld_unit] = 1;
						num_latency_mem[sel_ld_unit] = create_exe_unit[hold_mem].latency_exe;
						big_mem[sel_ld_unit].status = 1;
						ld_current_exe[big_mem[sel_ld_unit].res_stat_num] = 0;
						ld_step1[big_mem[sel_ld_unit].res_stat_num] = 0;
						needs_ex_ld[big_mem[sel_ld_unit].res_stat_num] = UNDEFINED;
            //is_load_ex = 0;
				}
			}
		}
	}


	/*
	for(unsigned i = 0; i < RES_Stat_Load.size(); i++){
		if(needs_ex_ld[i] == 1){
			if((RES_Stat_Load[i].Qj == 0) && (RES_Stat_Load[i].Qk == 0)){
				if(RES_Stat_Load[i].if_depend_Vj == 1 || RES_Stat_Load[i].if_depend_Vk == 1){
					RES_Stat_Load[i].if_depend_Vj = 0;
					RES_Stat_Load[i].if_depend_Vk = 0;
					for(unsigned it = 0; it < ld_ex_done.size(); it++){
						if(ld_ex_done[it] == 1){
							ld_ex_done[it] = 0;
						}
					}
					continue;
				}
				switch (RES_Stat_Load[i].op) {
					case MEMORY:

						if(is_store_ex == 0){

							for(unsigned sel_ld_unit = 0; sel_ld_unit < big_mem.size(); sel_ld_unit++){
								if(big_mem[sel_ld_unit].status == 1 && ROB_table[RES_Stat_Load[i].res_dest].state == ISSUE){
									if(ld_ex_done[sel_ld_unit] == 1){
										ld_ex_done[sel_ld_unit] = 0;
										continue;
									}

									big_mem[sel_ld_unit].status = 0;
									ROB_table[RES_Stat_Load[i].res_dest].state = EXECUTE;
									RES_Stat_Load[i].use_ex = 1;
									big_mem[sel_ld_unit].res_stat_num = i;
									needs_ex_ld[i] = UNDEFINED;
								}
							}
					}
				}
			}
		}
		else if(needs_ex_ld[i] == 0 && RES_Stat_Load[i].address_raw == 0){
			flag_mem[i] = 1;
			mem_result[i] = RES_Stat_Load[i].from_forward;
			ROB_table[RES_Stat_Load[i].res_dest].state = EXECUTE;
		}
	}
*/
/*
	for(unsigned i = 0; i < RES_Stat_Load.size(); i++){
		if(RES_Stat_Load[i].res_busy == 1){
			if((RES_Stat_Load[i].Qj == 0) && (RES_Stat_Load[i].Qk == 0)){
				if(RES_Stat_Load[i].if_depend_Vj == 1 || RES_Stat_Load[i].if_depend_Vk == 1){
					RES_Stat_Load[i].if_depend_Vj = 0;
					RES_Stat_Load[i].if_depend_Vk = 0;
					for(unsigned it = 0; it < ld_ex_done.size(); it++){
						if(ld_ex_done[it] == 1){
							ld_ex_done[it] = 0;
						}
					}
					continue;
				}
				switch(RES_Stat_Load[i].op){
					case MEMORY:
						std::cout << "Loading the functional unit in execution stage" << '\n';
						std::cout << "BIG MEM SIZE: " << big_mem.size() << '\n';
						for(unsigned sel_ld_unit = 0; sel_ld_unit < big_mem.size(); sel_ld_unit++){
							std::cout << "Status of Mem execution unit: " << big_mem[sel_ld_unit].status << '\n';
							std::cout << "State of Load: " << ROB_table[RES_Stat_Load[i].res_dest].state << '\n';
							std::cout << "Size of LOAD EX: " << ld_ex_done.size() << '\n';
							if(big_mem[sel_ld_unit].status == 1 && ROB_table[RES_Stat_Load[i].res_dest].state == ISSUE ){
								if(ld_ex_done[sel_ld_unit] == 1){
									std::cout << "Came here" << '\n';
									ld_ex_done[sel_ld_unit] = 0;
									continue;
								}

								std::cout << "Allocating functional unit" << '\n';
								big_mem[sel_ld_unit].status = 0;
								std::cout << "Value of ROB index: " << RES_Stat_Load[i].res_dest << '\n';
								ROB_table[RES_Stat_Load[i].res_dest].state = EXECUTE;
								RES_Stat_Load[i].use_ex = 1;
								big_mem[sel_ld_unit].res_stat_num = i;
								//num_latency_mem[sel_ld_unit]--;
								switch (opcode_map[instruction_memory[RES_Stat_Load[big_mem[sel_ld_unit].res_stat_num].op_num].op_code]) {
									case LW:
									case LWS:
										if((ROB_table[RES_Stat_Load[i].res_dest].entry - 1) > head_ROB){
											for(unsigned iter_ld = head_ROB; iter_ld < (ROB_table[RES_Stat_Load[i].res_dest].entry - 1) ; iter_ld++){
												if((opcode_map[instruction_memory[ROB_table[iter_ld].rob_temp_pc].op_code] == SW)
												|| (opcode_map[instruction_memory[ROB_table[iter_ld].rob_temp_pc].op_code] == SWS)) {
													store_check = 1;
													break;
												}
												else{
													store_check = 0;
												}
											}
										}
										if((RES_Stat_Load[big_mem[sel_ld_unit].res_stat_num].Qj == 0) && (store_check != 1)){
//										if(RES_Stat_Load[big_mem[sel_ld_unit].res_stat_num].Qj == 0){
											RES_Stat_Load[big_mem[sel_ld_unit].res_stat_num].res_A = RES_Stat_Load[big_mem[sel_ld_unit].res_stat_num].Vj + RES_Stat_Load[big_mem[sel_ld_unit].res_stat_num].res_A;

										}
										break;

									default:
										break;
									}
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
*/

	for(unsigned i = 0; i < RES_Stat_Load.size(); i++){
		std::cout << "Busy status of reservation station: " << RES_Stat_Load[i].res_busy <<'\n';
		if(RES_Stat_Load[i].res_busy){
			switch (opcode_map[instruction_memory[RES_Stat_Load[i].op_num].op_code]) {
				case SW:
						if((RES_Stat_Load[i].Qj == 0) && (RES_Stat_Load[i].Qk == 0)){
							if(RES_Stat_Load[i].if_depend_Vj == 1 || RES_Stat_Load[i].if_depend_Vk == 1){
								RES_Stat_Load[i].if_depend_Vj = 0;
								RES_Stat_Load[i].if_depend_Vk = 0;
								/*if(ld_current_exe[i] == 1){
									ld_current_exe[i] = 0;
								}*/
								continue;
							}
							/*if(ld_current_exe[i] == 1){
								ld_current_exe[i] = 0;
								continue;
							}*/
							ROB_table[RES_Stat_Load[i].res_dest].rob_dest = RES_Stat_Load[i].Vk + RES_Stat_Load[i].res_A;
							RES_Stat_Load[i].res_A = RES_Stat_Load[i].Vk + RES_Stat_Load[i].res_A;
							ROB_table[RES_Stat_Load[i].res_dest].dest_type = STORE_DEST;
							ROB_table[RES_Stat_Load[i].res_dest].state = EXECUTE;
							flag_mem[i] = 1;
						}
					break;

				case SWS:
					std::cout << "OPCODE check in SWS: " << instruction_memory[ROB_table[head_ROB].rob_temp_pc].op_code << '\n';
					std::cout << "Yes this is execution of SWS" << '\n';
					std::cout << "Qj: " << RES_Stat_Load[i].Qj << '\n';
					std::cout << "Qk: " << RES_Stat_Load[i].Qk << '\n';
					if((RES_Stat_Load[i].Qj == 0) && (RES_Stat_Load[i].Qk == 0)){
						std::cout << "Both conditions satisfied with Qj and Qk" << '\n';
						std::cout << "IF DEPEND Vj: " << RES_Stat_Load[i].if_depend_Vj << '\n';
						std::cout << "IF DEPEND Vk: " << RES_Stat_Load[i].if_depend_Vk << '\n';
						if(RES_Stat_Load[i].if_depend_Vj == 1 || RES_Stat_Load[i].if_depend_Vk == 1){
							std::cout << "Making dependencies in SWS zero and stalling" << '\n';
							RES_Stat_Load[i].if_depend_Vj = 0;
							RES_Stat_Load[i].if_depend_Vk = 0;
								/*if(ld_current_exe[i] == 1){
									ld_current_exe[i] = 0;
								}*/
							continue;
						}
							/*if(ld_current_exe[i] == 1){
								ld_current_exe[i] = 0;
								continue;
							}*/
            std::cout << "Value of Vk: " << RES_Stat_Load[i].Vk << '\n';
            std::cout << "Value of IMM: " << RES_Stat_Load[i].res_A << '\n';
						ROB_table[RES_Stat_Load[i].res_dest].rob_dest = RES_Stat_Load[i].Vk + RES_Stat_Load[i].res_A;
            std::cout << "Destination address: " << hex << ROB_table[RES_Stat_Load[i].res_dest].rob_dest << '\n';
						RES_Stat_Load[i].res_A = RES_Stat_Load[i].Vk + RES_Stat_Load[i].res_A;
						ROB_table[RES_Stat_Load[i].res_dest].dest_type = STORE_DEST;
						ROB_table[RES_Stat_Load[i].res_dest].state = EXECUTE;
						flag_mem[i] = 1;
					}
			break;
			}
		}
	}
/*	for(unsigned i = 0; i < RES_Stat_Load.size(); i++){
		if(RES_Stat_Load[i].res_busy){
			switch (opcode_map[instruction_memory[RES_Stat_Load[i].op_num].op_code]) {
				case SW:
					if((RES_Stat_Load[i].Qj == 0) && (RES_Stat_Load[i].Qk == 0) && (opcode_map[instruction_memory[ROB_table[head_ROB].rob_temp_pc].op_code] == SW)){
						ROB_table[head_ROB].rob_dest = RES_Stat_Load[i].Vk + RES_Stat_Load[i].res_A;
						RES_Stat_Load[i].res_A = RES_Stat_Load[i].Vk + RES_Stat_Load[i].res_A;
						ROB_table[head_ROB].dest_type = STORE_DEST;
						ROB_table[head_ROB].state = EXECUTE;
						flag_mem[i] = 1;
					}
					break;

				case SWS:
					if((RES_Stat_Load[i].Qj == 0) && ((RES_Stat_Load[i].Qk == 0)) && opcode_map[instruction_memory[ROB_table[head_ROB].rob_temp_pc].op_code] == SWS){
						ROB_table[head_ROB].rob_dest = RES_Stat_Load[i].Vk + RES_Stat_Load[i].res_A;
						RES_Stat_Load[i].res_A = RES_Stat_Load[i].Vk + RES_Stat_Load[i].res_A;
						ROB_table[head_ROB].dest_type = STORE_DEST;
						ROB_table[head_ROB].state = EXECUTE;
						flag_mem[i] = 1;
					}
					break;

				}
		}
	}
/*
	for(unsigned sel_ld_unit = 0; sel_ld_unit < big_mem.size(); sel_ld_unit++){
		std::cout << "Status of Mem when executing: " << big_mem[sel_ld_unit].status << '\n';
		if(big_mem[sel_ld_unit].status == 0){
			std::cout << "Latency of mem unit remaining: " << num_latency_mem[sel_ld_unit] << '\n';
			if(num_latency_mem[sel_ld_unit] > 1){
				std::cout << "The State in ROB: " << ROB_table[0].state << endl;
				std::cout << "Decrementing Latency" << '\n';
				num_latency_mem[sel_ld_unit]--;
				std::cout << "Index while latency: " << sel_ld_unit << '\n';
				std::cout << "Flag while latency: " << flag_mem[sel_ld_unit] << '\n';
				flag_mem[big_mem[sel_ld_unit].res_stat_num] = 0;
			}
			else{
				std::cout << "Load Unit: " << sel_ld_unit << '\n';
				std::cout << "Flag of mem unit right now: " << flag_mem[sel_ld_unit] << '\n';
				flag_mem[big_mem[sel_ld_unit].res_stat_num] = 1;
				std::cout << "Got into Load" << '\n';
				std::cout << "Served Station: " << big_mem[sel_ld_unit].res_stat_num << '\n';
				std::cout << "Op_Num: " << RES_Stat_Load[big_mem[sel_ld_unit].res_stat_num].op_num << '\n';
				switch (opcode_map[instruction_memory[RES_Stat_Load[big_mem[sel_ld_unit].res_stat_num].op_num].op_code]) {
					case LW:
					case LWS:
					std::cout << "In Load" << '\n';
						mem_result[big_mem[sel_ld_unit].res_stat_num] = char2unsigned(data_memory + RES_Stat_Load[big_mem[sel_ld_unit].res_stat_num].res_A);
						break;

					/*case SWS:
					case SW:
						if((opcode_map[instruction_memory[ROB_table[head_ROB].rob_pc].op_code] == SW)
						||(opcode_map[instruction_memory[ROB_table[head_ROB].rob_pc].op_code] == SWS)){
							sw_check_head = 1;
						}
						else{
							sw_check_head = 0;
						}
						if((RES_Stat_Load[big_mem[sel_ld_unit].res_stat_num].Qj == 0) && (sw_check_head == 1)){
						//	ROB_table[RES_Stat_Load[]] = RES_Stat_Load[big_mem[sel_ld_unit].res_stat_num].Vj + RES_Stat_Load[big_mem[sel_ld_unit].res_stat_num].res_A;
						}
						break;*/
/*				}
				for(unsigned a = 0; a < create_exe_unit.size(); a++){
					if(create_exe_unit[a].exe_unit == MEMORY){
						hold_mem = a;
					}
				}
				ld_ex_done[sel_ld_unit] = 1;
				num_latency_mem[sel_ld_unit] = create_exe_unit[hold_mem].latency_exe;
				big_mem[sel_ld_unit].status = 1;
			}
		}
	}
	*/
}

void sim_ooo::write_res(){
	std::cout << "****************WRITE RESULT******************" << '\n';
	unsigned b_rob;

	for(unsigned i = 0; i < RES_Stat_Int.size(); i++){
		if(flag_int[i] == 1){
			b_rob = RES_Stat_Int[i].res_dest + 1;
			RES_Stat_Int[i].res_busy = 0;

			for(unsigned j = 0; j < RES_Stat_Int.size(); j++){
				if(RES_Stat_Int[j].Qj == b_rob){
					RES_Stat_Int[j].Vj = int_result[i];
					RES_Stat_Int[j].Qj = 0;
				}
				if(RES_Stat_Int[j].Qk == b_rob){
					RES_Stat_Int[j].Vk = int_result[i];
					RES_Stat_Int[j].Qk = 0;
				}
			}

			for(unsigned j = 0; j < RES_Stat_Add.size(); j++){
				if(RES_Stat_Add[j].Qj == b_rob){
					RES_Stat_Add[j].Vj = int_result[i];
					RES_Stat_Add[j].Qj = 0;
				}
				if(RES_Stat_Add[j].Qk == b_rob){
					RES_Stat_Add[j].Vk = int_result[i];
					RES_Stat_Add[j].Qk = 0;
				}
			}

			for(unsigned j=0; j < RES_Stat_Mul.size(); j++){
				if(RES_Stat_Mul[j].Qj == b_rob){
					RES_Stat_Mul[j].Vj = int_result[i];
					RES_Stat_Mul[j].Qj = 0;
				}
				if(RES_Stat_Mul[j].Qk == b_rob){
					RES_Stat_Mul[j].Vk = int_result[i];
					RES_Stat_Mul[j].Qk = 0;
				}
			}

			for(unsigned j = 0; j < RES_Stat_Load.size(); j++){
				if(RES_Stat_Load[j].Qj == b_rob){
					RES_Stat_Load[j].Vj = int_result[i];
					RES_Stat_Load[j].Qj = 0;
				}
				if(RES_Stat_Load[j].Qk == b_rob){
					RES_Stat_Load[j].Vk = int_result[i];
					RES_Stat_Load[j].Qk = 0;
				}
			}


				ROB_table[RES_Stat_Int[i].res_dest].rob_val = int_result[i];
        int_ex_done[RES_Stat_Int[i].execution_unit_used] = 1;
				ROB_table[RES_Stat_Int[i].res_dest].if_branch_rob = RES_Stat_Int[i].if_branch;
				int_result[i] = UNDEFINED;
				ROB_table[b_rob - 1].ready = 1;
				ROB_table[b_rob - 1].state = WRITE_RESULT;
				RES_Stat_Int[i].res_pc = UNDEFINED;
				RES_Stat_Int[i].Vj = UNDEFINED;
				RES_Stat_Int[i].Vk = UNDEFINED;
				RES_Stat_Int[i].Qj = UNDEFINED;
				RES_Stat_Int[i].res_dest = UNDEFINED;
				RES_Stat_Int[i].res_A = UNDEFINED;
				RES_Stat_Int[i].op_num = UNDEFINED;
				RES_Stat_Int[i].if_branch = UNDEFINED;
				RES_Stat_Int[i].use_ex = 1;
				RES_Stat_Int[i].op = NOTVALID;
				RES_Stat_Int[i].wr_done = 1;
				flag_int[i] = 0;
			}
	}

	for(unsigned i = 0; i < RES_Stat_Add.size(); i++){
		if(flag_add[i] == 1){
			b_rob = RES_Stat_Add[i].res_dest + 1;
			RES_Stat_Add[i].res_busy = 0;

			for(unsigned j = 0; j < RES_Stat_Int.size(); j++){
				if(RES_Stat_Int[j].Qj == b_rob){
					RES_Stat_Int[j].Vj = add_result[i];
					RES_Stat_Int[j].Qj = 0;
				}
				if(RES_Stat_Int[j].Qk == b_rob){
					RES_Stat_Int[j].Vk = add_result[i];
					RES_Stat_Int[j].Qk = 0;
				}
			}

			for(unsigned j = 0; j < RES_Stat_Add.size(); j++){
				if(RES_Stat_Add[j].Qj == b_rob){
					RES_Stat_Add[j].Vj = add_result[i];
					RES_Stat_Add[j].Qj = 0;
				}
			  if(RES_Stat_Add[j].Qk == b_rob){
					RES_Stat_Add[j].Vk = add_result[i];
					RES_Stat_Add[j].Qk = 0;
				}
			}

			for(unsigned j=0; j < RES_Stat_Mul.size(); j++){
				if(RES_Stat_Mul[j].Qj == b_rob){
					RES_Stat_Mul[j].Vj = add_result[i];
					RES_Stat_Mul[j].Qj = 0;
				}
				if(RES_Stat_Mul[j].Qk == b_rob){
					RES_Stat_Mul[j].Vk = add_result[i];
					RES_Stat_Mul[j].Qk = 0;
				}
			}

			for(unsigned j = 0; j < RES_Stat_Load.size(); j++){
				if(RES_Stat_Load[j].Qj == b_rob){
					std::cout << "Gave the value to load store unit" << '\n';
					RES_Stat_Load[j].Vj = add_result[i];
					RES_Stat_Load[j].Qj = 0;
				}
				if(RES_Stat_Load[j].Qk == b_rob){
					std::cout << "Gave the value to load store unit for Qk" << '\n';
					RES_Stat_Load[j].Vk = mem_result[i];
					RES_Stat_Load[j].Qk = 0;
				}
			}
			  std::cout << "Value of ROB:" << RES_Stat_Add[i].res_dest << '\n';
				ROB_table[RES_Stat_Add[i].res_dest].rob_val = add_result[i];
				add_result[i] = UNDEFINED;
				add_ex_done[RES_Stat_Add[i].execution_unit_used] = 1;
				ROB_table[b_rob-1].ready = 1;
				ROB_table[b_rob-1].state = WRITE_RESULT;
				RES_Stat_Add[i].res_busy = 0;
				RES_Stat_Add[i].res_pc = UNDEFINED;
				RES_Stat_Add[i].Vj = UNDEFINED;
				RES_Stat_Add[i].Vk = UNDEFINED;
				RES_Stat_Add[i].Qj = UNDEFINED;
				RES_Stat_Add[i].res_dest = UNDEFINED;
				RES_Stat_Add[i].res_A = UNDEFINED;
				RES_Stat_Add[i].op_num = UNDEFINED;
				RES_Stat_Add[i].if_branch = UNDEFINED;
				RES_Stat_Add[i].use_ex = 1;
				RES_Stat_Add[i].op = NOTVALID;
				RES_Stat_Add[i].wr_done = 1;
				flag_add[i] = 0;
			}
	}

	for(unsigned i = 0; i < RES_Stat_Mul.size(); i++){
		std::cout << "Value of i entering Mul:" << i << '\n';
		std::cout << "Flag Mul value: " << flag_mem[i] << '\n';
		if(flag_mul[i] == 1){
			std::cout << "Value of i in Mul whose result is ready: " << i << '\n';
			b_rob = RES_Stat_Mul[i].res_dest + 1;
			RES_Stat_Mul[i].res_busy = 0;

			for(unsigned j = 0; j < RES_Stat_Int.size(); j++){
				if(RES_Stat_Int[j].Qj == b_rob){
					RES_Stat_Int[j].Vj = mul_result[i];
					RES_Stat_Int[j].Qj = 0;
				}
				if(RES_Stat_Int[j].Qk == b_rob){
					RES_Stat_Int[j].Vk = mul_result[i];
					RES_Stat_Int[j].Qk = 0;
				}
			}


			for(unsigned j = 0; j < RES_Stat_Add.size(); j++){
				if(RES_Stat_Add[j].Qj == b_rob){
					RES_Stat_Add[j].Vj = mul_result[i];
					RES_Stat_Add[j].Qj = 0;
				}
				if(RES_Stat_Add[j].Qk == b_rob){
					RES_Stat_Add[j].Vk = mul_result[i];
					RES_Stat_Add[j].Qk = 0;
				}
			}


			for(unsigned j=0; j < RES_Stat_Mul.size(); j++){
				if(RES_Stat_Mul[j].Qj == b_rob){
					RES_Stat_Mul[j].Vj = mul_result[i];
					RES_Stat_Mul[j].Qj = 0;
				}
				if(RES_Stat_Mul[j].Qk == b_rob){
					RES_Stat_Mul[j].Vk = mul_result[i];
					RES_Stat_Mul[j].Qk = 0;
				}
			}

			for(unsigned j = 0; j < RES_Stat_Load.size(); j++){
				if(RES_Stat_Load[j].Qj == b_rob){
					RES_Stat_Load[j].Vj = mul_result[i];
					RES_Stat_Load[j].Qj = 0;
				}
				if(RES_Stat_Load[j].Qk == b_rob){
					RES_Stat_Load[j].Vk = mem_result[i];
					RES_Stat_Load[j].Qk = 0;
				}
			}
        switch (opcode_map[instruction_memory[RES_Stat_Mul[i].op_num].op_code]) {
          case MULT:
          case MULTS:
            mul_ex_done[RES_Stat_Mul[i].execution_unit_used] = 1;
            break;

          case DIV:
          case DIVS:
            div_ex_done[RES_Stat_Mul[i].execution_unit_used] = 1;
            break;
        }
				ROB_table[RES_Stat_Mul[i].res_dest].rob_val = mul_result[i];
				mul_result[i] = UNDEFINED;
				ROB_table[b_rob-1].ready = 1;
				ROB_table[b_rob-1].state = WRITE_RESULT;
				RES_Stat_Mul[i].res_busy = 0;
				RES_Stat_Mul[i].res_pc = UNDEFINED;
				RES_Stat_Mul[i].Vj = UNDEFINED;
				RES_Stat_Mul[i].Vk = UNDEFINED;
				RES_Stat_Mul[i].Qj = UNDEFINED;
				RES_Stat_Mul[i].res_dest = UNDEFINED;
				RES_Stat_Mul[i].res_A = UNDEFINED;
				RES_Stat_Mul[i].op_num = UNDEFINED;
				RES_Stat_Mul[i].if_branch = UNDEFINED;
				RES_Stat_Mul[i].use_ex = 1;
				RES_Stat_Mul[i].op = NOTVALID;
				RES_Stat_Mul[i].wr_done = 1;
				flag_mul[i] = 0;
			}
	}

	unsigned if_addr_same;
	for(unsigned i = 0; i < RES_Stat_Load.size(); i++){
		std::cout << "Index of flag mem: " << i << '\n';
		std::cout << "flag_mem: " << flag_mem[i] << '\n';
		if(flag_mem[i] == 1){
			std::cout << "In Load of write result" << '\n';
			std::cout << "OP NUM: " << RES_Stat_Load[i].op_num << '\n';
			switch (opcode_map[instruction_memory[RES_Stat_Load[i].op_num].op_code]){
				case LW:
				case LWS:
					b_rob = RES_Stat_Load[i].res_dest + 1;
					std::cout << "Value of B: " << b_rob << '\n';
					RES_Stat_Load[i].res_busy = 0;

					/*for(unsigned iter_mem = 0; iter_mem < RES_Stat_Load.size(); iter_mem++){
						if(mem_result[iter_mem] == RES_Stat_Load[i].res_A){
							if_addr_same = 1;
							break;
						}
						else{
							if_addr_same = 0;
						}
					}*/
					for(unsigned j = 0; j < RES_Stat_Int.size(); j++){
						if(RES_Stat_Int[j].Qj == b_rob){
							RES_Stat_Int[j].Vj = mem_result[i];
							RES_Stat_Int[j].Qj = 0;
						}
						if(RES_Stat_Int[j].Qk == b_rob){
							RES_Stat_Int[j].Vk = mem_result[i];
							RES_Stat_Int[j].Qk = 0;
						}
					}

					for(unsigned j = 0; j < RES_Stat_Add.size(); j++){
						std::cout << "In here I'm present" << '\n';
						if(RES_Stat_Add[j].Qj == b_rob){
							RES_Stat_Add[j].Vj = mem_result[i];
							RES_Stat_Add[j].Qj = 0;
						}
						if(RES_Stat_Add[j].Qk == b_rob){
							std::cout << "Loading in Vk" << '\n';
							RES_Stat_Add[j].Vk = mem_result[i];
							RES_Stat_Add[j].Qk = 0;
						}
					}

					for(unsigned j=0; j < RES_Stat_Mul.size(); j++){
						if(RES_Stat_Mul[j].Qj == b_rob){
							RES_Stat_Mul[j].Vj = mem_result[i];
							RES_Stat_Mul[j].Qj = 0;
						}
						if(RES_Stat_Mul[j].Qk == b_rob){
							RES_Stat_Mul[j].Vk = mem_result[i];
							RES_Stat_Mul[j].Qk = 0;
						}
					}

					for(unsigned j = 0; j < RES_Stat_Load.size(); j++){
						if(RES_Stat_Load[j].Qj == b_rob){
							std::cout << "Present here!!!" << '\n';
							RES_Stat_Load[j].Vj = mem_result[i];
							RES_Stat_Load[j].Qj = 0;
						}
						if(RES_Stat_Load[j].Qk == b_rob){
							RES_Stat_Load[j].Vk = mem_result[i];
							RES_Stat_Load[j].Qk = 0;
						}
					}
						std::cout << "Value of i: " << i << '\n';
						std::cout << "Also came here" << '\n';
            ld_ex_done[RES_Stat_Load[i].execution_unit_used] = 1;
            if(is_load_ex == 1){
              is_load_ex = 0;
            }
            ROB_table[RES_Stat_Load[i].res_dest].rob_val = mem_result[i];
						mem_result[i] = UNDEFINED;
						ROB_table[b_rob-1].ready = 1;
						ROB_table[b_rob-1].state = WRITE_RESULT;
						RES_Stat_Load[i].res_pc = UNDEFINED;
						RES_Stat_Load[i].Vj = UNDEFINED;
						RES_Stat_Load[i].Vk = UNDEFINED;
						RES_Stat_Load[i].Qj = UNDEFINED;
						RES_Stat_Load[i].res_dest = UNDEFINED;
						RES_Stat_Load[i].res_A = UNDEFINED;
						RES_Stat_Load[i].op_num = UNDEFINED;
						RES_Stat_Load[i].if_branch = UNDEFINED;
						RES_Stat_Load[i].use_ex = 1;
						RES_Stat_Load[i].op = NOTVALID;
						RES_Stat_Load[i].wr_done = 1;
						flag_mem[i] = 0;
						std::cout << "Flag_MEM: " << flag_mem[i] << '\n';

					break;

				case SW:
				case SWS:
					b_rob = RES_Stat_Load[i].res_dest + 1;
					std::cout << "In write result of Store" << '\n';
					if(RES_Stat_Load[i].Qk == 0){
						std::cout << "No dependencies" << '\n';
						for(unsigned j = 0; j < RES_Stat_Load.size(); j++){
							if(RES_Stat_Load[j].address_raw == b_rob){
								std::cout << "ADDRESS RAW DEPENDENCY FOUND AT: " << j << '\n';
								RES_Stat_Load[j].address_raw = 0;
								RES_Stat_Load[j].from_forward = RES_Stat_Load[i].Vj;
								ld_step1[j] = 0;
								ld_current_exe[i] = 1;
							}
						}

            for(unsigned ld_ex = 0; ld_ex < ld_ex_done.size(); ld_ex++){
              if(ld_ex_done[ld_ex] == 1){
                ld_ex_done[ld_ex] = 0;
              }
            }


						ROB_table[b_rob - 1].rob_val = RES_Stat_Load[i].Vj;
						ROB_table[b_rob - 1].ready = 1;
						ROB_table[RES_Stat_Load[i].res_dest].state = WRITE_RESULT;
						RES_Stat_Load[i].res_busy = 0;
						RES_Stat_Load[i].res_pc = UNDEFINED;
						RES_Stat_Load[i].Vj = UNDEFINED;
						RES_Stat_Load[i].Vk = UNDEFINED;
						RES_Stat_Load[i].Qj = UNDEFINED;
						RES_Stat_Load[i].res_dest = UNDEFINED;
						RES_Stat_Load[i].res_A = UNDEFINED;
						RES_Stat_Load[i].op_num = UNDEFINED;
						RES_Stat_Load[i].if_branch = UNDEFINED;
						RES_Stat_Load[i].use_ex = 1;
						RES_Stat_Load[i].op = NOTVALID;
						RES_Stat_Load[i].wr_done = 1;
						flag_mem[i] = 0;
					}
					break;
			}
		}
	}
}

void sim_ooo::commit_stage(){
	std::cout << "**********COMMIT************" << '\n';
	unsigned d;
	unsigned if_branch = 0;
	unsigned flag_end = 0;
	res_type_t type_res;
	is_cleared = 0;
	unsigned if_store_commit = 0;
	unsigned hold_mem;
  if(ROB_table[head_ROB].ready == 1){
		std::cout << "PC of head: " << hex << ROB_table[head_ROB].rob_pc << '\n';
		d = ROB_table[head_ROB].rob_dest;
		type_res = ROB_table[head_ROB].dest_type;
		std::cout << "OPCODE at head during COMMIT: " << instruction_memory[ROB_table[head_ROB].rob_temp_pc].op_code << '\n';
		switch(opcode_map[instruction_memory[ROB_table[head_ROB].rob_temp_pc].op_code]){
			case BEQZ:
			case BNEZ:
			case BLTZ:
			case BGTZ:
			case BLEZ:
			case BGEZ:
				num_instructions++;
				if(ROB_table[head_ROB].if_branch_rob == 1){
					current_PC = ROB_table[head_ROB].rob_val;
					clear_rob();
					clear_res();
					clear_reg();
					clear_exe();

					head_ROB = 0;
					if_branch = 1;
				}
				break;

			case SW:
			case SWS:
        if_store_commit = 1;
				std::cout << "Am I a store: " << '\n';
				for(unsigned sel_ld_unit = 0; sel_ld_unit < big_mem.size(); sel_ld_unit++){
						std::cout << "Status of mem unit for store: " << big_mem[sel_ld_unit].status << '\n';
						std::cout << "State at ROB of store: " << ROB_table[head_ROB].state << '\n';
						if(big_mem[sel_ld_unit].status == 1 && ROB_table[head_ROB].state == WRITE_RESULT){
							if(is_load_ex == 1){
                std::cout << "Stalled here" << '\n';
								//ld_ex_done[sel_ld_unit] = 0;
								continue;
							}
							big_mem[sel_ld_unit].status = 0;
							ROB_table[head_ROB].state = COMMIT;
//							big_mem[sel_ld_unit].res_stat_num = ;
							is_store_ex = 1;
							num_instructions++;
					}
				}

				for(unsigned sel_ld_unit = 0; sel_ld_unit < big_mem.size(); sel_ld_unit++){
					if(big_mem[sel_ld_unit].status == 0){
            if(is_load_ex == 0){
              std::cout << "Latency of store before decrementing: " << num_latency_mem[sel_ld_unit] << '\n';
  						if(num_latency_mem[sel_ld_unit] > 1){
  							num_latency_mem[sel_ld_unit]--;
  //							flag_mem[big_mem[sel_ld_unit].res_stat_num] = 0;
  						}
  						else{
                //flag_mem[big_mem[sel_ld_unit].res_stat_num] = 1;
  							unsigned2char(ROB_table[head_ROB].rob_val,data_memory+ROB_table[head_ROB].rob_dest);
  							big_mem[sel_ld_unit].status = 1;
  							ld_ex_done[sel_ld_unit] = 1;
  							for(unsigned a = 0; a < create_exe_unit.size(); a++){
  								if(create_exe_unit[a].exe_unit == MEMORY){
  									hold_mem = a;
  								}
  							}
  							if_store_commit = 0;
  							num_latency_mem[sel_ld_unit] = create_exe_unit[hold_mem].latency_exe;
  							is_store_ex = 0;
  						}
            }
					}
				}
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
			case LW:
				num_instructions++;
				std::cout << "The destination register: " << d << '\n';
				std::cout << "The headROB: " << head_ROB << '\n';
				register_file[d] = ROB_table[head_ROB].rob_val;
				break;

			case ADDS:
			case SUBS:
			case MULTS:
			case DIVS:
			case LWS:
				num_instructions++;
				float_reg_file[d] = unsigned2float(ROB_table[head_ROB].rob_val);
				break;

			default:
				break;
		}
		if(if_store_commit == 0){
			ROB_table[head_ROB].rob_busy = false;
			ROB_table[head_ROB].ready = false;
			ROB_table[head_ROB].rob_pc = UNDEFINED;
			ROB_table[head_ROB].state = INVALID;
			ROB_table[head_ROB].rob_dest = UNDEFINED;
			ROB_table[head_ROB].dest_type = NOTASSIGNED;
			ROB_table[head_ROB].if_commit = 1;
			if(type_res == INTEGER_RES){
				if(int_reg_stat[d].ROB_num == head_ROB){
					int_reg_stat[d].reg_busy = 0;
				}
			}
			if(type_res == FLOAT_RES){
				if(float_reg_stat[d].ROB_num == head_ROB){
					float_reg_stat[d].reg_busy = 0;
				}
			}
			std::cout << "Check_END: " << check_end << '\n';

			head_ROB = head_ROB + 1;
			std::cout << "Incrementing ROB head:" << head_ROB << '\n';

			std::cout << "Size of ROB: " << ROB_table.size() << '\n';
			if(head_ROB == ROB_table.size()){
				std::cout << "True here" << '\n';
				head_ROB = 0;
			}
		}
		if(if_branch == 1){
			std::cout << "True here as well" << '\n';
			check_end = 0;
			is_cleared = 1;
			head_ROB = 0;
		}
	}
	if(check_end == 1){
		for(unsigned iter = 0; iter < ROB_table.size(); iter++){
				std::cout << "Iter: " << iter << '\n';
				std::cout << "Busy ROB:" << ROB_table[iter].rob_busy << '\n';
				if(ROB_table[iter].rob_busy != 0){
					std::cout << "Always true" << '\n';
					flag_end = 0;
					break;
				}
				else{
					flag_end = 1;
				}
		}
	}
	if(flag_end == 1){
			eop_end = 1;
	}
	else{
		eop_end = 0;
	}
	std::cout << "Head of ROB: " << head_ROB << '\n';
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
		flag_int[lint] = 0;
  }

	RES_Temp.res_name = "Mult";
  for(unsigned lmul = 0; lmul < RES_Stat_Mul.size(); lmul++){
		RES_Stat_Mul[lmul] = RES_Temp;
		mul_result[lmul] = UNDEFINED;
		flag_mul[lmul] = 0;
  }

	RES_Temp.res_name = "Add";
  for(unsigned ladd = 0; ladd < RES_Stat_Add.size(); ladd++){
		RES_Stat_Add[ladd] = RES_Temp;
		add_result[ladd] = UNDEFINED;
		flag_add[ladd] = 0;
	}

	RES_Temp.res_name = "Load";
  for(unsigned lload = 0; lload < RES_Stat_Load.size(); lload++){
		RES_Stat_Load[lload] = RES_Temp;
		mem_result[lload] = UNDEFINED;
		flag_mem[lload] = 0;
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

void sim_ooo::clear_exe(){
	unsigned hold_int;
	unsigned hold_mem;
	unsigned hold_mul;
	unsigned hold_div;
	unsigned hold_add;

	for(unsigned a = 0; a < create_exe_unit.size(); a++){
		if(create_exe_unit[a].exe_unit == INTEGER){
			hold_int = a;
		}
	}
	for(unsigned a = 0; a < create_exe_unit.size(); a++){
		if(create_exe_unit[a].exe_unit == MULTIPLIER){
			hold_mul = a;
		}
	}
	for(unsigned a = 0; a < create_exe_unit.size(); a++){
		if(create_exe_unit[a].exe_unit == DIVIDER){
			hold_div = a;
		}
	}
	for(unsigned a = 0; a < create_exe_unit.size(); a++){
		if(create_exe_unit[a].exe_unit == ADDER){
			hold_add = a;
		}
	}
	for(unsigned a = 0; a < create_exe_unit.size(); a++){
		if(create_exe_unit[a].exe_unit == MEMORY){
			hold_mem = a;
		}
	}
	for (unsigned i = 0; i < big_add.size(); i++){
		num_latency_add[i] = create_exe_unit[hold_add].latency_exe;
		big_add[i].status = 1;
		big_add[i].res_stat_num = UNDEFINED;
	}

	for (unsigned i = 0; i < big_int.size(); i++){
		num_latency_int[i] = create_exe_unit[hold_int].latency_exe;
		big_int[i].status = 1;
		big_int[i].res_stat_num = UNDEFINED;
	}

	for (unsigned i = 0; i < big_div.size(); i++){
		num_latency_div[i] = create_exe_unit[hold_div].latency_exe;
		big_div[i].status = 1;
		big_div[i].res_stat_num = UNDEFINED;
	}

	for (unsigned i = 0; i < big_mul.size(); i++){
		num_latency_mul[i] = create_exe_unit[hold_mul].latency_exe;
		big_mul[i].status = 1;
		big_mul[i].res_stat_num = UNDEFINED;
	}

	for (unsigned i = 0; i < big_mul.size(); i++){
		num_latency_mem[i] = create_exe_unit[hold_mem].latency_exe;
		big_mem[i].status = 1;
		big_mem[i].res_stat_num = UNDEFINED;
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
		if(ROB_table[i].rob_busy){std::cout << setw(6) << "yes";}
		else{std::cout << setw(6) << "no";}
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
				std::cout << setfill(' ') << setw(10) << "COMMIT";
				break;
			case INVALID:
				std::cout << setfill(' ') << setw(10) << "-";
				break;
		}
		if(ROB_table[i].state == INVALID || ROB_table[i].rob_dest == UNDEFINED)std::cout << setw(6) << "-";
		else if(ROB_table[i].dest_type == FLOAT_RES)std::cout << setfill(' ') << setw(5) << "F" << ROB_table[i].rob_dest;
		else if(ROB_table[i].dest_type == INTEGER_RES)std::cout << setfill(' ') << setw(5) << "R" << ROB_table[i].rob_dest;
		else if(ROB_table[i].dest_type == STORE_DEST)std::cout << setfill(' ') << setw(5) << dec << ROB_table[i].rob_dest;
		if(ROB_table[i].state == WRITE_RESULT)std::cout << setw(12) << hex << ROB_table[i].rob_val << '\n';
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
		if(RES_Stat_Int[i].res_busy == 1)std::cout << setw(6) << "yes";
		else std::cout << setw(6) << "no";
		if(RES_Stat_Int[i].res_busy == 1)std::cout << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Int[i].res_pc;
		else std::cout << setw(12) << "-";
		if((RES_Stat_Int[i].res_busy == 1) && (RES_Stat_Int[i].Vj!=UNDEFINED))std::cout << setfill(' ') << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Int[i].Vj;
		else std::cout << setfill(' ') << setw(12) << "-";
		if((RES_Stat_Int[i].res_busy == 1) && (RES_Stat_Int[i].Vk!=UNDEFINED))std::cout << setfill(' ') << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Int[i].Vk;
		else std::cout << setfill(' ') << setw(12) << "-";
		if((RES_Stat_Int[i].res_busy == 1) && (RES_Stat_Int[i].Qj!=0))std::cout << setfill(' ') << setw(6) << RES_Stat_Int[i].Qj-1;
		else std::cout << setfill(' ') << setw(6) << "-";
		if((RES_Stat_Int[i].res_busy == 1) && (RES_Stat_Int[i].Qk!=0))std::cout << setfill(' ') << setw(6) << RES_Stat_Int[i].Qk-1;
		else std::cout << setfill(' ') << setw(6) << "-";
		if(RES_Stat_Int[i].res_busy == 1)std::cout << setw(6) << RES_Stat_Int[i].res_dest;
		else std::cout << setfill(' ') << setw(6) << "-";
		if((RES_Stat_Int[i].res_busy == 1) && (RES_Stat_Int[i].res_A != UNDEFINED))std::cout << setfill(' ') << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Int[i].res_A << '\n';
		else std::cout << setfill(' ') << setw(12) << "-" << '\n';
	}

	for(unsigned i = 0; i < RES_Stat_Load.size(); i++){
		std::cout << setfill(' ') << setw(6) << RES_Stat_Load[i].res_name << (i+1);
		if(RES_Stat_Load[i].res_busy == 1)std::cout << setw(6) << "yes";
		else std::cout << setw(6) << "no";
		if(RES_Stat_Load[i].res_busy == 1)std::cout << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Load[i].res_pc;
		else std::cout << setw(12) << "-";
		if((RES_Stat_Load[i].res_busy == 1) && (RES_Stat_Load[i].Vj!=UNDEFINED))std::cout << setfill(' ') << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Load[i].Vj;
		else std::cout << setfill(' ') << setw(12) << "-";
		if((RES_Stat_Load[i].res_busy == 1) && (RES_Stat_Load[i].Vk!=UNDEFINED))std::cout << setfill(' ') << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Load[i].Vk;
		else std::cout << setfill(' ') << setw(12) << "-";
		if((RES_Stat_Load[i].res_busy == 1) && (RES_Stat_Load[i].Qj!=0))std::cout << setfill(' ') << setw(6) << RES_Stat_Load[i].Qj-1;
		else std::cout << setfill(' ') << setw(6) << "-";
		if((RES_Stat_Load[i].res_busy == 1) && (RES_Stat_Load[i].Qk!=0))std::cout << setfill(' ') << setw(6) << RES_Stat_Load[i].Qk-1;
		else std::cout << setfill(' ') << setw(6) << "-";
		if(RES_Stat_Load[i].res_busy == 1)std::cout << setw(6) << RES_Stat_Load[i].res_dest;
		else std::cout << setfill(' ') << setw(6) << "-";
		if((RES_Stat_Load[i].res_busy == 1) && (RES_Stat_Load[i].res_A != UNDEFINED))std::cout << setfill(' ') << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Load[i].res_A << '\n';
		else std::cout << setfill(' ') << setw(12) << "-" << '\n';
	}

	for(unsigned i = 0; i < RES_Stat_Add.size(); i++){
		std::cout << setfill(' ') << setw(6) << RES_Stat_Add[i].res_name << (i+1);
		if(RES_Stat_Add[i].res_busy == 1)std::cout << setw(6) << "yes";
		else std::cout << setw(6) << "no";
		if(RES_Stat_Add[i].res_busy == 1)std::cout << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Add[i].res_pc;
		else std::cout << setw(12) << "-";
		if((RES_Stat_Add[i].res_busy == 1) && (RES_Stat_Add[i].Vj!=UNDEFINED))std::cout << setfill(' ') << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Add[i].Vj;
		else std::cout << setfill(' ') << setw(12) << "-";
		if((RES_Stat_Add[i].res_busy == 1) && (RES_Stat_Add[i].Vk!=UNDEFINED))std::cout << setfill(' ') << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Add[i].Vk;
		else std::cout << setfill(' ') << setw(12) << "-";
		if((RES_Stat_Add[i].res_busy == 1) && (RES_Stat_Add[i].Qj!=0))std::cout << setfill(' ') << setw(6) << RES_Stat_Add[i].Qj-1;
		else std::cout << setfill(' ') << setw(6) << "-";
		if((RES_Stat_Add[i].res_busy == 1) && (RES_Stat_Add[i].Qk!=0))std::cout << setfill(' ') << setw(6) << RES_Stat_Add[i].Qk-1;
		else std::cout << setfill(' ') << setw(6) << "-";
		if(RES_Stat_Add[i].res_busy == 1)std::cout << setw(6) << RES_Stat_Add[i].res_dest;
		else std::cout << setfill(' ') << setw(6) << "-";
		if((RES_Stat_Add[i].res_busy == 1) && (RES_Stat_Add[i].res_A != UNDEFINED))std::cout << setfill(' ') << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Add[i].res_A << '\n';
		else std::cout << setfill(' ') << setw(12) << "-" << '\n';
	}

	for(unsigned i = 0; i < RES_Stat_Mul.size(); i++){
		std::cout << setfill(' ') << setw(6) << RES_Stat_Mul[i].res_name << (i+1);
		if(RES_Stat_Mul[i].res_busy == 1)std::cout << setw(6) << "yes";
		else std::cout << setw(6) << "no";
		if(RES_Stat_Mul[i].res_busy == 1)std::cout << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Mul[i].res_pc;
		else std::cout << setw(12) << "-";
		if((RES_Stat_Mul[i].res_busy == 1) && (RES_Stat_Mul[i].Vj!=UNDEFINED))std::cout << setfill(' ') << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Mul[i].Vj;
		else std::cout << setfill(' ') << setw(12) << "-";
		if((RES_Stat_Mul[i].res_busy == 1) && (RES_Stat_Mul[i].Vk!=UNDEFINED))std::cout << setfill(' ') << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Mul[i].Vk;
		else std::cout << setfill(' ') << setw(12) << "-";
		if((RES_Stat_Mul[i].res_busy == 1) && (RES_Stat_Mul[i].Qj!=0))std::cout << setfill(' ') << setw(6) << RES_Stat_Mul[i].Qj-1;
		else std::cout << setfill(' ') << setw(6) << "-";
		if((RES_Stat_Mul[i].res_busy == 1) && (RES_Stat_Mul[i].Qk!=0))std::cout << setfill(' ') << setw(6) << RES_Stat_Mul[i].Qk-1;
		else std::cout << setfill(' ') << setw(6) << "-";
		if(RES_Stat_Mul[i].res_busy == 1)std::cout << setw(6) << RES_Stat_Mul[i].res_dest;
		else std::cout << setfill(' ') << setw(6) << "-";
		if((RES_Stat_Mul[i].res_busy == 1) && (RES_Stat_Mul[i].res_A != UNDEFINED))std::cout << setfill(' ') << setw(4) << hex << "0x" << setw(8) << setfill('0') << RES_Stat_Mul[i].res_A << '\n';
		else std::cout << setfill(' ') << setw(12) << "-" << '\n';
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
	return float(float(num_instructions)/float(num_cycles)); //fill here
}

unsigned sim_ooo::get_instructions_executed(){
	return  num_instructions; //fill here
}

unsigned sim_ooo::get_clock_cycles(){
	return num_cycles; //fill here
}
