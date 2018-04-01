CC = g++
OPT = -g
WARN = -Wall
INCLUDE = -I..
CFLAGS = $(OPT) $(WARN) $(INCLUDE)

#################################

# default rule
all: .cc.o

# generic rule for converting any .cc file to any .o file
.cc.o:
	$(CC) $(CFLAGS) -c *.cc
