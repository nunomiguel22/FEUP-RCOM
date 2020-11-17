#Author: Nuno Miguel Fernandes Marques @ https://github.com/nunomiguel22
#General purpose C makefile, tested on linux and windows

# compiler
CXX = gcc
CXXFLAGS = -Wall -Wextra -Wfloat-equal -pedantic -Wundef -Wswitch-default -Wunreachable-code
LDFLAGS = -g -I./source
LBLIBS = 
BUILD_PATH = build

# Source code folders - Add $(wildcard path/to/object.o) if using a precompiled object such as adding an icon to the executable 
SRC := $(wildcard source/**/**/**/**/*.c) $(wildcard source/**/**/**/*.c) $(wildcard source/**/**/*.c) $(wildcard source/**/*.c) $(wildcard source/*.c)

#Objects
OBJ := $(patsubst %.c,$(BUILD_PATH)/%.o,$(SRC))

#Executable Name
EXEC = RCOM

all: $(EXEC)

#Building
$(EXEC): $(OBJ)
	$(CXX) $(LDFLAGS) -o $@ $(OBJ) $(LBLIBS) $(CXXFLAGS) 

ifdef OS #Windows
$(BUILD_PATH)/%.o: %.c
	$(shell if not exist $(dir $(subst /,\,$@)) mkdir $(dir $(subst /,\,$@)))
	$(CXX) $(LDFLAGS) -c  $< -o $@ $(CXXFLAGS)
	
else #linux
$(BUILD_PATH)/%.o: %.c
	$(shell mkdir -p $(dir $@))
	$(CXX) $(LDFLAGS) -c $< -o $@ $(CXXFLAGS)
endif

#Cleaning
clean:
ifdef OS #Windows
	if exist $(BUILD_PATH) rmdir /s /q $(BUILD_PATH)
	if exist $(addsuffix .exe, $(EXEC)) del $(addsuffix .exe, $(EXEC))
else #linux
	rm -rf $(OBJ) $(EXEC)
endif

#useful LDFLAGS:
#-m32 													#Compile x86 binary using x64 gcc
#-g 													#Generate executable with debug information
#-s 													# Strips executable of debug information
#-Wl,-subsystem,windows 								#Sets windows subsystem, (window mode)
#-municode 												#Compile unicode often needed by windows api(wWinMain undefined error, etc)

#useful LBLIBS
#-lpthread												#Cross-platform multithreading library
#-ldl													#Dynamic library loading for linux, mandatory if using dynamic libraries on linux
#-lSDL2 -lSDL2main -lSDL2_ttf -lSDL2_net -lSDL2_mixer	#Cross-platform low level hardware(graphics, fonts, audio, etc) access library - #define SDL_MAIN_HANDLED if main error
#-lglfw3												#Cross-platform library for openGL development
#-lopengl32 											#OpenGL cross-platform low level graphics libarary
#-lole32 -lcomctl32 -loleaut32 -luuid 					#Windows common controls(dialog boxes, etc)
#-lws2_32 												#winsock (windows http/udp api)
#-lfreetype												#Cross-platform font loading library 
#-lassimp 												#Cross-platform model/mesh loading library
#-lzlib 												#-lz for Linux, data-compression library
#-lX11													#Linux graphic interface library, may be needed for graphic applications on linux


#Useful CXXFLAGS:
#-fsanitize=address 									#Analyze memory leaks (LINUX ONLY)
#-Wall -Wextra  										#Show compiler warnings
#-Werror 												#Treat warnings as errors
#-Wfloat-equal 											#Warning when comparing floats with 0
#-Wundef 												#Warning if an undefined identifier is evaluated
#-Wswitch-default 										#Warning if a switch stamentent does not have a default case
#-Wunreachable-code 									#Warn if the compiler detects code that cannot be reached
#-Wconversion 											#Warn if conversion may alter value

#Optimization flags
#-O0 													#Default: No optimization, fastest compile time, recommended for debugging
#-O1 													#Moderate optimization
#-O2 													#Performs nearly all supported optimizations
#-O3 													#Full optimization
#-Os 													#Optimize for size




#MAKE code snippets
#OS Conditional example:
#ifdef OS #Windows
#	do stuff for windows 
#else #linux
#	do stuff for linux
#endif
