#===========================================================================
#
# EECS461 at the University of Michigan
# 5553/Metroworks Makefile for Lab 2
#
# Created 6-15-06, Jeff Lovell, Eric Williams
#
# Last Revised:
# 	7-27-06	lovelljc
# 	8-28-06	ericjw
#
# Revised and adapted to use at IMRT ETH Zurich 
# 151-0593-00 Embedded Conrol Systems (ECS) 
#
# Revision History:
#	2008-08-26		Marianne Schmid   -spe_vector -abi eabi added to COMPILE_TOOL_OPTS
#							-abi eabi added to LINK_TOOL_OPTS
#===========================================================================
#
# Commands to invoke this file:
# 	gmake
# 	gmake angle
# 	gmake clean
#
#---------------------------------TOOLCHAIN------------------------------------
#---------- Define metrowerks tools to build using command line ---------------
#------------------------------------------------------------------------------

COMPILE_TOOL	 	= $(MW_TOOL)/PowerPC_EABI_Tools/Command_Line_Tools/mwcceppc
# see "C:/Freescale/MPC55xxV2-2/Help/PDFPower Architecture Build Tools Reference.pdf" chapter "Command-Line Options for Diagnostic Messages"
# -w all
COMPILE_TOOL_OPTS	= -gdwarf-2 -fp efpu -char signed -proc Zen\
					-gccincludes -nostdinc -nosyspath -ansi off\
					-Cpp_exceptions off -O0  -spe_vector -abi eabi
# added -O0 flag above to turn off code optimizations; removed "-opt all" - JCL

LINK_TOOL		= $(MW_TOOL)/PowerPC_EABI_Tools/Command_Line_Tools/mwldeppc
LINK_TOOL_OPTS		= -gdwarf-2 -fp efpu -char signed -proc Zen -nostdlib \
					-srec -lcf ../freescale/MPC5554_DEBUG.lcf -abi eabi

TOOL_INCLUDES		= -I$(MW_TOOL)/PowerPC_EABI_Support/Runtime/Include \
			  -I$(MW_TOOL)/PowerPC_EABI_Support/Runtime/Src \
			  -I$(MW_TOOL)/PowerPC_EABI_Support/Runtime/Lib \
			  -I$(MW_TOOL)/PowerPC_EABI_Support/MSL/MSL_C/PPC_EABI/Lib \
			  -I$(MW_TOOL)/PowerPC_EABI_Support/MSL/MSL_C/PPC_EABI/Include \
			  -I$(MW_TOOL)/PowerPC_EABI_Support/MSL/MSL_C/MSL_Common/Include \
			  -I$(MW_TOOL)/PowerPC_EABI_Support/MSL/MSL_C++/PPC_EABI/Lib \
			  -I$(MW_TOOL)/PowerPC_EABI_Support/MSL/MSL_C++/PPC_EABI/Include \
			  -I$(MW_TOOL)/PowerPC_EABI_Support/MSL/MSL_C++/MSL_Common/Include

TOOL_CFiles		= __ppc_eabi_init.c MPC5554_HWInit.c Exceptions.c IntcInterrupts.c MPC55xx_init_debug.c \
			  etpu_util.c etpu_qd.c etpu_ppa.c

TOOL_LIB		= $(MW_TOOL)/PowerPC_EABI_Support/Runtime/Lib/Runtime.PPCEABI.E.a \
			  $(MW_TOOL)/PowerPC_EABI_Support/MSL/MSL_C/PPC_EABI/Lib/MSL_C.PPCEABI.bare.E.a

ECS_CFiles		= ecs.c fqd.c serial.c

ECS_INCLUDES	= -I../include \
			  -I../lib \
			  -I../freescale

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
#------------------------------------------------------------------------------


INCLUDES 		=  -I. \
			  $(TOOL_INCLUDES) \
			  $(ECS_INCLUDES)


#-------------------- Definitons for application files: -----------------------
#
# LAB_FILES		= overall lab C code (if applicable)
# ()_FILES		= C code for subcomponents of lab  (if applicable)
# SUPPORT_FILES		= supplimental C files
# S_FILES       	= all application specific assembly source files.
# APP_INCS		= all application specific include files.


#====================================
#============ LAB NAME ==============
#====================================
LAB 			= lab2


SUPPORT_FILES 		= $(TOOL_CFiles)\
			  $(ECS_CFiles)

LAB_FILES 		= $(LAB).c

ANGLE_FILES		= $(LAB)angle.c

CFLAGS 			= $(INCLUDES) \
			  $(CPP_REQ_DEFINES) \
			  $(EXT_CC_OPTS)

LFLAGS 			= $(TOOL_LIB) \
			  $(EXT_LIB) \
			  $(S_FUNCTIONS_LIB) \
			  -Map $(LAB).map

LFLAGS_ANGLE		= $(TOOL_LIB) \
			  $(EXT_LIB) \
			  $(S_FUNCTIONS_LIB) \
			  -Map $(LAB)angle.map

#
#------------------------ Construct the object file lists------------------------
#
SUPPORT_OBJECTS 	= $(addprefix ../freescale/,$(TOOL_CFiles:.c=.o)) $(addprefix ../lib/,$(ECS_CFiles:.c=.o))
LAB_OBJECTS 		= $(LAB_FILES:.c=.o)
ANGLE_OBJECTS 		= $(ANGLE_FILES:.c=.o)
OBJ_FILES 		= $(SUPPORT_OBJECTS) $(LAB_OBJECTS)
OBJ_FILES_ANGLE		= $(SUPPORT_OBJECTS) $(ANGLE_OBJECTS)


#
#---------------------------------------Build rules------------------------------
#
all: $(LAB).elf
	@echo ################ C Files ################
	@echo $(SUPPORT_FILES) $(LAB_FILES)
	@echo ################ Objects ################
	@echo $(OBJ_FILES)
	@echo ################ Binary ################
	@echo $^

lab2angle: $(LAB)angle.elf
	@echo ################ C Files ################
	@echo $(SUPPORT_FILES) $(ANGLE_FILES)
	@echo ################ Objects ################
	@echo $(OBJ_FILES_ANGLE)
	@echo ################ Binary ################
	@echo $^

clean:
	@echo clean
	@echo ==== Removing object files from build directory ====
	if exist *.o del *.o
	if exist ..\lib\*.o del ..\lib\*.o
	if exist *.elf del *.elf
	if exist *.out del *.out
	if exist *.mot del *.mot
	if exist *.map del *.map
	if exist *.bin del *.bin

$(LAB).elf: $(OBJ_FILES)
	@echo **************** BUILDING BINARY $@ ****************
	if exist $@ del $@
	$(LINK_TOOL) $^ -o $@ $(LINK_TOOL_OPTS) $(LFLAGS)

$(LAB)angle.elf: $(OBJ_FILES_ANGLE)
	@echo **************** BUILDING BINARY $@ ****************
	if exist $@ del $@
	$(LINK_TOOL) $^ -o $@ $(LINK_TOOL_OPTS) $(LFLAGS_ANGLE)

%.o: %.c
	@echo **************** BUILDING LAB FILE $@ ****************
	$(COMPILE_TOOL) $(COMPILE_TOOL_OPTS) $(CFLAGS) -o $@ -c $(@:.o=.c)

../lib/%.o: ../lib/%.c
	@echo **************** BUILDING LIBRARY FILE $@ ****************
	$(COMPILE_TOOL) $(COMPILE_TOOL_OPTS) $(CFLAGS) -o $@ -c ../lib/$(@:.o=.c)

../freescale/%.o: ../freescale/%.c
	@echo **************** BUILDING TOOL FILE $@ ****************
	$(COMPILE_TOOL) $(COMPILE_TOOL_OPTS) $(CFLAGS) -o $@ -c $(@:.o=.c)

# EOF: MAKEFILE
