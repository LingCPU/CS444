# Compiler
CC = g++ 

# Compiler flags
CFLAGS = -Wall 

# Source files for camera calibration
CALIB_SRCS = calib.cc readParams.cc readData.cc utils.cc matrixUtils.cc

# Source files for rectification test
RECT_SRCS = rectification_main.c rectification.c utils.cc matrixUtils.cc

# Object files
CALIB_OBJS = $(CALIB_SRCS:.cc=.o)
RECT_OBJS = rectification_main.o rectification.o utils.o matrixUtils.o

# Output executables
CALIB_TARGET = calib
RECT_TARGET = rectification_test

# Default rule
all: $(CALIB_TARGET) $(RECT_TARGET)

# Rule to build the final calibration executable
$(CALIB_TARGET): $(CALIB_OBJS)
	$(CC) $(CFLAGS) -o $(CALIB_TARGET) $(CALIB_OBJS)

# Rule to build the rectification test executable
$(RECT_TARGET): $(RECT_OBJS)
	$(CC) $(CFLAGS) -o $(RECT_TARGET) $(RECT_OBJS) -lm

# Rule to build object files from .cc files
%.o: %.cc
	$(CC) $(CFLAGS) -c $< -o $@

# Rule to build object files from .c files
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Clean rule
clean:
	rm -f *.o $(CALIB_TARGET) $(RECT_TARGET)

# Run the calibration program
run-calib: $(CALIB_TARGET)
	./$(CALIB_TARGET)

# Run the rectification test program
run-rect: $(RECT_TARGET)
	./$(RECT_TARGET)