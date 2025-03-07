# Compiler
CC = g++ 

# Compiler flags
CFLAGS = -Wall 

# Source files for camera calibration
CALIB_SRCS = calib.cc readParams.cc readData.cc utils.cc matrixUtils.cc

# Source files for rectification test
RECT_SRCS = rectification.c utils.cc matrixUtils.cc

# Object files (replace .cc with .o)
CALIB_OBJS = $(CALIB_SRCS:.cc=.o)
RECT_OBJS = $(RECT_SRCS:.c=.o) $(RECT_SRCS:.cc=.o)

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
	rm -f $(CALIB_OBJS) $(RECT_OBJS) $(CALIB_TARGET) $(RECT_TARGET)

# Run the calibration program
run-calib: $(CALIB_TARGET)
	./$(CALIB_TARGET)

# Run the rectification test program
run-rect: $(RECT_TARGET)
	./$(RECT_TARGET)