# Compiler
CC = g++ 

# Compiler flags
CFLAGS = -Wall -I/usr/include/opencv4
#Load Lib Path
LDLIBPATH = -I/usr/include/opencv4 

#Load Libraries
LDLIBS = -I/usr/include/opencv4 -ljpeg -lm -lopencv_core -lopencv_imgproc -lopencv_calib3d -lopencv_highgui -lopencv_imgcodecs -lopencv_calib3d -lopencv_features2d -llapacke -llapack -lblas

# Source files
SRCS =  pRectify.cc


# Object files (replace .cc with .o)
OBJS = $(SRCS:.cc=.o)

# Output executable
TARGET =  pRectify

#Rule to build the final executable
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJS) $(LDLIBS)

# Rule to build object files
%.o: %.cc
	$(CC) $(CFLAGS) -c $< -o $@

# Clean rule
clean:
	rm -f $(OBJS) $(TARGET)

# Run the program
run: $(TARGET)
	./$(TARGET)

