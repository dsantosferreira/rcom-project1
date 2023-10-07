CC = gcc  # Compiler to use
CFLAGS = -Wall  # Compilation flags: -Wall shows warnings
TARGET = myprogram  # Name of the final executable file

# List of all .c files
SRC = main.c data_link/data_link.c

# List of all .o files
OBJ = $(SRC:.c=.o)

# Default rule: build the target
all: $(TARGET)

# Rule to build the target
$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) -o $@ $^

# Rule to build .o files from .c files
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Rule to clean intermediate and final artifacts
clean:
	rm -f $(OBJ) $(TARGET)

# Rule to run the sender
run_sender: $(TARGET)
	./$(TARGET) /dev/ttyS10 s

# Rule to run the receiver
run_receiver: $(TARGET)
	./$(TARGET) /dev/ttyS11 r