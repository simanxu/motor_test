CC = gcc
LDFLAGS = -g -Wall -o
LINKOPTS = -lpthread -lrt

EXEC=motor_control
OBJECTS=motor_control.o

all:$(EXEC)

$(EXEC):$(OBJECTS)
	$(CC) $(LDFLAGS) $^ $@ $(LINKOPTS)