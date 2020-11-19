ROOTNAME=main
RUNOPTIONS=
CC=gcc
CFLAGS= -lwiringPi -lpthread
CFLAGS2=
LIBS =
DEPS = 
OBJ = $(ROOTNAME).c

$(ROOTNAME): $(OBJ)
	$(CC) -o $(ROOTNAME) $(OBJ) $(CFLAGS)

clean:
	rm  $(ROOTNAME) 

run: $(ROOTNAME)
	sudo ./$(ROOTNAME) $(RUNOPTIONS)
