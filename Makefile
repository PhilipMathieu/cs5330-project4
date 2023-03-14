# Philip Englund Mathieu
# CS5330 Spring 2023
# 

# compiler and flags
CC = g++
CXXFLAGS = -std=c++17 -Wall -I$(CVPATH) -I$(INCLUDE)

# paths for executables, headers, source, and opencv
BINDIR = ./bin
INCLUDE = ./include
CVPATH = /usr/local/include/opencv4

# opencv libraries to link for this project
LIBS = -ltiff -lpng -ljpeg -lz -lopencv_core -lopencv_highgui -lopencv_video -lopencv_videoio -lopencv_imgcodecs -lopencv_imgproc -lopencv_objdetect

# params for testing


all: clean example

example:
	$(CC) -g $(CXXFLAGS) $^ -o $(BINDIR)/$@.out $(LIBS)

clean:
	rm -f *.o *~ bin/*.out
	rm -f data/*

clean-template:
	rm environment.yml INSTALL.md LICENSE.md README.md */.gitkeep
	echo "# Blank Project" > README.md
