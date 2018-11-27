GXX = g++
GXXFLAG = -std=c++11
FLAG = -Wall -O3

SRC = fixed-outline_floorplanning.cpp
EXE = hw3

all::$(EXE)

$(EXE): $(SRC)
	$(GXX) $(GXXFLAG) $(FLAG) -o $@ $^

clean:
	rm -rf $(EXE)
