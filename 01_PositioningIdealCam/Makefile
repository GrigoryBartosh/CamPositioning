CXX = g++
CXXFLAGS = -std=c++11 -O2 -Wall -Wextra -g -Iinclude `pkg-config --cflags opencv` `pkg-config --cflags libconfig++`
LDFLAGS = `pkg-config --libs opencv` `pkg-config --libs libconfig++`

EXE = main
SRCDIR = src
BINDIR = bin

OBJECTS = $(patsubst $(SRCDIR)/%.cpp,$(BINDIR)/%.o,$(wildcard $(SRCDIR)/*.cpp))

all: $(EXE)

$(EXE): $(BINDIR) $(OBJECTS)
	$(CXX) $(OBJECTS) -o $(EXE) $(LDFLAGS)

$(BINDIR)/%.o: $(SRCDIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c -MMD -o $@ $<

include $(wildcard $(BINDIR)/*.d)

$(BINDIR):
	mkdir -p $(BINDIR)

rand: all
	./$(EXE) rand

rand_short: all
	./$(EXE) rand_short

ransac: all
	./$(EXE) ransac

ransac_short: all
	./$(EXE) ransac_short

generate: all
	./$(EXE) generate

clean:
	rm -rf $(BINDIR) $(EXE)

.PHONY: clean all
