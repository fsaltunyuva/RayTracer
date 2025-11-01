CXX = g++
CXXFLAGS = -std=c++17 -O3 -fopenmp -Iinclude
TARGET = raytracer

SRC = src/main.cpp \
      src/Camera.cpp \
      src/Light.cpp \
      src/Material.cpp \
      src/Object.cpp \
      src/Scene.cpp \
      src/Vec3.cpp \
      ply/plyfile.c

OBJ = $(SRC:.cpp=.o)
OBJ := $(OBJ:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

%.o: %.c
	$(CXX) $(CXXFLAGS) -x c -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)
