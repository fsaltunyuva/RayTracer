CXX = g++
CC = gcc

CXXFLAGS = -std=c++17 -O3 -fopenmp -Iinclude -Ijson -Iglm -Iply -Istb -Itinyexr
CFLAGS = -O3 -Itinyexr
TARGET = raytracer

SRC_CPP = src/main.cpp \
          src/Scene.cpp \
          src/Intersector.cpp \
          src/BVH.cpp \
          src/Transformation.cpp \
          src/Image.cpp \
          src/ImageIO.cpp \
          src/Tonemapper.cpp \
          src/Vec2.cpp \
          src/Vec3.cpp \
          src/BRDF.cpp

SRC_C = tinyexr/miniz.c

OBJ_CPP = $(SRC_CPP:.cpp=.o)
OBJ_C = $(SRC_C:.c=.o)
OBJ = $(OBJ_CPP) $(OBJ_C)

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)
