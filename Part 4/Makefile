CXX = g++
CXXFLAGS = -std=c++17 -O3 -fopenmp -Iinclude -Ijson -Iglm -Iply -Istb
TARGET = raytracer

SRC = src/main.cpp \
      src/Scene.cpp \
      src/Intersector.cpp \
      src/BVH.cpp \
      src/Transformation.cpp \
      src/Image.cpp \
      src/Vec2.cpp \
      src/Vec3.cpp

OBJ = $(SRC:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)
