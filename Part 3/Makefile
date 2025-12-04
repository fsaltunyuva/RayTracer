CXX = g++
CXXFLAGS = -std=c++17 -O3 -fopenmp -Iinclude -Ijson -Iglm -Iply -Istb
TARGET = raytracer

SRC = src/main.cpp \
      src/Camera.cpp \
      src/Light.cpp \
      src/Material.cpp \
      src/Object.cpp \
      src/Scene.cpp \
      src/Vec3.cpp \
      src/BVH.cpp \
      src/Intersector.cpp \
      src/Transformation.cpp

OBJ = $(SRC:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)
