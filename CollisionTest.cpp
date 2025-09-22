#include "CollisionChecking.h"
#include <vector>

Rectangle ob1 = Rectangle{2.0, 2.9, 0.5, 1.0};
Rectangle ob2 = Rectangle{2.0, 1.2, 0.5, 1.0};
Rectangle ob3 = Rectangle{3.5, 2.0, 0.5, 1.0};
std::vector<Rectangle> obstacles = {ob1, ob2, ob3};

int main (){
    if(isValidSquare(2.0, 1.19, M_PI/2.0, 0.5, obstacles, 5.0, 5.0)){
        std::cout << "Valid Square\n";
    }else{ std::cout << "Invalid Square\n";}
}