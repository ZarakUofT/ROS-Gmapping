#include "math.h"

int main(){
    std::vector<float> test = Math::linspace(-5, 5, 7);

    for (auto i: test)
        std::cout << i << ", ";

    return 0;
}