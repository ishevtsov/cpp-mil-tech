#include <iostream>

int main()
{
    std::cout << "Hello, World!" << std::endl;

    int a[] = {1, 2, 3, 4, 5};
    for (int i : a)
    {
        std::cout << i << " ";
    }
    std::cout << std::endl;

    return 0;
}
