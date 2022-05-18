#include <iostream>

void * get_pointer(){
    static int number = 7;
    int *p2number = &number;
    std::cout << "pointer_ref" << (void*) p2number << std::endl;
    return ((void *) p2number);
}

void set_pointer(void * pointer){
    std::cout << "int value from pointer " << *((int*) pointer) << std::endl;
    std::cout << "void pointer " << pointer << std::endl;
} 