#include <iostream>

using namespace std;

void squared_array(int array_num[6]) {
    //Declarar puntero y asignar array numerico
    int *pointer = array_num;
    //Bucle para cambiar cada valor por su cuadrado
    for (int i=0; i<6; i++) {
        *pointer = (*pointer)*(*pointer);
        cout << *pointer << endl;
        //Aumentamos puntero para que apunte al siguiente elem del array
        pointer++;
    }
}

int main() {
    int array_num[6] = {4, 8, 15, 16, 23, 42};
    squared_array(array_num);
    return 0;
}