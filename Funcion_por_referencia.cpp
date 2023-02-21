#include <iostream>
using namespace std;

void increment(int &number) {
    cout << "The address of number is: " << &number << endl;
    number++;
    cout << "Now the value of number is: " << number << endl;
}

int main() {
    
    // Declare the number
    int number = 42;
    // Print the number's value
    cout << " The value of number is: " << number << " and the address is: " << &number << endl;
    // Call the function increment
    increment(number);
    
    
    return 0;
}