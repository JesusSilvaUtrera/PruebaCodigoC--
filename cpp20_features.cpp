#include <iostream>
#include <vector>
#include <ranges>
#include <compare>

// C++20 Feature: Designated Initializers
// Allows initializing members of a struct by name, making code clearer.
struct Point {
    int x;
    int y;
};

// C++20 Feature: Spaceship Operator (<=>)
// Simplifies comparison logic by automatically generating all relational operators.
struct Value {
    int val;

    // The compiler will generate ==, !=, <, <=, >, >= based on this single operator.
    auto operator<=>(const Value& other) const = default;
};

void demonstrate_ranges() {
    std::cout << "--- Demonstrating std::ranges ---" << std::endl;
    std::vector<int> numbers = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    std::cout << "Original numbers: ";
    for (int n : numbers) {
        std::cout << n << " ";
    }
    std::cout << std::endl;

    // 'views::filter' creates a view of even numbers without copying them.
    // 'views::transform' creates a view that squares each of those even numbers.
    // The pipe operator | chains these operations together in a readable way.
    auto result = numbers
                | std::views::filter([](int n) { return n % 2 == 0; })
                | std::views::transform([](int n) { return n * n; });

    std::cout << "Squares of even numbers: ";
    for (int n : result) {
        std::cout << n << " ";
    }
    std::cout << "\n" << std::endl;
}

void demonstrate_designated_initializers() {
    std::cout << "--- Demonstrating Designated Initializers ---" << std::endl;
    // Initialize 'p1' with named members for clarity.
    Point p1 = {.x = 10, .y = 20};
    std::cout << "Point p1 initialized with designated initializers: x=" << p1.x << ", y=" << p1.y << std::endl;

    // The initializers must be in the same order as the member declarations.
    Point p2 = {.x = 40, .y = 30};
    std::cout << "Point p2 initialized with members in order: x=" << p2.x << ", y=" << p2.y << "\n" << std::endl;
}

void demonstrate_spaceship_operator() {
    std::cout << "--- Demonstrating Spaceship Operator (<=>) ---" << std::endl;
    Value v1{10};
    Value v2{20};

    std::cout << "Comparing v1(10) and v2(20):" << std::endl;
    std::cout << "v1 == v2: " << std::boolalpha << (v1 == v2) << std::endl;
    std::cout << "v1 != v2: " << std::boolalpha << (v1 != v2) << std::endl;
    std::cout << "v1 < v2: " << std::boolalpha << (v1 < v2) << std::endl;
    std::cout << "v1 > v2: " << std::boolalpha << (v1 > v2) << std::endl;
}

int main() {
    std::cout << "Welcome to the C++20 Features Showcase!" << std::endl;
    std::cout << "========================================" << std::endl;

    demonstrate_ranges();
    demonstrate_designated_initializers();
    demonstrate_spaceship_operator();

    return 0;
}
