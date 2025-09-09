// To compile and run this file, use the following commands:
// g++ -std=c++20 -o cpp20_features cpp20_features.cpp
// ./cpp20_features

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

    // The pipe operator `|` is a key part of the std::ranges library. It allows you to
    // chain "range adaptors" (like `views::filter` and `views::transform`) together
    // to create a processing pipeline. Data flows from left to right.
    //
    // 1. `numbers` is the input range.
    // 2. It's "piped" into `std::views::filter`, which creates a lazy view of only the even numbers.
    // 3. The result of the filter is then "piped" into `std::views::transform`, which creates
    //    another lazy view that contains the square of each number from the previous step.
    //
    // This approach is highly efficient because no intermediate containers are created.
    // The operations are performed on-demand as you iterate over the final `result` view.
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
