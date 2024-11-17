#include <angle_converter.hpp>

// Function to divide a string into two parts
void divideString(const String& input, String& part1, String& part2) {
    size_t len = input.length();
    size_t mid = len / 2;

    part1 = input.substring(0, mid);
    part2 = input.substring(mid);
}

// Function to get the angle based on the input string
int greyCode2angle(const String& partinput) {
    if (partinput == "000") {
        return 0;
    } else if (partinput == "001") {
        // return 45;
        return 315;
    } else if (partinput == "011") {
        // return 90;
        return 270;
    } else if (partinput == "010") {
        // return 135;
        return 225;
    } else if (partinput == "110") {
        // return 180;
        return 180;
    } else if (partinput == "111") {
        // return 225;
        return 135;
    } else if (partinput == "101") {
        // return 270;
        return 90;
    } else if (partinput == "100") {
        // return 315;
        return 45;
    } else {
        return 0;
    }
}

// void setup() {
//     // Initialize serial communication
//     Serial.begin(9600);

//     // Example usage
//     String abc = "100101";
//     String part1, part2;

//     divideString(abc, part1, part2);

//     Serial.print("First part: ");
//     Serial.println(part1);
//     Serial.print("Second part: ");
//     Serial.println(part2);

//     int result2 = angle(part1);
//     int result3 = angle(part2);

//     Serial.print("First part angle: ");
//     Serial.println(result2);
//     Serial.print("Second part angle: ");
//     Serial.println(result3);
// }
