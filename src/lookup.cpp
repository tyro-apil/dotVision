#include "lookup.hpp"

// Function to initialize the Braille map
std::map<char, String> initializeBrailleMap() {
    std::map<char, String> brailleMap;

    // Mapping lowercase alphabets to Braille
    brailleMap['a'] = "100000";
    brailleMap['b'] = "110000";
    brailleMap['c'] = "100100";
    brailleMap['d'] = "100110";
    brailleMap['e'] = "100010";
    brailleMap['f'] = "110100";
    brailleMap['g'] = "110110";
    brailleMap['h'] = "110010";
    brailleMap['i'] = "010100";
    brailleMap['j'] = "010110";
    brailleMap['k'] = "101000";
    brailleMap['l'] = "111000";
    brailleMap['m'] = "101100";
    brailleMap['n'] = "101110";
    brailleMap['o'] = "101010";
    brailleMap['p'] = "111100";
    brailleMap['q'] = "111110";
    brailleMap['r'] = "111010";
    brailleMap['s'] = "011100";
    brailleMap['t'] = "011110";
    brailleMap['u'] = "101001";
    brailleMap['v'] = "111001";
    brailleMap['w'] = "010111";
    brailleMap['x'] = "101101";
    brailleMap['y'] = "101111";
    brailleMap['z'] = "101011";

    // Mapping uppercase alphabets to Braille (with capitalization prefix)
    // brailleMap['A'] = "000001100000";
    // brailleMap['B'] = "000001110000";
    // brailleMap['C'] = "000001100100";
    // brailleMap['D'] = "000001100110";
    // brailleMap['E'] = "000001100010";
    // brailleMap['F'] = "000001110100";
    // brailleMap['G'] = "000001110110";
    // brailleMap['H'] = "000001110010";
    // brailleMap['I'] = "000001010100";
    // brailleMap['J'] = "000001010110";
    // brailleMap['K'] = "000001101000";
    // brailleMap['L'] = "000001111000";
    // brailleMap['M'] = "000001101100";
    // brailleMap['N'] = "000001101110";
    // brailleMap['O'] = "000001101010";
    // brailleMap['P'] = "000001111100";
    // brailleMap['Q'] = "000001111110";
    // brailleMap['R'] = "000001111010";
    // brailleMap['S'] = "000001011100";
    // brailleMap['T'] = "000001011110";
    // brailleMap['U'] = "000001101001";
    // brailleMap['V'] = "000001111001";
    // brailleMap['W'] = "000001010111";
    // brailleMap['X'] = "000001101101";
    // brailleMap['Y'] = "000001101111";
    // brailleMap['Z'] = "000001101011";

    // Mapping numbers to Braille (with number sign prefix)
    // brailleMap['1'] = "001111100000";
    // brailleMap['2'] = "001111110000";
    // brailleMap['3'] = "001111100100";
    // brailleMap['4'] = "001111100110";
    // brailleMap['5'] = "001111100010";
    // brailleMap['6'] = "001111110100";
    // brailleMap['7'] = "001111110110";
    // brailleMap['8'] = "001111110010";
    // brailleMap['9'] = "001111010100";
    // brailleMap['0'] = "001111010110";

    // Mapping symbols to Braille
    brailleMap['.'] = "010011";
    brailleMap[','] = "010000";
    brailleMap[';'] = "011000";
    brailleMap[':'] = "010010";
    brailleMap['!'] = "011010";
    brailleMap['('] = "011011";
    brailleMap[')'] = "011011";
    brailleMap['?'] = "011001";
    brailleMap['-'] = "001001";
    brailleMap['\''] = "001000";
    brailleMap['\"'] = "001010";
    brailleMap['/'] = "001100";
    brailleMap['\\'] = "001101";
    brailleMap['+'] = "011111";
    brailleMap['*'] = "010111";
    brailleMap['='] = "111011";
    brailleMap['@'] = "000111";
    brailleMap['#'] = "001111";
    brailleMap['$'] = "111101";
    brailleMap['%'] = "110101";
    brailleMap['&'] = "111111";
    brailleMap['_'] = "001011";
    brailleMap['<'] = "011101";
    brailleMap['>'] = "011110";
    brailleMap['['] = "011011";
    brailleMap[']'] = "011011";
    brailleMap['{'] = "011011";
    brailleMap['}'] = "011011";
    brailleMap['^'] = "001110";
    brailleMap['~'] = "001111";
    brailleMap['|'] = "011000";
    brailleMap['`'] = "000010";

    return brailleMap;
}

// Function to get Braille representation of a character
String getBraille(char c, const std::map<char, String>& brailleMap) {
    auto it = brailleMap.find(c);
    if (it != brailleMap.end()) {
        return it->second;
    } else {
        return "000000"; // Return empty Braille pattern for unknown characters
    }
}