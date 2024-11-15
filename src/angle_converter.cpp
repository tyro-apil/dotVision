#include <iostream>
#include <string>
#include <cmath>

std::pair<std::string, std::string> divideString(const std::string& input) {
    size_t mid = input.size() / 2;
    std::string part1 = input.substr(0, mid);
    std::string part2 = input.substr(mid);
    return {part1, part2};
}

int angle(const std::string& partinput)
{
    if(partinput=="000"){
        return 0;
    }
    else if (partinput=="001")
    {
        return 45;
    }
    else if (partinput=="011")
    {
        return 90;
    }
    else if (partinput=="010")
    {
        return 135;
    }
    else if (partinput=="110")
    {
        return 180;
    }
    else if (partinput=="111")
    {
        return 225;
    }
    else if (partinput=="101")
    {
        return 270;
    }
    else if(partinput=="100")
    {
        return 315;
    }
    else
    {
        std::cout << "Invalid string"<< std::endl;
        return 0;
    }
        std::cout << "here"<< std::endl;
}

int main(){
    std::string abc="100101";
    auto result = divideString(abc);
    std::cout << "First part: " << result.first << std::endl;
    std::cout << "Second part: " << result.second << std::endl;

    auto result2 = angle(result.first);
    auto result3 = angle(result.second);
    std::cout << "First part: " << result2 << std::endl;
    std::cout << "Second part: " << result3 << std::endl;
    return 0;
}

