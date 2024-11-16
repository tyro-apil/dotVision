#pragma once

#include <Arduino.h>
#include <map>

std::map<char, String> initializeBrailleMap();

String getBraille(char c, const std::map<char, String>& brailleMap);