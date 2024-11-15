#! /usr/env/bin/python3


class angler():
    def __init__(self) -> None:
        pass
    
    def braille_to_text(self, braille):
        part1, part2 = self.divide_string(braille)
        top_angle = self.get_angles(part1)
        bottom_angle = self.get_angles(part2)
        return top_angle, bottom_angle
    
    def get_angles(self,braille):
        if braille=="000" :
            return 0
        
        elif braille=="001":
            return 45
        
        elif (braille=="011"):
            return 90
        
        elif (braille=="010"):
            return 135
            
        elif (braille=="110"):
            return 180
            
        elif (braille=="111"):
            return 225
            
        elif (braille=="101"):
            return 270

        elif(braille=="100"):
            return 315

        else:
            print("Invalid Braille")
            return 

    def divide_string(self,input_string):
        mid = len(input_string) // 2
        part1 = input_string[:mid]
        part2 = input_string[mid:]
        return part1, part2

