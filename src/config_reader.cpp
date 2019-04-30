//
// Created by hamlet on 4/30/19.
//


#include <fstream>
#include <algorithm>
#include <iostream>
#include <map>
#include "../include/config_reader.h"


void read_data()
{
    std::ifstream cFile (config_file);
    if (cFile.is_open())
    {
        std::string line;
        while(getline(cFile, line)){
            line.erase(std::remove_if(line.begin(), line.end(), isspace),
                       line.end());
            if(line[0] == '#' || line.empty()) {
                continue;
            }
            auto delimiterPos = line.find("=");
            auto name = line.substr(0, delimiterPos);
            auto value = line.substr(delimiterPos + 1);
            std::cout << name << " " << value << '\n';

            vars[name] = value;


        }

    }
    else {
        std::cerr << "Couldn't open config file for reading.\n";
    }
}