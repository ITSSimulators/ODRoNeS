#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <fstream>
#include "numerical.h"

std::vector<scalar> lineParser(std::string line)
{
    std::stringstream ss(line);
    std::vector<scalar> result;

    while( ss.good() )
    {
        std::string substr;
        getline( ss, substr, ',' );
        result.push_back( std::stod( substr ) );
    }

    return result;

}


int main(int argc, char* argv[])
{
    std::string iFile = "verticesAtOrigin.csv";
    std::cout << "uniform sampling of " << iFile << std::endl;

    std::ifstream fin;
    fin.open(iFile, std::ifstream::in);
    if (!fin.is_open())
    {
        std::cerr << "[ err ] We could not open " << iFile << std::endl;
        return 1;
    }

    std::vector<std::vector<scalar>> p;
    std::string line;
    while (getline(fin, line))
    {
        p.push_back(lineParser(line));
    }

    return 0;
}
