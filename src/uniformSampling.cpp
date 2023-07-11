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
    getline(fin, line); // ignore the one line header;
    while (getline(fin, line))
    {
        p.push_back(lineParser(line));
    }

    for (uint i = 1; i < p.size(); ++i)
    {
        arr2 po = {p[i-1][0], p[i-1][1]};
        arr2 pi = {p[i][0],   p[i][1]};
        arr2 t = mvf::tangent(po, pi);
        if (!mvf::areCloseEnough(std::atan2(t[1], t[0]), p[i-1][3], 1e-6))
        {
            std::cout << "Err!" << std::endl;
            return 1;
        }
    }

    return 0;
}
