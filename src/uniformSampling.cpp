#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <fstream>
#include "geometry.h"
#include "numerical.h"


class ddNumerical : public numerical
{
public:
    // ddNumerical();
    void nSetupPointsXYUniformly(scalar ds) override
    {
        // std::string iFile = "verticesAtOrigin.csv";
	std::string iFile = "object_vertices_july.csv";
        std::ifstream fin;
        fin.open(iFile, std::ifstream::in);
        if (!fin.is_open())
        {
            std::cerr << "[ err ] We could not open " << iFile << std::endl;
            return;
        }

        std::vector<std::vector<scalar>> p;
        std::string line;
        getline(fin, line); // ignore the one line header;
        while (getline(fin, line))
        {
            p.push_back(lineParser(line));
        }

        std::cout << "points 0: " << p[0][0] << ", " << p[0][1] << std::endl;
        std::cout << "points 1: " << p[1][0] << ", " << p[1][1] << std::endl;
        std::cout << "points 2: " << p[2][0] << ", " << p[2][1] << std::endl;
        std::cout << std::endl;
        std::cout << "p.size(): " << p.size() << std::endl;



        // Create a functional (though shitty) numerical lane:
        numerical::allocateMemory(p.size());
        _pointsS[0] = 0;
        _pointsX[0] = p[0][0];
        _pointsY[0] = p[0][1];
        scalar precision = 1e-3;
        uint stop = 4;
        for (uint i = 1; i < p.size(); ++i)
        {
            _pointsX[i] = p[i][0];
            _pointsY[i] = p[i][1];
            scalar d = mvf::distance({_pointsX[i-1], _pointsY[i-1]},
                                     {_pointsX[i],   _pointsY[i]});
            _pointsS[i] = _pointsS[i-1] + d;

	    /*
            // Check that -atan2(x,y) is "h" really:
            arr2 t = mvf::tangent({_pointsX[i-1], _pointsY[i-1]},
                                  {_pointsX[i],   _pointsY[i]});
            scalar at2 = - std::atan2(t[0], t[1]) * 180 / ct::pi;
            scalar h = p[i][3];
            if (!mvf::areCloseEnough(at2, h, precision))
            {
                std::cerr << "[ Err ] " << std::abs(at2 - h) << " > " << precision << ", or more explicitly h: " << h << " and -atan2(x,y) " << at2 << std::endl;
            }
	    */
        }
        std::cout << "_pointsS[_pointsSize -1]: " << _pointsS[_pointsSize -1] << std::endl;
        _approxDs = _pointsS[_pointsSize - 1] / (_pointsSize - 1);
        std::cout << "approxDs: " << _approxDs << std::endl;


        // Calculate a new set of points that are uniformly distributed.
        uint newPointsSize = 1 + _pointsS[_pointsSize -1] / ds;
        scalar newDs = _pointsS[_pointsSize -1] / (newPointsSize -1);
        std::cout << "newDs: " << newDs << std::endl;
        std::cout << "newPointsSize: " << newPointsSize << std::endl;

        std::vector<arr2> uniform(newPointsSize);
        for (uint i = 0; i < newPointsSize; ++i)
        {
            uniform[i] = interpolate(newDs * i);
        }

        std::vector<scalar> headings(newPointsSize);
        for (uint i = 0; i < newPointsSize -1; ++i)
        {
            arr2 t = mvf::tangent(uniform[i], uniform[i+1]);
            headings[i] = - std::atan2(t[0], t[1]) * 180 / ct::pi;
        }
        headings[newPointsSize -1] = headings[newPointsSize -2];


        std::string oFile = "uniformsample.csv";
        std::ofstream fout;
        fout.open(oFile, std::ifstream::out);
        if (!fout.is_open())
        {
            std::cerr << "[ err ] We could not open " << oFile << std::endl;
            return;
        }

        for (uint i = 0; i < newPointsSize; ++i)
        {
            std::string line = std::to_string(uniform[i][0]) + "," +
                    std::to_string(uniform[i][1]) + ",0.000000," +
                    std::to_string(headings[i])
                    + ",0.000000,0.000000";
            fout << line << std::endl;
        }
        fout.close();
    };


private:
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


};


int main(int argc, char* argv[])
{
    std::cout << "uniform sampling " << std::endl;
    ddNumerical ddn;
    ddn.nSetupPointsXYUniformly(2.0);

    return 0;
}
