// 
//  This file is part of the ODRoNeS (OpenDRIVE Road Network System) package.
//  
//  Copyright (c) 2023 Albert Solernou, University of Leeds.
// 
//  GTSmartActors is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
// 
//  GTSmartActors is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
// 
//  You should have received a copy of the GNU General Public License
//  along with ODRoNeS. If not, see <http://www.gnu.org/licenses/>.
// 
//  We would appreciate that if you use this software for work leading 
//  to publications you cite the package and its related publications. 
//

#include <QApplication>

#include <filesystem>
#include <fstream>

#include "mainwindow.h"
#include "cxxopts.hpp"
#include "rns.h"


static double rnsversion = 0.8;

int main(int argc, char *argv[])
{

    cxxopts::Options options(argv[0], "Allowed options");
    options.positional_help("[optional args]");
    options.show_positional_help();

    options.set_width(70);
    options.set_tab_expansion();
    options.allow_unrecognised_options();

    options.add_options()
        ("h,help", "Print usage message")
            ("m,map", "Input file map", cxxopts::value<std::string>())
            ("v,version", "Print RNS version")
            ("i,identify", "Identify lanes");

    options.parse_positional("map");

    auto result = options.parse(argc, argv);
    if (result.count("help"))
    {
    std::cout << options.help() << std::endl;
    return 0;
    }


    // Option 1 - Version
    if (result.count("version"))
    {
        std::cout << "Road Network System version: " << rnsversion << std::endl;
        return 0;
    }

    // Option 2 - Map
    std::string iFile = "";
    if (result.count("map"))
    {
        iFile = result["map"].as<std::string>();
        std::filesystem::path fsIFile = iFile;
        fsIFile = std::filesystem::canonical(fsIFile);
        std::ostringstream em;

        // check that it exists:
        if (std::filesystem::exists(fsIFile))
        {
            if (!std::filesystem::is_regular_file(fsIFile))
            {
                std::string errMsg = "input file " + fsIFile.string() + " is not a regular file!";
                std::cerr << errMsg << std::endl;
                return 1;
            }
            else
            {
                std::ifstream fin;
                fin.open(fsIFile.string(), std::ifstream::in);
                if (!fin.is_open())
                {
                    std::string errMsg = "unable to read input file: " + fsIFile.string();
                    std::cerr << errMsg << std::endl;
                    return 1;
                }
                else fin.close();
            }

        }
        else
        {
            std::string errMsg = "input file: " + fsIFile.string() + " could not be found ";
            std::cerr << errMsg << std::endl;
            return 1;
        }

        iFile = fsIFile.string();
    }
    else
    {
        std::cerr << "You need to pass in an input map to load!" << std::endl;
        return 1;
    }

    // Option 3 - Identify lanes
    bool identifyLanes = false;
    if (result.count("identify"))
        identifyLanes = true;


    QApplication app(argc, argv);
    MainWindow w(iFile, identifyLanes);
    w.show();
    return app.exec();
}
