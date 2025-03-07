ODRoNeS (OpenDRIVE Road Network System)
=======================================
ODRoNeS is a simulation-ready road network system supporting the OpenDRIVE standard.
 While its main purpose is allow you to use both cartesian and road coordinates interchangeably,
 it can also be used for visualisation purposes.


The library is written in C++ and is shipped with few open-source dependencies: 

 - [cxxopts](https://github.com/jarro2783/cxxopts)
 - [tinyxml2](https://github.com/leethomason/tinyxml2)
 - [clothoids](https://github.com/ebertolazzi/Clothoids)

and it uses [Qt](https://www.qt.io) for visualisation. 


Install
-------
Building the library only needs a C++ compiler and CMake. 
 Visualisation is achieved using Qt. If not required, the flag ` USE_QT=OFF ` should be passed
 at configure time.
Thus, on a Unix platform one can use `make`:

    mkdir build 
    cmake ../ -DCMAKE_INSTALL_PREFIX=<install-path>
    make
    make install 

and on Windows one: 
 
    mkdir build 
    cmake ../ -DCMAKE_INSTALL_PREFIX=<install-path>
    cmake --build . --config=Release --target install

Alternatively, if using Microsoft Visual Studio, you can open the created solution after configuring with CMake.


Usage
-----
In order to learn how to use the library and the headers, have a look at the `doc` folder. 
 In order to visualise your map, you can use `rnscheck`. 
 Launch it from the command line passing the path to the map as the last argument:

    ./rnscheck <map-file.xodr>

You can also identify the different lanes with labels and colours passing the `-i` flag:

    ./rnscheck -i <map-file.xodr>




Documentation
-------------
Details on the structure of the library can be found in ` doc `. 
 The code is also heavily commented.


OpenDRIVE Support
-----------------
The coverage of the standard 1.8.1 is quite complete though it still misses:

 * Elevation and superelevation.
 * Several traffic signs.
 * Poly3 geometry (though it is deprecated).
 * Rail and railway stations (not planned).   

While there's still work to be done, the library is now strong enough to go public. 


Author
------
 * Albert Solernou  
   Institute for Transport Studies  
   University of Leeds   
   email: a.solernou@leeds.ac.uk  
   homepage: [https://environment.leeds.ac.uk/transport/staff/978/dr-albert-solernou](https://environment.leeds.ac.uk/transport/staff/978/dr-albert-solernou)

   