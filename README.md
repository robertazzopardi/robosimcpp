# Robosim

C++ implementation of Robosim https://cgi.csc.liv.ac.uk/~trp/COMP329RoboSim.html

## Requirements

- CMake
- Cpp Compiler (This project uses clang)
- SDL2 (add the sdl2 install directory to path)
- include-what-you-use (Optional)

## Installation

### Building

Set SDL2_LIBRARY (to the sdl2 library path) and SDL2_INCLUDE_DIR (to the sdl2 header file path) environement variables

And build with:

```sh
cmake --build build --config Release
```

### Installing

Building is enough if you would like to choose the install directory for robosim,
otherwise installing with cmake will move the library file and the headers (RobotMonitor.h, EnvController.h and Colour.h) to the local directory. On linux and Macos the files go to /usr/local/lib and /usr/local/include/robosim

```sh
cmake --build build --config Release --target install
```

## Usage

See examples/main.cpp and compile and link with:

```sh
clang++ -Wall -Werror -Wextra -o main main.cpp -I/usr/local/include/robosim -L/usr/local/lib -lrobosim
```

```sh
./main
```

Ensuring you include -I{PATH_TO_ROBOSIM_HEADERS} and link -L{PATH_TO_ROBOSIM_LIBRARY}

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License

[MIT](https://choosealicense.com/licenses/mit/)
