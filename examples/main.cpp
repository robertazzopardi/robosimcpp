#include <iostream>
#include <robosim.h>

#define CONFIG_NAME "./defaultConfig.txt"

class Robot : public robosim::RobotMonitor {
  private:
  public:
    Robot(int delay, bool verbose) : robosim::RobotMonitor(delay, verbose) {}
    void run(bool *in) {
        std::cout << "Hello Robot " << *in << std::endl;
        travel();
    }
};

int main() {
    Robot robot(1000, false);
    robosim::EnvController env(CONFIG_NAME, 7, 7, &robot);

    env.updateEnv();

    return 0;
}

// clang++ -Wall -Werror -Wextra -std=c++20 -o main main.cpp -I/usr/local/include/robosim -L/usr/local/lib -lrobosim

// DON'T NEED: NO NEED TO LINK SDL
// clang++ -Wall -Werror -Wextra -std=c++20 -D_THREAD_SAFE `sdl2-config --cflags --libs` -o main main.cpp -I/usr/local/include/robosim -L/usr/local/lib -lrobosim
