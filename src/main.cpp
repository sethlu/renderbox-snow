#include <iostream>
#include <unordered_map>

void launchDemoSnowball(int argc, char const **argv);

void launchSimGenSnowball(int argc, char const **argv);

void launchSimScene0(int argc, char const **argv);

void launchVizScene0(int argc, char const **argv);

int main(int argc, char const **argv) {

    std::unordered_map<std::string, void (*)(int argc, char const **argv)> routines;
    routines.insert(std::make_pair("sim-scene0", launchSimScene0));
    routines.insert(std::make_pair("sim-gen-snowball", launchSimGenSnowball));
#if USE_RENDERBOX
    routines.insert(std::make_pair("demo-snowball", launchDemoSnowball));
    routines.insert(std::make_pair("viz-scene0", launchVizScene0));
#endif //USE_RENDERBOX

    if (argc < 2) {
        std::cout << "Usage: ./snow [launcher]" << std::endl;

        std::cout << "Available launchers:" << std::endl;
        for (auto it : routines) {
            std::cout << "* " << it.first << std::endl;
        }

        exit(1);
    }

    std::string routine = argv[1];
    auto it = routines.find(routine);
    if (it == routines.end()) {
        std::cout << "Launcher " << routine << " not found" << std::endl;
        exit(1);
    }

    (it->second)(argc, argv);

}
