#include <iostream>
#include <unordered_map>

void launchDemoSnowball(int argc, char const **argv);

int main(int argc, char const **argv) {

    std::unordered_map<std::string, void (*)(int argc, char const **argv)> routines;

#if USE_RENDERBOX
    routines.insert(std::make_pair("demo-snowball", launchDemoSnowball));
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
