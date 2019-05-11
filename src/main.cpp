#include <iostream>
#include <unordered_map>


void launchInfo(int argc, char const **argv);

void launchDemoSnowball(int argc, char const **argv);

void launchDemoDiffSnowball(int argc, char const **argv);

void launchDemoSlabOverWedge(int argc, char const **argv);

void launchDemoSnowman(int argc, char const **argv);

void launchDemoLavaSnowball(int argc, char const **argv);

void launchSimGenSnowball(int argc, char const **argv);

void launchSimGenSlab(int argc, char const **argv);

void launchSimGenSnowman(int argc, char const **argv);

void launchSimScene0(int argc, char const **argv);

void launchVizScene0(int argc, char const **argv);

void launchVizDiffScene0(int argc, char const **argv);

void launchSimScene1(int argc, char const **argv);

void launchVizScene1(int argc, char const **argv);

void launchRenderScene1(int argc, char const **argv);

void launchVizDiffScene1(int argc, char const **argv);

int main(int argc, char const **argv) {

    std::unordered_map<std::string, void (*)(int argc, char const **argv)> routines;

    routines.insert(std::make_pair("info", launchInfo));

    routines.insert(std::make_pair("sim-gen-snowball", launchSimGenSnowball));
    routines.insert(std::make_pair("sim-gen-slab", launchSimGenSlab));
    routines.insert(std::make_pair("sim-gen-snowman", launchSimGenSnowman));

    routines.insert(std::make_pair("sim-scene0", launchSimScene0));
    routines.insert(std::make_pair("sim-scene1", launchSimScene1));

#if USE_RENDERBOX

    // Snow solver demos
    routines.insert(std::make_pair("demo-snowball", launchDemoSnowball));
    routines.insert(std::make_pair("demo-diff-snowball", launchDemoDiffSnowball));
    routines.insert(std::make_pair("demo-slab-over-wedge", launchDemoSlabOverWedge));
    routines.insert(std::make_pair("demo-snowman", launchDemoSnowman));

    // "Lava" solver demos
    routines.insert(std::make_pair("demo-lava-snowball", launchDemoLavaSnowball));

    routines.insert(std::make_pair("viz-scene0", launchVizScene0));
    routines.insert(std::make_pair("viz-diff-scene0", launchVizDiffScene0));
    routines.insert(std::make_pair("viz-scene1", launchVizScene1));
    routines.insert(std::make_pair("render-scene1", launchRenderScene1));
    routines.insert(std::make_pair("viz-diff-scene1", launchVizDiffScene1));

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
