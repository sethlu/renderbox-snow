#include <iostream>
#include <map>


void launchInfo(int argc, char const **argv);

void launchDemoSnowball(int argc, char const **argv);

void launchDemoDiffSnowball(int argc, char const **argv);

void launchDemoSlabOverWedge(int argc, char const **argv);

void launchDemoSnowman(int argc, char const **argv);

void launchSimGenSnowball(int argc, char const **argv);

void launchSimGenSlab(int argc, char const **argv);

void launchSimGenSnowman(int argc, char const **argv);

void launchSimScene0(int argc, char const **argv);

void launchSimScene1(int argc, char const **argv);

void launchVizScene0(int argc, char const **argv);

void launchVizDiffScene0(int argc, char const **argv);

void launchVizScene1(int argc, char const **argv);

void launchVizDiffScene1(int argc, char const **argv);

void launchRenderScene1(int argc, char const **argv);

void lavaLaunchDemoSnowball(int argc, char const **argv);

void lavaLaunchSimGenSnowball(int argc, char const **argv);

void lavaLaunchSimScene0(int argc, char const **argv);

void lavaLaunchVizScene0(int argc, char const **argv);

int main(int argc, char const **argv) {

    std::map<std::string, void (*)(int argc, char const **argv)> routines;

    routines.insert(std::make_pair("info", launchInfo));

    // Snow solver starting frame generation
    routines.insert(std::make_pair("sim-gen-snowball", launchSimGenSnowball));
    routines.insert(std::make_pair("sim-gen-slab", launchSimGenSlab));
    routines.insert(std::make_pair("sim-gen-snowman", launchSimGenSnowman));

    // Snow solver simulate
    routines.insert(std::make_pair("sim-scene0", launchSimScene0));
    routines.insert(std::make_pair("sim-scene1", launchSimScene1));

    // "Lava" solver starting frame generation
    routines.insert(std::make_pair("lava:sim-gen-snowball", lavaLaunchSimGenSnowball));

    // "Lava" solver simulate
    routines.insert(std::make_pair("lava:sim-scene0", lavaLaunchSimScene0));

#if USE_RENDERBOX

    // Snow solver demos
    routines.insert(std::make_pair("demo-snowball", launchDemoSnowball));
    routines.insert(std::make_pair("demo-diff-snowball", launchDemoDiffSnowball));
    routines.insert(std::make_pair("demo-slab-over-wedge", launchDemoSlabOverWedge));
    routines.insert(std::make_pair("demo-snowman", launchDemoSnowman));

    // "Lava" solver demos
    routines.insert(std::make_pair("lava:demo-snowball", lavaLaunchDemoSnowball));

    // Snow solver visualizations
    routines.insert(std::make_pair("viz-scene0", launchVizScene0));
    routines.insert(std::make_pair("viz-diff-scene0", launchVizDiffScene0));
    routines.insert(std::make_pair("viz-scene1", launchVizScene1));
    routines.insert(std::make_pair("render-scene1", launchRenderScene1));
    routines.insert(std::make_pair("viz-diff-scene1", launchVizDiffScene1));

    // "Lava" solver visualizations
    routines.insert(std::make_pair("lava:viz-scene0", lavaLaunchVizScene0));

#endif //USE_RENDERBOX

    if (argc < 2) {
        std::cout << "Usage: ./snow [launcher]" << std::endl;

        std::cout << "Available launchers:" << std::endl;
        for (auto const &it : routines) {
            std::cout << "  * " << it.first << std::endl;
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
