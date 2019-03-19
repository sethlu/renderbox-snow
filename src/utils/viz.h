#ifndef SNOW_VIZ_H
#define SNOW_VIZ_H

#include <sstream>
#include <sys/stat.h>

#include "../../lib/SnowSolver.h"

#include "renderer.h"


static unsigned int startFrame;
static unsigned int endFrame;

static std::string dir;

#ifdef VIZ_RENDER
static std::string renderOutputDir;
#endif //VIZ_RENDER


static void initViz(int argc, char const **argv) {

    startFrame = static_cast<unsigned int>(atoi(argv[3]));
    endFrame = static_cast<unsigned int>(atoi(argv[4]));

    // Simulation

    dir = argv[2];

#ifdef VIZ_RENDER
    renderOutputDir = dir + ".sequence";
#endif //VIZ_RENDER

    std::ostringstream filename;
    filename << "frame-" << startFrame << ".snowstate";

    solver.reset(new SnowSolver(joinPath(dir, filename.str())));

    // Rendering

    initRenderer();

}

static void vizRenderLoopUpdate(unsigned int frame) {

    unsigned int wrappedFrame = startFrame + frame % (endFrame - startFrame);
    std::ostringstream filename;
    filename << "frame-" << wrappedFrame << ".snowstate";
    solver->loadState(joinPath(dir, filename.str()));

}

#ifdef VIZ_RENDER

static bool vizRenderLoopCallback(unsigned int frame) {

    unsigned int wrappedFrame = startFrame + frame % (endFrame - startFrame);

    unsigned char pixels[renderTarget->getFramebufferWidth() * renderTarget->getFramebufferHeight() * 3];
    glReadPixels(0, 0, renderTarget->getFramebufferWidth(), renderTarget->getFramebufferHeight(), GL_RGB,
                 GL_UNSIGNED_BYTE, pixels);
    auto texture = std::make_shared<renderbox::Texture>(static_cast<unsigned char const *>(pixels), sizeof(pixels),
                                                        renderTarget->getFramebufferWidth(),
                                                        renderTarget->getFramebufferHeight(),
                                                        renderbox::TEXTURE_PIXEL_FORMAT_RGB_UNSIGNED_BYTE,
                                                        renderbox::TEXTURE_COORDINATES_UR);

    std::ostringstream filename;
    filename << "frame-" << wrappedFrame << ".bmp";
    renderbox::Texture::convert(texture, renderbox::TEXTURE_PIXEL_FORMAT_BGR_UNSIGNED_BYTE,
                                renderbox::TEXTURE_COORDINATES_UR)->saveBMPFile(
            joinPath(renderOutputDir, filename.str()));

    return frame + 1 < endFrame - startFrame;

}

#endif //VIZ_RENDER

static void startVizLoop() {

#ifndef VIZ_RENDER
    startRenderLoop(vizRenderLoopUpdate);
#else
    mkdir(renderOutputDir.c_str(), ALLPERMS);
    startRenderLoop(vizRenderLoopUpdate, vizRenderLoopCallback);
#endif //VIZ_RENDER


}


#endif //SNOW_VIZ_H
