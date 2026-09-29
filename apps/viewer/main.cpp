#include "camera.hpp"
#include "renderer.hpp"
#include "scene.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

namespace {
struct Options {
    std::filesystem::path trajectory, screenshot;
    int frames=0;
    bool hidden=false, paused=false, help=false;
};
Options parse(int argc,char** argv) {
    Options options;
    for (int i=1; i<argc; ++i) {
        const std::string arg=argv[i];
        auto value=[&]() -> std::string {
            if (++i>=argc) throw std::invalid_argument("Missing value after "+arg);
            return argv[i];
        };
        if (arg=="--help" || arg=="-h") options.help=true;
        else if (arg=="--hidden") options.hidden=true;
        else if (arg=="--paused") options.paused=true;
        else if (arg=="--screenshot") options.screenshot=value();
        else if (arg=="--frames") {
            const auto count=value();
            const auto result=std::from_chars(count.data(),count.data()+count.size(),options.frames);
            if (result.ec!=std::errc{} || result.ptr!=count.data()+count.size() || options.frames<1 || options.frames>1000000)
                throw std::invalid_argument("--frames must be between 1 and 1000000");
        } else if (!arg.empty() && arg[0]!='-' && options.trajectory.empty()) options.trajectory=arg;
        else throw std::invalid_argument("Unexpected argument: "+arg);
    }
    if (!options.help && (options.hidden || !options.screenshot.empty()) && !options.frames)
        throw std::invalid_argument("--hidden and --screenshot require --frames for bounded verification");
    return options;
}
void glfwError(int code,const char* description) { std::cerr<<"GLFW "<<code<<": "<<description<<'\n'; }
struct GlfwRuntime {
    GlfwRuntime() {
        glfwSetErrorCallback(glfwError);
        if (!glfwInit()) throw std::runtime_error("GLFW initialization failed; a working graphical display is required");
    }
    ~GlfwRuntime() { glfwTerminate(); }
};
struct Input {
    std::array<bool,GLFW_KEY_LAST+1> pressed{};
    double wheel=0;
};
void keyCallback(GLFWwindow* window,int key,int,int action,int) {
    if (action==GLFW_PRESS && key>=0 && key<=GLFW_KEY_LAST)
        if (auto* input=static_cast<Input*>(glfwGetWindowUserPointer(window))) input->pressed[key]=true;
}
void scrollCallback(GLFWwindow* window,double,double y) {
    if (auto* input=static_cast<Input*>(glfwGetWindowUserPointer(window))) input->wheel+=y;
}
std::filesystem::path shaders(const char* program) {
#ifdef __linux__
    std::error_code error;
    const auto binary=std::filesystem::read_symlink("/proc/self/exe",error);
    if (!error) return binary.parent_path()/"shaders";
#endif
    return std::filesystem::absolute(program).parent_path()/"shaders";
}
void home(csim::viewer::Camera& camera,const csim::viewer::Frame& frame) {
    const auto centre=(frame.drone_position_W+frame.payload_position_W)*0.5;
    const double length=(frame.drone_position_W-frame.payload_position_W).norm();
    camera.reset(centre-csim::math::Vector3{0,0,1.3},std::max(6.0,7+3*length));
}
}

int main(int argc,char** argv) {
    try {
        const auto options=parse(argc,argv);
        if (options.help) {
            std::cout<<"Usage: viewer [trajectory.csv] [--paused] [--frames N] [--screenshot image.ppm] [--hidden]\n"
                     <<"Default: live coupled payload demo. CSV: replay recorded physical states.\n"
                     <<"Space: pause/resume; N/right: one step; R: restart; +/-: speed (0.125x-4x).\n"
                     <<"Left drag: orbit; right drag: pan; wheel: zoom; F: follow; C: home camera.\n"
                     <<"T: trails; Esc: exit. --frames uses deterministic 1/60 s display ticks.\n";
            return EXIT_SUCCESS;
        }
        // Parse before creating a window, so invalid recordings fail clearly even headlessly.
        csim::viewer::Scene scene(options.trajectory.empty() ? std::vector<csim::viewer::Frame>{}
                                                          : csim::viewer::loadTrajectory(options.trajectory));
        scene.setPaused(options.paused);
        GlfwRuntime runtime;
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3); glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3);
        glfwWindowHint(GLFW_OPENGL_PROFILE,GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_VISIBLE,options.hidden?GLFW_FALSE:GLFW_TRUE);
#ifdef __APPLE__
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT,GLFW_TRUE);
#endif
        std::unique_ptr<GLFWwindow,decltype(&glfwDestroyWindow)> window(
            glfwCreateWindow(1280,720,"CSim | Suspended payload",nullptr,nullptr),glfwDestroyWindow);
        if (!window) throw std::runtime_error("Cannot create an OpenGL 3.3 core window");
        glfwMakeContextCurrent(window.get()); glfwSwapInterval(options.frames?0:1);
        Input input;
        glfwSetWindowUserPointer(window.get(),&input);
        glfwSetKeyCallback(window.get(),keyCallback); glfwSetScrollCallback(window.get(),scrollCallback);
        csim::viewer::Camera camera; home(camera,scene.frame());
        csim::viewer::Renderer renderer(shaders(argv[0]));
        bool trails=true, following=false;
        std::string physics_error;
        double previous=glfwGetTime(), mouse_x=0, mouse_y=0;
        glfwGetCursorPos(window.get(),&mouse_x,&mouse_y);
        int rendered=0;
        std::cout<<"CSim viewer: "<<(scene.replay()?"trajectory replay":"live coupled simulation")
                 <<". Space pause, N step, R reset, mouse orbit/pan/zoom.\n";
        while (!glfwWindowShouldClose(window.get())) {
            glfwPollEvents();
            const double now=glfwGetTime();
            double elapsed=std::max(0.0,now-previous); previous=now;
            if (options.frames) elapsed=rendered?1.0/60:0;
            auto keys=input.pressed; input.pressed.fill(false);
            if (keys[GLFW_KEY_ESCAPE]) glfwSetWindowShouldClose(window.get(),GLFW_TRUE);
            if (glfwWindowShouldClose(window.get())) break;
            int width=0,height=0; glfwGetFramebufferSize(window.get(),&width,&height);
            camera.setViewport(width,height);
            double x=0,y=0; glfwGetCursorPos(window.get(),&x,&y);
            if (glfwGetMouseButton(window.get(),GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS) camera.orbit(x-mouse_x,y-mouse_y);
            if (glfwGetMouseButton(window.get(),GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS) {
                following=false; camera.pan(x-mouse_x,y-mouse_y);
            }
            mouse_x=x; mouse_y=y; camera.zoom(input.wheel); input.wheel=0;
            if (!width || !height) { glfwWaitEventsTimeout(0.1); previous=glfwGetTime(); continue; }
            if (keys[GLFW_KEY_T]) trails=!trails;
            if (keys[GLFW_KEY_F]) following=!following;
            if (keys[GLFW_KEY_C]) { following=false; home(camera,scene.frame()); }
            if (keys[GLFW_KEY_EQUAL] || keys[GLFW_KEY_KP_ADD]) scene.setSpeed(std::min(4.0,scene.speed()*2));
            if (keys[GLFW_KEY_MINUS] || keys[GLFW_KEY_KP_SUBTRACT]) scene.setSpeed(std::max(0.125,scene.speed()/2));
            try {
                if (keys[GLFW_KEY_R]) { scene.reset(); physics_error.clear(); elapsed=0; }
                if (keys[GLFW_KEY_SPACE] && physics_error.empty()) { scene.setPaused(!scene.paused()); elapsed=0; }
                if (keys[GLFW_KEY_N] || keys[GLFW_KEY_RIGHT]) scene.singleStep();
                if (physics_error.empty()) scene.advance(elapsed);
            } catch (const std::exception& error) {
                scene.setPaused(true); physics_error=error.what();
                std::cerr<<"Physics stopped at t="<<scene.frame().time<<": "<<physics_error<<'\n';
            }
            if (following) camera.setTarget((scene.frame().drone_position_W+scene.frame().payload_position_W)*0.5);
            renderer.render(camera,{scene.frame(),scene.trail()},trails);
            ++rendered;
            if (options.frames && rendered==options.frames && !options.screenshot.empty())
                renderer.screenshot(options.screenshot,width,height);
            glfwSwapBuffers(window.get());
            const auto title=std::string("CSim | ")+(scene.replay()?"Replay":"Live")
                +(scene.paused()?" | Paused":"")+" | t="+std::to_string(scene.frame().time)+" s";
            glfwSetWindowTitle(window.get(),title.c_str());
            if (options.frames && rendered>=options.frames) break;
        }
        glfwSetWindowUserPointer(window.get(),nullptr);
        std::cout<<"Viewer closed: frames="<<rendered<<", time="<<scene.frame().time<<" s\n";
        // renderer is destroyed while the context is still current, then window and GLFW.
        return physics_error.empty()?EXIT_SUCCESS:EXIT_FAILURE;
    } catch (const std::exception& error) {
        std::cerr<<"Viewer failed: "<<error.what()<<'\n'; return EXIT_FAILURE;
    }
}
