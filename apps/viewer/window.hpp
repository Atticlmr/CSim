#pragma once
#include "renderer.hpp"
#include <memory>
#include <thread>

namespace csim::viewer {
// Passive window: consumes display samples, never calls a physics step.
// Public Python wrapper retains it for main-thread atexit cleanup if forgotten.
class Window {
public:
    Window(const std::filesystem::path& shaders, int width, int height, bool hidden);
    ~Window();
    Window(const Window&)=delete;
    Window& operator=(const Window&)=delete;
    void sync(const Frame& frame, const model::RigidBodyAsset* asset=nullptr,
              const model::RigidBodyAsset* payload_asset=nullptr);
    bool isRunning() const;
    void close();
    void screenshot(const std::filesystem::path& path);
private:
    void checkThread() const;
    void home();
    GLFWwindow* window_=nullptr;
    std::unique_ptr<Renderer> renderer_;
    Camera camera_;
    Frame frame_;
    std::deque<Frame> trail_;
    std::thread::id owner_=std::this_thread::get_id();
    bool first_=true, trails_=true, following_=true;
    bool home_pressed_=false, follow_pressed_=false, trails_pressed_=false;
    double wheel_=0, mouse_x_=0, mouse_y_=0;
    static bool active_;
};
} // namespace csim::viewer
