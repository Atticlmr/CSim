#include "window.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace csim::viewer {
bool Window::active_=false;
Window::Window(const std::filesystem::path& shaders,int width,int height,bool hidden) {
    if (active_) throw std::runtime_error("Only one CSim Python viewer may be open; close the previous viewer first");
    if (width<320 || height<240 || width>4096 || height>4096)
        throw std::invalid_argument("Window dimensions must be 320..4096 by 240..4096");
    if (!glfwInit()) throw std::runtime_error("GLFW initialization failed: a graphical display is required");
    try {
        glfwDefaultWindowHints();
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3); glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3);
        glfwWindowHint(GLFW_OPENGL_PROFILE,GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_VISIBLE,hidden?GLFW_FALSE:GLFW_TRUE);
#ifdef __APPLE__
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT,GLFW_TRUE);
#endif
        window_=glfwCreateWindow(width,height,"CSim | Python simulation",nullptr,nullptr);
        if (!window_) throw std::runtime_error("Cannot create an OpenGL 3.3 core window");
        glfwMakeContextCurrent(window_);
        glfwSwapInterval(0); // Python owns pacing; sync never schedules physics.
        renderer_=std::make_unique<Renderer>(shaders);
        glfwSetWindowUserPointer(window_,this);
        glfwSetScrollCallback(window_,[](GLFWwindow* w,double,double y) {
            static_cast<Window*>(glfwGetWindowUserPointer(w))->wheel_+=y;
        });
        glfwSetKeyCallback(window_,[](GLFWwindow* w,int key,int,int action,int) {
            if (action!=GLFW_PRESS) return;
            auto& self=*static_cast<Window*>(glfwGetWindowUserPointer(w));
            if (key==GLFW_KEY_ESCAPE) glfwSetWindowShouldClose(w,GLFW_TRUE);
            if (key==GLFW_KEY_C) self.home_pressed_=true;
            if (key==GLFW_KEY_F) self.follow_pressed_=true;
            if (key==GLFW_KEY_T) self.trails_pressed_=true;
        });
        glfwGetCursorPos(window_,&mouse_x_,&mouse_y_);
        active_=true;
    } catch (...) {
        renderer_.reset();
        if (window_) glfwDestroyWindow(window_);
        window_=nullptr; glfwTerminate(); throw;
    }
}
Window::~Window() { if (window_) close(); }
void Window::checkThread() const {
    if (std::this_thread::get_id()!=owner_) throw std::runtime_error("Viewer operations must use its creating main thread");
}
bool Window::isRunning() const {
    checkThread();
    return window_ && !glfwWindowShouldClose(window_);
}
void Window::close() {
    checkThread();
    if (!window_) return;
    glfwMakeContextCurrent(window_); renderer_.reset();
    glfwSetWindowUserPointer(window_,nullptr);
    glfwDestroyWindow(window_); window_=nullptr;
    glfwTerminate(); active_=false;
}
void Window::home() {
    camera_.reset(centre(frame_)-math::Vector3{0,0,1.3},frame_.has_payload ?
        std::max(6.0,7+3*(frame_.drone_position_W-frame_.payload_position_W).norm()) : 8.0);
}
void Window::sync(const Frame& candidate, const model::RigidBodyAsset* asset) {
    checkThread();
    if (!window_) throw std::runtime_error("Viewer is closed");
    if (!std::isfinite(candidate.time) || candidate.time<0 || !candidate.drone_position_W.isFinite()
        || candidate.drone_position_W.norm()>10000 || !candidate.q_WB.isFinite())
        throw std::invalid_argument("State exceeds viewer position/time limits");
    const auto orientation=candidate.q_WB.normalized();
    if (candidate.has_payload && (!candidate.payload_position_W.isFinite()
        || candidate.payload_position_W.norm()>10000 || !std::isfinite(candidate.tension)
        || (!candidate.cable_slack && candidate.tension<=0)))
        throw std::invalid_argument("Invalid payload display state");
    if (first_ || candidate.time<frame_.time) trail_.clear();
    frame_=candidate; frame_.q_WB=orientation;
    if (trail_.empty() || frame_.time-trail_.back().time>=0.02-1e-12) {
        trail_.push_back(frame_); if (trail_.size()>1500) trail_.pop_front();
    }
    if (first_) { home(); first_=false; }
    glfwPollEvents();
    if (!isRunning()) return;
    glfwMakeContextCurrent(window_);
    int width=0,height=0; glfwGetFramebufferSize(window_,&width,&height);
    camera_.setViewport(width,height);
    double x=0,y=0; glfwGetCursorPos(window_,&x,&y);
    if (glfwGetMouseButton(window_,GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS) camera_.orbit(x-mouse_x_,y-mouse_y_);
    if (glfwGetMouseButton(window_,GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS) {
        following_=false; camera_.pan(x-mouse_x_,y-mouse_y_);
    }
    mouse_x_=x; mouse_y_=y; camera_.zoom(wheel_); wheel_=0;
    if (home_pressed_) { home(); following_=false; }
    if (follow_pressed_) following_=!following_;
    if (trails_pressed_) trails_=!trails_;
    home_pressed_=follow_pressed_=trails_pressed_=false;
    if (following_) camera_.setTarget(centre(frame_));
    if (width && height) {
        renderer_->render(camera_,{frame_,trail_,asset},trails_);
        glfwSwapBuffers(window_);
    }
}
void Window::screenshot(const std::filesystem::path& path) {
    checkThread();
    if (!window_ || first_) throw std::runtime_error("Screenshot requires an open viewer and a synced frame");
    glfwMakeContextCurrent(window_);
    renderer_->screenshot(path,camera_.viewportWidth(),camera_.viewportHeight());
}
} // namespace csim::viewer
