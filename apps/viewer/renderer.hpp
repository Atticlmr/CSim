#pragma once

#include "gl_api.hpp"
#include "camera.hpp"
#include "display.hpp"
#include <filesystem>
#include <vector>

namespace csim::viewer {

struct Vertex { float x,y,z,r,g,b,nx,ny,nz; };
class Renderer {
public:
    // Construction, use and destruction all require the same current GL context.
    explicit Renderer(const std::filesystem::path& shader_directory);
    ~Renderer();
    Renderer(const Renderer&)=delete;
    Renderer& operator=(const Renderer&)=delete;
    void render(const Camera& camera, const Display& scene, bool trails);
    void screenshot(const std::filesystem::path& path, int width, int height);
private:
    void upload(const std::vector<Vertex>& vertices, GLenum mode, const Matrix4& matrix);
    void release() noexcept;
    void resizeFramebuffer(int width, int height);
    GlApi gl_;
    GLuint program_=0, vao_=0, vbo_=0;
    GLint matrix_location_=-1;
    GLuint framebuffer_=0, color_buffer_=0, depth_buffer_=0;
    int framebuffer_width_=0, framebuffer_height_=0;
};

} // namespace csim::viewer
