#pragma once

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#ifdef __APPLE__
#include <OpenGL/gl3.h>
#else
#ifndef GL_GLEXT_PROTOTYPES
#define GL_GLEXT_PROTOTYPES
#endif
#include <GL/glcorearb.h>
#endif

#include <stdexcept>
#include <string>

namespace csim::viewer {
// Load only the OpenGL 3.3 entry points used here, after making a context current.
// Keep pointer names distinct from OpenGL symbols; no third-party loader required.
#define CSIM_GL_FUNCTIONS(X) \
    X(Clear) X(ClearColor) X(Enable) X(Disable) X(Viewport) X(GetError) \
    X(CreateShader) X(ShaderSource) X(CompileShader) X(GetShaderiv) X(GetShaderInfoLog) X(DeleteShader) \
    X(CreateProgram) X(AttachShader) X(LinkProgram) X(GetProgramiv) X(GetProgramInfoLog) X(DeleteProgram) \
    X(UseProgram) X(GetUniformLocation) X(UniformMatrix4fv) \
    X(GenVertexArrays) X(BindVertexArray) X(DeleteVertexArrays) \
    X(GenBuffers) X(BindBuffer) X(BufferData) X(DeleteBuffers) \
    X(EnableVertexAttribArray) X(VertexAttribPointer) X(DrawArrays) \
    X(PixelStorei) X(ReadBuffer) X(ReadPixels) \
    X(GenFramebuffers) X(BindFramebuffer) X(DeleteFramebuffers) X(CheckFramebufferStatus) X(BlitFramebuffer) \
    X(GenRenderbuffers) X(BindRenderbuffer) X(RenderbufferStorage) X(DeleteRenderbuffers) X(FramebufferRenderbuffer)

struct GlApi {
#define CSIM_GL_DECLARE(name) decltype(&::gl##name) name = nullptr;
    CSIM_GL_FUNCTIONS(CSIM_GL_DECLARE)
#undef CSIM_GL_DECLARE
    GlApi() {
#define CSIM_GL_LOAD(name) \
        name = reinterpret_cast<decltype(name)>(glfwGetProcAddress("gl" #name)); \
        if (!name) throw std::runtime_error("OpenGL function unavailable: gl" #name);
        CSIM_GL_FUNCTIONS(CSIM_GL_LOAD)
#undef CSIM_GL_LOAD
    }
};
#undef CSIM_GL_FUNCTIONS
} // namespace csim::viewer
