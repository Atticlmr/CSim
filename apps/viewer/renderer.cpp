#include "renderer.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace csim::viewer {
namespace {
using math::Vector3;
using Mesh=std::vector<Vertex>;
const Vector3 cyan{0.18,0.83,0.86}, gold{1,0.69,0.27}, white{0.83,0.9,0.96};
constexpr double pi=3.14159265358979323846;
Vertex vertex(Vector3 p, Vector3 c, Vector3 n={}) {
    return {static_cast<float>(p.x),static_cast<float>(p.y),static_cast<float>(p.z),
        static_cast<float>(c.x),static_cast<float>(c.y),static_cast<float>(c.z),
        static_cast<float>(n.x),static_cast<float>(n.y),static_cast<float>(n.z)};
}
void triangle(Mesh& mesh, Vector3 a, Vector3 b, Vector3 c, Vector3 color, Vector3 normal={}) {
    mesh.push_back(vertex(a,color,normal)); mesh.push_back(vertex(b,color,normal)); mesh.push_back(vertex(c,color,normal));
}
void line(Mesh& mesh, Vector3 a, Vector3 b, Vector3 color) {
    mesh.push_back(vertex(a,color)); mesh.push_back(vertex(b,color));
}
void cylinder(Mesh& mesh, Vector3 a, Vector3 b, double radius, Vector3 color, int sides=16) {
    const auto axis=(b-a).normalized();
    const auto u=axis.cross(std::abs(axis.z)<0.9 ? Vector3{0,0,1} : Vector3{1,0,0}).normalized();
    const auto v=axis.cross(u);
    for (int i=0; i<sides; ++i) {
        const double t=2*pi*i/sides, next=2*pi*(i+1)/sides;
        const auto r=(u*std::cos(t)+v*std::sin(t))*radius;
        const auto s=(u*std::cos(next)+v*std::sin(next))*radius;
        const auto normal=(r+s).normalized();
        triangle(mesh,a+r,b+r,b+s,color,normal); triangle(mesh,a+r,b+s,a+s,color,normal);
        triangle(mesh,a,a+s,a+r,color,-axis); triangle(mesh,b,b+r,b+s,color,axis);
    }
}
void sphere(Mesh& mesh, Vector3 centre, double radius, Vector3 color) {
    auto direction=[](double latitude,double longitude) {
        return Vector3{std::cos(latitude)*std::cos(longitude),std::cos(latitude)*std::sin(longitude),std::sin(latitude)};
    };
    auto add=[&](Vector3 n) { mesh.push_back(vertex(centre+n*radius,color,n)); };
    for (int j=0; j<12; ++j) for (int i=0; i<24; ++i) {
        const double a=-pi/2+pi*j/12, b=-pi/2+pi*(j+1)/12;
        const double t=2*pi*i/24, s=2*pi*(i+1)/24;
        const auto n1=direction(a,t), n2=direction(a,s), n3=direction(b,s), n4=direction(b,t);
        add(n1); add(n2); add(n3); add(n1); add(n3); add(n4);
    }
}
void box(Mesh& mesh, Vector3 centre, Vector3 half, const math::Matrix3& rotation, Vector3 color) {
    const std::array<Vector3,8> p{{{-half.x,-half.y,-half.z},{half.x,-half.y,-half.z},
        {half.x,half.y,-half.z},{-half.x,half.y,-half.z},
        {-half.x,-half.y,half.z},{half.x,-half.y,half.z},{half.x,half.y,half.z},{-half.x,half.y,half.z}}};
    const int faces[6][4]={{0,3,2,1},{4,5,6,7},{0,1,5,4},{1,2,6,5},{2,3,7,6},{3,0,4,7}};
    for (const auto& face:faces) {
        const auto a=centre+rotation*p[face[0]], b=centre+rotation*p[face[1]];
        const auto c=centre+rotation*p[face[2]], d=centre+rotation*p[face[3]];
        const auto normal=(b-a).cross(c-a).normalized();
        triangle(mesh,a,b,c,color,normal); triangle(mesh,a,c,d,color,normal);
    }
}
std::string source(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Cannot open shader: "+path.string());
    std::ostringstream data; data<<input.rdbuf();
    if (input.bad()) throw std::runtime_error("Cannot read shader: "+path.string());
    return data.str();
}
}

Renderer::Renderer(const std::filesystem::path& directory) {
    GLuint vertex_shader=0, fragment_shader=0;
    auto compile=[&](GLenum type,const std::filesystem::path& path) {
        const auto code=source(path); const char* pointer=code.c_str();
        const GLuint shader=gl_.CreateShader(type);
        if (!shader) throw std::runtime_error("Cannot allocate OpenGL shader");
        gl_.ShaderSource(shader,1,&pointer,nullptr); gl_.CompileShader(shader);
        GLint ok=0; gl_.GetShaderiv(shader,GL_COMPILE_STATUS,&ok);
        if (!ok) {
            std::array<char,4096> log{}; gl_.GetShaderInfoLog(shader,static_cast<GLsizei>(log.size()),nullptr,log.data());
            gl_.DeleteShader(shader); throw std::runtime_error(path.string()+": "+log.data());
        }
        return shader;
    };
    try {
        vertex_shader=compile(GL_VERTEX_SHADER,directory/"scene.vert");
        fragment_shader=compile(GL_FRAGMENT_SHADER,directory/"scene.frag");
        program_=gl_.CreateProgram();
        gl_.AttachShader(program_,vertex_shader); gl_.AttachShader(program_,fragment_shader); gl_.LinkProgram(program_);
        GLint ok=0; gl_.GetProgramiv(program_,GL_LINK_STATUS,&ok);
        if (!ok) {
            std::array<char,4096> log{}; gl_.GetProgramInfoLog(program_,static_cast<GLsizei>(log.size()),nullptr,log.data());
            throw std::runtime_error(std::string("Shader link failed: ")+log.data());
        }
        gl_.DeleteShader(vertex_shader); vertex_shader=0; gl_.DeleteShader(fragment_shader); fragment_shader=0;
        matrix_location_=gl_.GetUniformLocation(program_,"u_mvp");
        if (matrix_location_<0) throw std::runtime_error("Shader is missing u_mvp");
        gl_.GenVertexArrays(1,&vao_); gl_.GenBuffers(1,&vbo_);
        if (!vao_ || !vbo_) throw std::runtime_error("Cannot allocate OpenGL geometry buffers");
        gl_.BindVertexArray(vao_); gl_.BindBuffer(GL_ARRAY_BUFFER,vbo_);
        const std::size_t offsets[]={offsetof(Vertex,x),offsetof(Vertex,r),offsetof(Vertex,nx)};
        for (GLuint i=0; i<3; ++i) {
            gl_.EnableVertexAttribArray(i);
            gl_.VertexAttribPointer(i,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),reinterpret_cast<const void*>(offsets[i]));
        }
        gl_.BindVertexArray(0);
    } catch (...) {
        if (vertex_shader) gl_.DeleteShader(vertex_shader);
        if (fragment_shader) gl_.DeleteShader(fragment_shader);
        release(); throw;
    }
}
Renderer::~Renderer() { release(); }
void Renderer::release() noexcept {
    if (depth_buffer_) gl_.DeleteRenderbuffers(1,&depth_buffer_);
    if (color_buffer_) gl_.DeleteRenderbuffers(1,&color_buffer_);
    if (framebuffer_) gl_.DeleteFramebuffers(1,&framebuffer_);
    if (vbo_) gl_.DeleteBuffers(1,&vbo_);
    if (vao_) gl_.DeleteVertexArrays(1,&vao_);
    if (program_) gl_.DeleteProgram(program_);
    vbo_=vao_=program_=0;
    framebuffer_=color_buffer_=depth_buffer_=0;
}
void Renderer::resizeFramebuffer(int width,int height) {
    if (framebuffer_width_==width && framebuffer_height_==height) return;
    if (!framebuffer_) gl_.GenFramebuffers(1,&framebuffer_);
    if (!color_buffer_) gl_.GenRenderbuffers(1,&color_buffer_);
    if (!depth_buffer_) gl_.GenRenderbuffers(1,&depth_buffer_);
    if (!framebuffer_ || !color_buffer_ || !depth_buffer_) throw std::runtime_error("Cannot allocate framebuffers");
    gl_.BindFramebuffer(GL_FRAMEBUFFER,framebuffer_);
    gl_.BindRenderbuffer(GL_RENDERBUFFER,color_buffer_);
    gl_.RenderbufferStorage(GL_RENDERBUFFER,GL_RGBA8,width,height);
    gl_.FramebufferRenderbuffer(GL_FRAMEBUFFER,GL_COLOR_ATTACHMENT0,GL_RENDERBUFFER,color_buffer_);
    gl_.BindRenderbuffer(GL_RENDERBUFFER,depth_buffer_);
    gl_.RenderbufferStorage(GL_RENDERBUFFER,GL_DEPTH_COMPONENT24,width,height);
    gl_.FramebufferRenderbuffer(GL_FRAMEBUFFER,GL_DEPTH_ATTACHMENT,GL_RENDERBUFFER,depth_buffer_);
    if (gl_.CheckFramebufferStatus(GL_FRAMEBUFFER)!=GL_FRAMEBUFFER_COMPLETE)
        throw std::runtime_error("Incomplete offscreen framebuffer");
    framebuffer_width_=width; framebuffer_height_=height;
}
void Renderer::upload(const std::vector<Vertex>& vertices,GLenum mode,const Matrix4& matrix) {
    if (vertices.empty()) return;
    if (vertices.size()>static_cast<std::size_t>(std::numeric_limits<GLsizei>::max()))
        throw std::overflow_error("Too many display vertices");
    const auto packed=columnMajor(matrix);
    gl_.UseProgram(program_); gl_.UniformMatrix4fv(matrix_location_,1,GL_FALSE,packed.data());
    gl_.BindVertexArray(vao_); gl_.BindBuffer(GL_ARRAY_BUFFER,vbo_);
    gl_.BufferData(GL_ARRAY_BUFFER,static_cast<GLsizeiptr>(vertices.size()*sizeof(Vertex)),vertices.data(),GL_DYNAMIC_DRAW);
    gl_.DrawArrays(mode,0,static_cast<GLsizei>(vertices.size()));
}
void Renderer::render(const Camera& camera,const Display& scene,bool trails) {
    const int width=camera.viewportWidth(), height=camera.viewportHeight();
    if (!width || !height) return;
    resizeFramebuffer(width,height);
    gl_.BindFramebuffer(GL_FRAMEBUFFER,framebuffer_);
    gl_.Viewport(0,0,width,height); gl_.Enable(GL_DEPTH_TEST);
    gl_.ClearColor(0.035F,0.052F,0.08F,1); gl_.Clear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    Mesh solids, lines;
    const auto& f=scene.frame;
    const double cx=std::floor(camera.target().x), cy=std::floor(camera.target().y);
    triangle(solids,{cx-30,cy-30,-0.02},{cx+30,cy-30,-0.02},{cx+30,cy+30,-0.02},{0.075,0.10,0.14});
    triangle(solids,{cx-30,cy-30,-0.02},{cx+30,cy+30,-0.02},{cx-30,cy+30,-0.02},{0.075,0.10,0.14});
    for (int i=-30; i<=30; ++i) {
        const Vector3 color=i%5==0 ? Vector3{0.2,0.25,0.31} : Vector3{0.12,0.16,0.21};
        line(lines,{cx+i,cy-30,0},{cx+i,cy+30,0},color);
        line(lines,{cx-30,cy+i,0},{cx+30,cy+i,0},color);
    }
    cylinder(solids,{0,0,0},{2,0,0},0.018,{0.96,0.3,0.3});
    cylinder(solids,{0,0,0},{0,2,0},0.018,{0.3,0.85,0.47});
    cylinder(solids,{0,0,0},{0,0,2},0.018,{0.35,0.6,1});
    const auto rotation=f.q_WB.toRotationMatrix();
    auto body=[&](Vector3 p) { return f.drone_position_W+rotation*p; };
    auto render_asset=[&](const model::RigidBodyAsset& asset,Vector3 origin,
                          const math::Matrix3& orientation,Vector3 default_color) {
        for (const auto& g:asset.visuals_B) {
            const auto position=origin+orientation*g.body_from_geometry.position;
            const auto r=orientation*g.body_from_geometry.orientation.toRotationMatrix();
            const Vector3 color=g.rgba ? Vector3{(*g.rgba)[0],(*g.rgba)[1],(*g.rgba)[2]} : default_color;
            std::visit([&](const auto& shape) {
                using T=std::decay_t<decltype(shape)>;
                if constexpr (std::is_same_v<T,model::Box>) box(solids,position,shape.size*0.5,r,color);
                else if constexpr (std::is_same_v<T,model::Sphere>) sphere(solids,position,shape.radius,color);
                else if constexpr (std::is_same_v<T,model::Cylinder>) {
                    const auto axis=r*Vector3{0,0,shape.length*0.5};
                    cylinder(solids,position-axis,position+axis,shape.radius,color);
                } else if constexpr (std::is_same_v<T,model::MeshReference>) {
                    auto point=[&](std::size_t i) {
                        const auto& v=shape.mesh->vertices[i];
                        return position+r*Vector3{v.x*shape.scale.x,v.y*shape.scale.y,v.z*shape.scale.z};
                    };
                    for (const auto& t:shape.mesh->triangles) {
                        const auto a=point(t[0]),b=point(t[1]),c=point(t[2]);
                        triangle(solids,a,b,c,color,(b-a).cross(c-a).normalized());
                    }
                }
            },g.shape);
        }
    };
    if (scene.asset && !scene.asset->visuals_B.empty()) {
        render_asset(*scene.asset,f.drone_position_W,rotation,cyan);
    } else {
    box(solids,body({0,0,0}),{0.27,0.18,0.07},rotation,cyan);
    box(solids,body({0.27,0,0.015}),{0.05,0.13,0.055},rotation,{1,0.34,0.3});
    for (int x:{-1,1}) for (int y:{-1,1}) {
        const Vector3 end{0.45*x,0.45*y,0.02};
        cylinder(solids,body({}),body(end),0.035,white);
        cylinder(solids,body(end+Vector3{0,0,-0.04}),body(end+Vector3{0,0,0.08}),0.07,{0.16,0.22,0.29});
        cylinder(solids,body(end+Vector3{0,0,0.085}),body(end+Vector3{0,0,0.10}),0.24,
            x>0 ? Vector3{0.85,0.48,0.43} : Vector3{0.25,0.48,0.53},24);
        // Static rotor markers are geometry only, not simulated motor speeds.
        cylinder(solids,body(end+Vector3{-0.21,0,0.105}),body(end+Vector3{0.21,0,0.105}),0.012,white,8);
    }
    }
    if (f.has_payload) {
        const auto start=f.drone_attachment_W.value_or(f.drone_position_W);
        const auto end=f.payload_attachment_W.value_or(f.payload_position_W);
        if ((end-start).norm()>1e-12) {
            if (f.cable_slack) {
                for (int segment=0;segment<20;segment+=2)
                    line(lines,start+(end-start)*(segment/20.),start+(end-start)*((segment+1)/20.),gold*.55);
            } else cylinder(solids,start,end,0.009,gold,10);
        }
        if (f.rigid_payload) {
            const auto orientation=f.q_WP.toRotationMatrix();
            if (scene.payload_asset && !scene.payload_asset->visuals_B.empty())
                render_asset(*scene.payload_asset,f.payload_position_W,orientation,gold);
            else box(solids,f.payload_position_W,{.1,.075,.06},orientation,gold);
            for (const auto& axis:{Vector3{.25,0,0},Vector3{0,.25,0},Vector3{0,0,.25}})
                line(lines,f.payload_position_W,f.payload_position_W+orientation*axis,
                    axis.x>0 ? Vector3{1,.3,.3} : axis.y>0 ? Vector3{.3,1,.3} : Vector3{.3,.5,1});
        } else sphere(solids,f.payload_position_W,0.13,gold);
        if (f.drone_attachment_W) sphere(solids,start,.022,white);
        if (f.payload_attachment_W) sphere(solids,end,.022,white);
    }
    sphere(solids,f.drone_position_W,0.045,white);
    if (!scene.asset) for (int y:{-1,1}) {
        cylinder(solids,body({-0.15,0.15*y,-0.05}),body({-0.15,0.23*y,-0.25}),0.017,white,8);
        cylinder(solids,body({0.15,0.15*y,-0.05}),body({0.15,0.23*y,-0.25}),0.017,white,8);
        cylinder(solids,body({-0.3,0.23*y,-0.25}),body({0.3,0.23*y,-0.25}),0.02,white,8);
    }
    if (trails && !scene.trail.empty()) {
        Frame previous=scene.trail.front();
        std::size_t index=0;
        for (const auto& sample:scene.trail) {
            const double fade=0.2+0.6*static_cast<double>(++index)/scene.trail.size();
            line(lines,previous.drone_position_W,sample.drone_position_W,cyan*fade);
            if (previous.has_payload && sample.has_payload)
                line(lines,previous.payload_position_W,sample.payload_position_W,gold*fade);
            previous=sample;
        }
        line(lines,previous.drone_position_W,f.drone_position_W,cyan);
        if (f.has_payload && previous.has_payload) line(lines,previous.payload_position_W,f.payload_position_W,gold);
    }
    upload(solids,GL_TRIANGLES,camera.viewProjection()); upload(lines,GL_LINES,camera.viewProjection());

    gl_.BindVertexArray(0); gl_.UseProgram(0);
    gl_.BindFramebuffer(GL_READ_FRAMEBUFFER,framebuffer_);
    gl_.BindFramebuffer(GL_DRAW_FRAMEBUFFER,0);
    gl_.BlitFramebuffer(0,0,width,height,0,0,width,height,GL_COLOR_BUFFER_BIT,GL_NEAREST);
    gl_.BindFramebuffer(GL_FRAMEBUFFER,0);
    const GLenum error_code=gl_.GetError();
    if (error_code!=GL_NO_ERROR) throw std::runtime_error("OpenGL rendering error: "+std::to_string(error_code));
}
void Renderer::screenshot(const std::filesystem::path& path,int width,int height) {
    if (width<=0 || height<=0 || width>16384 || height>16384) throw std::invalid_argument("Invalid screenshot dimensions");
    std::vector<unsigned char> pixels(static_cast<std::size_t>(width)*height*3);
    if (width!=framebuffer_width_ || height!=framebuffer_height_)
        throw std::invalid_argument("Screenshot size must match the rendered frame");
    gl_.BindFramebuffer(GL_READ_FRAMEBUFFER,framebuffer_);
    gl_.PixelStorei(GL_PACK_ALIGNMENT,1); gl_.ReadBuffer(GL_COLOR_ATTACHMENT0);
    gl_.ReadPixels(0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,pixels.data());
    gl_.BindFramebuffer(GL_READ_FRAMEBUFFER,0);
    if (gl_.GetError()!=GL_NO_ERROR) throw std::runtime_error("OpenGL screenshot readback failed");
    std::ofstream file(path,std::ios::binary);
    if (!file) throw std::runtime_error("Cannot write screenshot: "+path.string());
    file<<"P6\n"<<width<<' '<<height<<"\n255\n";
    for (int y=height-1; y>=0; --y)
        file.write(reinterpret_cast<const char*>(pixels.data()+static_cast<std::size_t>(y)*width*3),width*3);
    file.close();
    if (!file) throw std::runtime_error("Screenshot write failed");
}
} // namespace csim::viewer
