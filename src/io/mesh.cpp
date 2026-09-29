#include <csim/io/mesh.hpp>
#include <charconv>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace csim::io {
namespace {
using math::Vector3;
using model::TriangleMesh;
constexpr std::size_t limit=200000;
void check(bool ok,const std::string& message) { if (!ok) throw std::invalid_argument(message); }
void triangle(TriangleMesh& mesh,std::array<std::size_t,3> t) {
    check(mesh.triangles.size()<limit,"Mesh exceeds 200000 triangles");
    const auto a=mesh.vertices[t[0]],b=mesh.vertices[t[1]],c=mesh.vertices[t[2]];
    check((b-a).cross(c-a).norm()>1e-16,"Degenerate mesh triangle");
    mesh.triangles.push_back(t);
}
Vector3 vector(std::istream& input) {
    Vector3 v;
    check(static_cast<bool>(input>>v.x>>v.y>>v.z)&&v.isFinite(),"Invalid mesh vector");
    return v;
}
std::size_t index(const std::string& word,std::size_t count) {
    long long value=0;
    const auto end=word.find('/');
    const auto length=end==std::string::npos?word.size():end;
    const auto result=std::from_chars(word.data(),word.data()+length,value);
    check(result.ec==std::errc{} && result.ptr==word.data()+length && value!=0,"Invalid OBJ vertex index");
    if (value<0) value=static_cast<long long>(count)+value+1;
    check(value>0&&static_cast<unsigned long long>(value)<=count,"OBJ index out of range");
    return static_cast<std::size_t>(value-1);
}
TriangleMesh obj(const std::string& text) {
    TriangleMesh mesh;
    std::istringstream input(text);
    std::string line;
    std::size_t number=0, normals=0, textures=0;
    while (std::getline(input,line)) {
        ++number;
        try {
            line.resize(line.find('#')==std::string::npos?line.size():line.find('#'));
            std::istringstream row(line); row.imbue(std::locale::classic());
            std::string type; if (!(row>>type)) continue;
            if (type=="v") {
                check(mesh.vertices.size()<limit,"Too many OBJ vertices");
                mesh.vertices.push_back(vector(row));
                std::string extra; check(!(row>>extra),"OBJ homogeneous/color vertices unsupported");
            } else if (type=="f") {
                std::vector<std::size_t> face; std::string word;
                while (row>>word) { check(face.size()<64,"OBJ face too large"); const auto slash=word.find('/');
                    if (slash!=std::string::npos) {
                        const auto second=word.find('/',slash+1);
                        if (second==std::string::npos) (void)index(word.substr(slash+1),textures);
                        else {
                            check(word.find('/',second+1)==std::string::npos,"Malformed OBJ face index");
                            if (second>slash+1) (void)index(word.substr(slash+1,second-slash-1),textures);
                            (void)index(word.substr(second+1),normals);
                        }
                    }
                    face.push_back(index(word,mesh.vertices.size())); }
                check(face.size()>=3,"OBJ face requires at least 3 vertices");
                const auto origin=mesh.vertices[face[0]];
                const auto normal=(mesh.vertices[face[1]]-origin).cross(mesh.vertices[face[2]]-origin).normalized();
                for (std::size_t i=0;i<face.size();++i) {
                    const auto a=mesh.vertices[face[i]],b=mesh.vertices[face[(i+1)%face.size()]],c=mesh.vertices[face[(i+2)%face.size()]];
                    check(std::abs((a-origin).dot(normal))<1e-9*std::max(1.0,(a-origin).norm()),"Non-planar OBJ polygon");
                    check((b-a).cross(c-b).dot(normal)>=0,"Concave OBJ polygons require triangulation before import");
                }
                for (std::size_t i=1;i+1<face.size();++i) triangle(mesh,{face[0],face[i],face[i+1]});
            } else if (type=="vn") { (void)vector(row); std::string extra; check(!(row>>extra),"Invalid OBJ normal"); ++normals; }
            else if (type=="vt") {
                double v; int n=0;
                while (row>>v) { check(std::isfinite(v),"Invalid texture coordinate"); ++n; }
                check(row.eof() && n>=1 && n<=3,"Invalid OBJ texture coordinates"); ++textures;
            } else if (type!="o" && type!="g" && type!="s")
                check(false,"Unsupported OBJ directive: "+type+" (textured materials are not supported)");
        } catch (const std::exception& e) { throw std::invalid_argument("OBJ line "+std::to_string(number)+": "+e.what()); }
    }
    check(!mesh.triangles.empty(),"Empty OBJ mesh"); return mesh;
}
std::uint32_t u32(const char* data) {
    const auto* b=reinterpret_cast<const unsigned char*>(data);
    return b[0] | (std::uint32_t(b[1])<<8) | (std::uint32_t(b[2])<<16) | (std::uint32_t(b[3])<<24);
}
double f32(const char* data) {
    auto bits=u32(data); float value;
    static_assert(sizeof(value)==sizeof(bits)); std::memcpy(&value,&bits,sizeof(value));
    check(std::isfinite(value),"Non-finite binary STL coordinate"); return value;
}
TriangleMesh stl(const std::string& bytes) {
    TriangleMesh mesh;
    if (bytes.size()>=84 && u32(bytes.data()+80)<=limit && bytes.size()==84+50ULL*u32(bytes.data()+80)) {
        const auto count=u32(bytes.data()+80);
        for (std::size_t i=0;i<count;++i) {
            const char* p=bytes.data()+84+50*i+12;
            const auto base=mesh.vertices.size();
            for (int j=0;j<3;++j,p+=12) mesh.vertices.push_back({f32(p),f32(p+4),f32(p+8)});
            triangle(mesh,{base,base+1,base+2});
        }
    } else {
        std::istringstream input(bytes); input.imbue(std::locale::classic());
        std::string token,line;
        auto expect=[&](const char* expected){ check(static_cast<bool>(input>>token)&&token==expected,std::string("Expected STL ")+expected); };
        expect("solid"); std::getline(input,line);
        bool ended=false;
        while (input>>token) {
            if (token=="endsolid") { std::getline(input,line); ended=true; break; }
            check(token=="facet","Expected STL facet"); expect("normal"); (void)vector(input);
            expect("outer"); expect("loop"); const auto base=mesh.vertices.size();
            for (int i=0;i<3;++i) { expect("vertex"); mesh.vertices.push_back(vector(input)); }
            expect("endloop"); expect("endfacet"); triangle(mesh,{base,base+1,base+2});
        }
        check(ended && !(input>>token),"Incomplete or trailing ASCII STL data");
    }
    check(!mesh.triangles.empty(),"Empty STL mesh"); return mesh;
}
}
model::TriangleMesh loadMesh(const std::filesystem::path& path) {
    std::ifstream input(path,std::ios::binary);
    if (!input) throw std::invalid_argument("Cannot open mesh: "+path.string());
    std::string bytes;
    char block[8192];
    while (input.read(block,sizeof(block)) || input.gcount()) {
        if (bytes.size()+static_cast<std::size_t>(input.gcount())>32*1024*1024) throw std::invalid_argument("Mesh exceeds 32 MiB");
        bytes.append(block,static_cast<std::size_t>(input.gcount()));
    }
    check(!input.bad(),"Mesh read failed");
    auto extension=path.extension().string();
    for (auto& c:extension) c=static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (extension==".obj") return obj(bytes);
    if (extension==".stl") return stl(bytes);
    throw std::invalid_argument("Unsupported mesh format (use untextured OBJ or STL): "+extension);
}
} // namespace csim::io
