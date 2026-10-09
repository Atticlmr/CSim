#include <csim/io/urdf.hpp>
#include <csim/io/mjcf.hpp>
#include <csim/io/mesh.hpp>
#include <csim/model/rigid_body.hpp>
#include <tinyxml2.h>
#include <fstream>
#include <cctype>
#include <functional>
#include <locale>
#include <map>
#include <set>
#include <sstream>

namespace csim::io {
namespace {
using E=tinyxml2::XMLElement;
using model::Pose;
using math::Vector3;
using math::Quaternion;
struct Failure : std::runtime_error {
    ImportDiagnostic diagnostic;
    explicit Failure(ImportDiagnostic d):std::runtime_error(d.message),diagnostic(std::move(d)) {}
};
struct Parser {
    std::filesystem::path file;
    ImportOptions options;
    tinyxml2::XMLDocument xml;
    std::map<std::string,std::shared_ptr<const model::TriangleMesh>> meshes;
    std::size_t triangles=0;
    model::SourceLocation source(const E* e) const {
        return {file,e?static_cast<std::size_t>(e->GetLineNum()):0,
            e?std::string(e->Name())+(e->Attribute("name")?std::string("[@name='")+e->Attribute("name")+"']":""):""};
    }
    [[noreturn]] void fail(const E* e,std::string message,ImportErrorCode code=ImportErrorCode::invalid_value) const {
        throw Failure({DiagnosticSeverity::error,code,source(e),std::move(message)});
    }
    const E* load() {
        std::error_code error;
        const auto size=std::filesystem::file_size(file,error);
        if (error || size>4*1024*1024) fail(nullptr,"Cannot read model, or XML exceeds 4 MiB",ImportErrorCode::file_error);
        if (xml.LoadFile(file.string().c_str())!=tinyxml2::XML_SUCCESS) {
            auto s=source(nullptr); s.line=static_cast<std::size_t>(std::max(0,xml.ErrorLineNum()));
            throw Failure({DiagnosticSeverity::error,ImportErrorCode::xml_error,s,xml.ErrorStr()});
        }
        if (!xml.RootElement() || xml.RootElement()->NextSiblingElement()) fail(nullptr,"XML requires exactly one root",ImportErrorCode::xml_error);
        std::size_t nodes=0;
        std::function<void(const tinyxml2::XMLNode*,int)> scan=[&](const tinyxml2::XMLNode* node,int depth){
            if (depth>128 || ++nodes>10000) fail(node->ToElement(),"XML nesting/node limit exceeded");
            if (node->ToUnknown()) fail(node->Parent()?node->Parent()->ToElement():nullptr,"DTD/unknown XML declarations are unsupported",ImportErrorCode::unsupported_feature);
            if (const auto* text=node->ToText()) {
                std::string value=text->Value();
                if (value.find_first_not_of(" \t\r\n")!=std::string::npos) fail(node->Parent()->ToElement(),"Unexpected XML text");
            }
            for (auto* child=node->FirstChild();child;child=child->NextSibling()) scan(child,depth+1);
        };
        scan(&xml,0);
        return xml.RootElement();
    }
    void attributes(const E* e,std::initializer_list<const char*> allowed) const {
        for (auto* a=e->FirstAttribute();a;a=a->Next()) {
            bool found=false; for (auto* name:allowed) if (std::string(a->Name())==name) found=true;
            if (!found) fail(e,"Unsupported attribute: "+std::string(a->Name()),ImportErrorCode::unsupported_feature);
        }
    }
    void children(const E* e,std::initializer_list<const char*> allowed) const {
        for (auto* c=e->FirstChildElement();c;c=c->NextSiblingElement()) {
            bool found=false; for (auto* name:allowed) if (std::string(c->Name())==name) found=true;
            if (!found) fail(c,"Unsupported element: "+std::string(c->Name()),ImportErrorCode::unsupported_feature);
        }
    }
    const E* one(const E* e,const char* name,bool required=false) const {
        auto* c=e->FirstChildElement(name);
        if (!c && required) fail(e,"Missing element: "+std::string(name));
        if (c && c->NextSiblingElement(name)) fail(c,"Duplicate element: "+std::string(name));
        return c;
    }
    std::string attr(const E* e,const char* name,const char* fallback=nullptr) const {
        auto* value=e->Attribute(name);
        if (!value) { if (fallback) return fallback; fail(e,"Missing attribute: "+std::string(name)); }
        if (!*value) fail(e,"Empty attribute: "+std::string(name));
        return value;
    }
    std::vector<double> numbers(const E* e,const char* name,std::size_t count,const char* fallback=nullptr) const {
        std::istringstream input(attr(e,name,fallback)); input.imbue(std::locale::classic());
        std::vector<double> values; double value;
        while (input>>value) { if (!std::isfinite(value)) fail(e,"Non-finite "+std::string(name)); values.push_back(value); }
        if (!input.eof() || values.size()!=count) fail(e,"Expected "+std::to_string(count)+" finite numbers for "+name);
        return values;
    }
    double number(const E* e,const char* name,const char* fallback=nullptr) const { return numbers(e,name,1,fallback)[0]; }
    Vector3 triple(const E* e,const char* name,const char* fallback=nullptr) const {
        const auto v=numbers(e,name,3,fallback); return {v[0],v[1],v[2]};
    }
    std::array<double,4> color(const E* e,const char* key="rgba") const {
        const auto a=numbers(e,key,4);
        for (double c:a) if (c<0 || c>1) fail(e,"RGBA must be in [0,1]");
        if (a[3]!=1) fail(e,"Transparent materials are not supported by this renderer",ImportErrorCode::unsupported_feature);
        return {a[0],a[1],a[2],a[3]};
    }
    Pose origin(const E* parent) const {
        auto* e=one(parent,"origin"); if (!e) return {};
        attributes(e,{"xyz","rpy"}); children(e,{});
        const auto r=triple(e,"rpy","0 0 0");
        return {triple(e,"xyz","0 0 0"),Quaternion::fromRollPitchYaw(r.x,r.y,r.z)};
    }
    std::filesystem::path resource(const E* e,const std::string& uri,const std::filesystem::path& meshdir={}) const {
        std::filesystem::path result;
        if (uri.rfind("package://",0)==0) {
            const auto rest=uri.substr(10); const auto split=rest.find('/');
            if (split==std::string::npos) fail(e,"package URI requires a file path",ImportErrorCode::resource_error);
            const auto found=options.package_roots.find(rest.substr(0,split));
            if (found==options.package_roots.end()) fail(e,"Unmapped package: "+rest.substr(0,split),ImportErrorCode::resource_error);
            auto base=found->second; if (base.is_relative()) base=file.parent_path()/base;
            const auto relative=std::filesystem::path(rest.substr(split+1));
            if (relative.is_absolute()) fail(e,"Invalid package resource path",ImportErrorCode::resource_error);
            for (const auto& part:relative) if (part=="..") fail(e,"package resource cannot escape its package",ImportErrorCode::resource_error);
            result=base/relative;
        } else {
            if (uri.find("://")!=std::string::npos) fail(e,"Only local/package mesh resources are supported",ImportErrorCode::resource_error);
            result=uri;
            if (result.is_relative()) result=file.parent_path()/meshdir/result;
        }
        return std::filesystem::absolute(result).lexically_normal();
    }
    model::MeshReference mesh(const E* e,const std::string& uri,Vector3 scale,const std::filesystem::path& meshdir={}) {
        if (!scale.isFinite()||scale.x<=0||scale.y<=0||scale.z<=0) fail(e,"Mesh scale must be finite positive");
        const auto path=resource(e,uri,meshdir); const auto key=path.string();
        if (!meshes.count(key)) {
            try {
                auto data=std::make_shared<model::TriangleMesh>(loadMesh(path));
                triangles+=data->triangles.size();
                if (triangles>200000) fail(e,"Model exceeds total mesh triangle budget");
                meshes[key]=std::move(data);
            } catch (const Failure&) { throw; }
              catch (const std::exception& ex) { fail(e,path.string()+": "+ex.what(),ImportErrorCode::resource_error); }
        }
        return {uri,scale,source(e),meshes.at(key)};
    }
};
model::InertialDescription urdfInertia(Parser& p,const E* e) {
    p.attributes(e,{}); p.children(e,{"origin","mass","inertia"});
    auto* mass=p.one(e,"mass",true); p.attributes(mass,{"value"}); p.children(mass,{});
    auto* inertia=p.one(e,"inertia",true); p.attributes(inertia,{"ixx","ixy","ixz","iyy","iyz","izz"}); p.children(inertia,{});
    const auto a=p.number(inertia,"ixx"),b=p.number(inertia,"ixy"),c=p.number(inertia,"ixz"),
        d=p.number(inertia,"iyy"),f=p.number(inertia,"iyz"),g=p.number(inertia,"izz");
    return {p.number(mass,"value"),p.origin(e),math::Matrix3{a,b,c,b,d,f,c,f,g},p.source(e)};
}
model::ModelDescription urdf(Parser& p,const E* root) {
    if (std::string(root->Name())!="robot") p.fail(root,"Expected URDF robot root");
    p.attributes(root,{"name"}); p.children(root,{"link","joint","material"});
    model::ModelDescription d; d.name=p.attr(root,"name"); d.source=p.source(root);
    std::map<std::string,std::array<double,4>> colors;
    auto material=[&](const E* e,bool definition) {
        p.attributes(e,{"name"}); p.children(e,{"color"});
        const auto name=p.attr(e,"name");
        const auto* color=p.one(e,"color",definition);
        if (color) { p.attributes(color,{"rgba"}); p.children(color,{}); return p.color(color); }
        if (!colors.count(name)) p.fail(e,"Unknown material: "+name);
        return colors.at(name);
    };
    for (auto* e=root->FirstChildElement("material");e;e=e->NextSiblingElement("material")) {
        if (!colors.emplace(p.attr(e,"name"),material(e,true)).second) p.fail(e,"Duplicate material");
    }
    std::map<std::string,std::size_t> body_indices;
    for (auto* e=root->FirstChildElement("link");e;e=e->NextSiblingElement("link")) {
        p.attributes(e,{"name"}); p.children(e,{"inertial","visual","collision"});
        model::BodyDescription b; b.name=p.attr(e,"name"); b.source=p.source(e);
        if (auto* i=p.one(e,"inertial")) b.inertial=urdfInertia(p,i);
        for (const char* kind:{"visual","collision"}) for (auto* v=e->FirstChildElement(kind);v;v=v->NextSiblingElement(kind)) {
            p.attributes(v,{"name"}); p.children(v,std::string(kind)=="visual" ?
                std::initializer_list<const char*>{"origin","geometry","material"} : std::initializer_list<const char*>{"origin","geometry"});
            model::GeometryInstance g; g.name=p.attr(v,"name",""); g.source=p.source(v); g.body_from_geometry=p.origin(v);
            auto* geometry=p.one(v,"geometry",true); p.attributes(geometry,{}); p.children(geometry,{"box","sphere","cylinder","mesh"});
            auto* shape=geometry->FirstChildElement();
            if (!shape || shape->NextSiblingElement()) p.fail(geometry,"Geometry requires exactly one shape");
            const std::string type=shape->Name(); p.children(shape,{});
            if (type=="box") { p.attributes(shape,{"size"}); g.shape=model::Box{p.triple(shape,"size")}; }
            else if (type=="sphere") { p.attributes(shape,{"radius"}); g.shape=model::Sphere{p.number(shape,"radius")}; }
            else if (type=="cylinder") { p.attributes(shape,{"radius","length"}); g.shape=model::Cylinder{p.number(shape,"radius"),p.number(shape,"length")}; }
            else { p.attributes(shape,{"filename","scale"}); g.shape=p.mesh(shape,p.attr(shape,"filename"),p.triple(shape,"scale","1 1 1")); }
            if (auto* m=p.one(v,"material")) g.rgba=material(m,false);
            (std::string(kind)=="visual"?b.visuals:b.collisions).push_back(std::move(g));
        }
        if (!body_indices.emplace(b.name,d.bodies.size()).second) p.fail(e,"Duplicate link name",ImportErrorCode::invalid_topology);
        d.bodies.push_back(std::move(b));
    }
    std::set<std::string> joints;
    for (auto* e=root->FirstChildElement("joint");e;e=e->NextSiblingElement("joint")) {
        p.attributes(e,{"name","type"}); p.children(e,{"parent","child","origin"});
        if (!joints.insert(p.attr(e,"name")).second) p.fail(e,"Duplicate joint name");
        if (p.attr(e,"type")!="fixed") p.fail(e,"Only fixed URDF joints are supported",ImportErrorCode::unsupported_feature);
        auto* parent=p.one(e,"parent",true); auto* child=p.one(e,"child",true);
        p.attributes(parent,{"link"}); p.attributes(child,{"link"}); p.children(parent,{}); p.children(child,{});
        const auto pn=p.attr(parent,"link"),cn=p.attr(child,"link");
        if (!body_indices.count(pn)||!body_indices.count(cn)) p.fail(e,"Unknown joint link",ImportErrorCode::invalid_topology);
        auto& b=d.bodies[body_indices.at(cn)];
        if (b.parent) p.fail(e,"Link has multiple parents",ImportErrorCode::invalid_topology);
        b.parent=pn; b.parent_from_body=p.origin(e);
    }
    return d;
}
Pose mjPose(Parser& p,const E* e,double angle,const std::string& sequence) {
    Pose result; result.position=p.triple(e,"pos","0 0 0");
    if (e->Attribute("quat") && e->Attribute("euler")) p.fail(e,"Multiple orientation specifications");
    if (e->Attribute("quat")) {
        const auto q=p.numbers(e,"quat",4); result.orientation=Quaternion{q[0],q[1],q[2],q[3]}.normalized();
    } else if (e->Attribute("euler")) {
        const auto angles=p.numbers(e,"euler",3);
        for (std::size_t i=0;i<3;++i) {
            char axis=sequence[i]; const char lower=static_cast<char>(std::tolower(static_cast<unsigned char>(axis)));
            Vector3 vector{lower=='x'?1.0:0,lower=='y'?1.0:0,lower=='z'?1.0:0};
            const auto q=Quaternion::fromAxisAngle(vector,angles[i]*angle);
            result.orientation=std::islower(static_cast<unsigned char>(axis))?result.orientation*q:q*result.orientation;
        }
        result.orientation=result.orientation.normalized();
    }
    return result;
}
model::ModelDescription mjcf(Parser& p,const E* root) {
    if (std::string(root->Name())!="mujoco") p.fail(root,"Expected MJCF mujoco root");
    p.attributes(root,{"model"}); p.children(root,{"compiler","asset","worldbody"});
    model::ModelDescription d; d.name=p.attr(root,"model","model"); d.source=p.source(root); d.format=model::SourceFormat::mjcf; d.root_motion=model::RootMotion::fixed;
    double angle=std::acos(-1.0)/180; std::string sequence="xyz"; std::filesystem::path meshdir;
    const auto* compiler=p.one(root,"compiler",true);
    p.attributes(compiler,{"angle","eulerseq","inertiafromgeom","meshdir","coordinate"}); p.children(compiler,{});
    if (p.attr(compiler,"inertiafromgeom")!="false") p.fail(compiler,"Explicit inertiafromgeom=false is required",ImportErrorCode::unsupported_feature);
    const auto unit=p.attr(compiler,"angle","degree");
    if (unit=="radian") angle=1; else if (unit!="degree") p.fail(compiler,"Invalid angle unit");
    if (p.attr(compiler,"coordinate","local")!="local") p.fail(compiler,"Only local MJCF coordinates are supported");
    sequence=p.attr(compiler,"eulerseq","xyz");
    if (sequence.size()!=3 || sequence.find_first_not_of("xyzXYZ")!=std::string::npos) p.fail(compiler,"Invalid eulerseq");
    meshdir=p.attr(compiler,"meshdir","");
    std::map<std::string,model::MeshReference> meshes;
    if (auto* asset=p.one(root,"asset")) {
        p.attributes(asset,{}); p.children(asset,{"mesh"});
        for (auto* e=asset->FirstChildElement();e;e=e->NextSiblingElement()) {
            p.attributes(e,{"name","file","scale"}); p.children(e,{});
            auto mesh=p.mesh(e,p.attr(e,"file"),p.triple(e,"scale","1 1 1"),meshdir);
            if (!meshes.emplace(p.attr(e,"name"),std::move(mesh)).second) p.fail(e,"Duplicate mesh name");
        }
    }
    auto* world=p.one(root,"worldbody",true); p.attributes(world,{}); p.children(world,{"body"});
    auto* base=p.one(world,"body",true);
    std::set<std::string> names,geom_names;
    std::function<void(const E*,std::optional<std::string>)> body=[&](const E* e,std::optional<std::string> parent) {
        p.attributes(e,{"name","pos","quat","euler"}); p.children(e,{"inertial","geom","body","freejoint","site"});
        model::BodyDescription b; b.name=p.attr(e,"name"); b.parent=parent; b.parent_from_body=mjPose(p,e,angle,sequence); b.source=p.source(e);
        if (!names.insert(b.name).second) p.fail(e,"Duplicate body name");
        if (auto* free=p.one(e,"freejoint")) {
            p.attributes(free,{"name"}); p.children(free,{});
            if (parent) p.fail(free,"Only root freejoint is supported",ImportErrorCode::unsupported_feature);
            d.root_motion=model::RootMotion::free;
        }
        if (auto* i=p.one(e,"inertial")) {
            p.attributes(i,{"mass","pos","quat","euler","diaginertia","fullinertia"}); p.children(i,{});
            model::InertialDescription inertia; inertia.mass=p.number(i,"mass");
            (void)p.attr(i,"pos"); inertia.body_from_inertial=mjPose(p,i,angle,sequence); inertia.source=p.source(i);
            if (i->Attribute("diaginertia") && i->Attribute("fullinertia")) p.fail(i,"Specify one inertia representation");
            if (i->Attribute("fullinertia")) {
                if (i->Attribute("quat") || i->Attribute("euler")) p.fail(i,"fullinertia is expressed in body axes, not an explicit inertial orientation");
                const auto j=p.numbers(i,"fullinertia",6); inertia.inertia=math::Matrix3{j[0],j[3],j[4],j[3],j[1],j[5],j[4],j[5],j[2]};
            } else {
                const auto j=p.numbers(i,"diaginertia",3); inertia.inertia=math::Matrix3{j[0],0,0,0,j[1],0,0,0,j[2]};
            }
            b.inertial=inertia;
        } else p.fail(e,"Explicit inertial element required; geometry-derived inertia is unsupported",ImportErrorCode::unsupported_feature);
        for (auto* g=e->FirstChildElement("geom");g;g=g->NextSiblingElement("geom")) {
            p.attributes(g,{"name","type","size","pos","quat","euler","rgba","contype","conaffinity","mesh"}); p.children(g,{});
            if (p.number(g,"contype","1")!=0 || p.number(g,"conaffinity","1")!=0)
                p.fail(g,"This subset requires contact-disabled geoms: contype=0 conaffinity=0",ImportErrorCode::unsupported_feature);
            model::GeometryInstance visual; visual.name=p.attr(g,"name",""); visual.source=p.source(g); visual.body_from_geometry=mjPose(p,g,angle,sequence);
            if (!visual.name.empty() && !geom_names.insert(visual.name).second) p.fail(g,"Duplicate geom name");
            if (g->Attribute("rgba")) visual.rgba=p.color(g);
            const auto type=p.attr(g,"type",g->Attribute("mesh")?"mesh":"sphere");
            if (type=="box") visual.shape=model::Box{p.triple(g,"size")*2};
            else if (type=="sphere") visual.shape=model::Sphere{p.number(g,"size")};
            else if (type=="cylinder") { const auto size=p.numbers(g,"size",2); visual.shape=model::Cylinder{size[0],2*size[1]}; }
            else if (type=="mesh") {
                if (g->Attribute("size")) p.fail(g,"Mesh size/fitting is unsupported; use asset scale");
                const auto name=p.attr(g,"mesh"); if (!meshes.count(name)) p.fail(g,"Unknown mesh asset"); visual.shape=meshes.at(name);
            } else p.fail(g,"Unsupported geom type: "+type,ImportErrorCode::unsupported_feature);
            if (type!="mesh" && g->Attribute("mesh")) p.fail(g,"Primitive fitting to mesh is unsupported",ImportErrorCode::unsupported_feature);
            b.visuals.push_back(std::move(visual));
        }
        for (auto* site=e->FirstChildElement("site");site;site=site->NextSiblingElement("site")) {
            p.attributes(site,{"name","pos","quat","euler","size","rgba","type","group"});
            p.children(site,{});
            if (site->Attribute("size")) {
                std::istringstream values(p.attr(site,"size"));
                double value; std::size_t count=0;
                while (values>>value) {
                    if (!std::isfinite(value)||value<=0) p.fail(site,"Invalid site size");
                    ++count;
                }
                if (!values.eof() || (count!=1 && count!=3)) p.fail(site,"Site size requires 1 or 3 values");
            }
            if (site->Attribute("rgba")) (void)p.color(site);
            const auto type=p.attr(site,"type","sphere");
            if (type!="sphere" && type!="box" && type!="ellipsoid" && type!="capsule" && type!="cylinder")
                p.fail(site,"Unsupported site type");
            if (site->Attribute("group")) {
                const auto group=p.number(site,"group");
                if (group<0 || group>5 || std::floor(group)!=group) p.fail(site,"Invalid site group");
            }
            d.attachments.push_back({p.attr(site,"name"),b.name,mjPose(p,site,angle,sequence),p.source(site)});
        }
        const auto name=b.name; d.bodies.push_back(std::move(b));
        if (d.bodies.size()>256) p.fail(e,"Too many bodies");
        for (auto* child=e->FirstChildElement("body");child;child=child->NextSiblingElement("body")) body(child,name);
    };
    body(base,std::nullopt); return d;
}
ImportResult load(const std::filesystem::path& path,const ImportOptions& options,bool is_urdf) {
    Parser parser; parser.file=path;
    try {
        parser.file=std::filesystem::absolute(path).lexically_normal(); parser.options=options;
        const auto* root=parser.load();
        auto d=is_urdf?urdf(parser,root):mjcf(parser,root);
        try { model::validateDescription(d); }
        catch (const std::exception& e) { parser.fail(root,e.what()); }
        return {std::move(d),{}};
    } catch (const Failure& f) { return {std::nullopt,{f.diagnostic}}; }
      catch (const std::exception& e) { return {std::nullopt,{{DiagnosticSeverity::error,ImportErrorCode::invalid_value,parser.source(nullptr),e.what()}}}; }
}
}
ImportResult loadUrdf(const std::filesystem::path& path,const ImportOptions& options) { return load(path,options,true); }
ImportResult loadMjcf(const std::filesystem::path& path,const ImportOptions& options) { return load(path,options,false); }
} // namespace csim::io
