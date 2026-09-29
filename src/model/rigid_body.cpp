#include <csim/model/rigid_body.hpp>
#include <csim/dynamics/drone.hpp>
#include <functional>
#include <map>
#include <stdexcept>

namespace csim::model {
Pose compose(const Pose& p,const Pose& c) {
    return {p.position+p.orientation.rotate(c.position),(p.orientation*c.orientation).normalized()};
}
namespace {
void require(bool condition,const char* message) { if (!condition) throw std::invalid_argument(message); }
void pose(const Pose& p) {
    require(p.position.isFinite() && p.position.norm()<=10000,"Model pose outside finite 10000 m limit");
    (void)p.orientation.normalized();
}
void inertia(const InertialDescription& i) {
    pose(i.body_from_inertial);
    (void)dynamics::Drone(i.mass,i.inertia); // Positive mass, symmetric positive definite tensor.
    // Physical principal-moment triangle inequalities: trace(J)/2 I - J is PSD.
    // Jacobi rotations here only validate a 3x3 parameter tensor, not a public eigensolver.
    auto j=i.inertia;
    double scale=0;
    for (std::size_t a=0;a<3;++a) for (std::size_t b=0;b<3;++b) scale=std::max(scale,std::abs(j(a,b)));
    for (std::size_t a=0;a<3;++a) for (std::size_t b=0;b<3;++b) j(a,b)/=scale;
    for (int iteration=0;iteration<32;++iteration) {
        std::size_t p=0,q=1;
        for (std::size_t a=0;a<3;++a) for (std::size_t b=a+1;b<3;++b)
            if (std::abs(j(a,b))>std::abs(j(p,q))) { p=a; q=b; }
        if (std::abs(j(p,q))<1e-14) break;
        const double angle=0.5*std::atan2(2*j(p,q),j(q,q)-j(p,p));
        auto r=math::Matrix3::identity();
        r(p,p)=r(q,q)=std::cos(angle); r(p,q)=std::sin(angle); r(q,p)=-std::sin(angle);
        j=r.transposed()*j*r;
    }
    const double trace=j(0,0)+j(1,1)+j(2,2);
    for (std::size_t a=0;a<3;++a) require(2*j(a,a)<=trace+1e-12,"Inertia violates principal-moment triangle inequality");
}
std::vector<Pose> transforms(const ModelDescription& d) {
    std::map<std::string,std::size_t> names;
    for (std::size_t i=0;i<d.bodies.size();++i) names.emplace(d.bodies[i].name,i);
    std::vector<Pose> poses(d.bodies.size());
    std::vector<int> marks(d.bodies.size());
    std::function<void(std::size_t)> visit=[&](std::size_t i) {
        require(marks[i]!=1,"Cycle in body tree");
        if (marks[i]==2) return;
        marks[i]=1;
        if (d.bodies[i].parent) {
            const auto parent=names.find(*d.bodies[i].parent);
            require(parent!=names.end(),"Unknown parent body"); visit(parent->second);
            poses[i]=compose(poses[parent->second],d.bodies[i].parent_from_body);
        } else poses[i]={}; // T_RR, root's source placement handled separately.
        marks[i]=2;
    };
    for (std::size_t i=0;i<d.bodies.size();++i) visit(i);
    return poses;
}
void geometry(const GeometryInstance& g) {
    pose(g.body_from_geometry);
    if (g.rgba) for (double c:*g.rgba) require(std::isfinite(c)&&c>=0&&c<=1,"Invalid RGBA");
    std::visit([](const auto& s) {
        using T=std::decay_t<decltype(s)>;
        if constexpr (std::is_same_v<T,std::monostate>) require(false,"Missing geometry");
        else if constexpr (std::is_same_v<T,Box>)
            require(s.size.isFinite()&&s.size.x>0&&s.size.y>0&&s.size.z>0&&s.size.norm()<=1000,"Invalid box size");
        else if constexpr (std::is_same_v<T,Sphere>) require(std::isfinite(s.radius)&&s.radius>0&&s.radius<=1000,"Invalid sphere radius");
        else if constexpr (std::is_same_v<T,Cylinder>) require(std::isfinite(s.radius)&&s.radius>0&&s.radius<=1000
            &&std::isfinite(s.length)&&s.length>0&&s.length<=1000,"Invalid cylinder dimensions");
        else {
            require(s.mesh && !s.mesh->triangles.empty(),"Missing decoded mesh");
            require(s.scale.isFinite()&&s.scale.x>0&&s.scale.y>0&&s.scale.z>0,"Mesh scale must be finite positive");
            for (const auto& v:s.mesh->vertices) require(v.isFinite()&&math::Vector3{v.x*s.scale.x,v.y*s.scale.y,v.z*s.scale.z}.norm()<=1000,"Mesh dimensions exceed limit");
            for (const auto& t:s.mesh->triangles) for (auto index:t) require(index<s.mesh->vertices.size(),"Mesh index out of bounds");
        }
    },g.shape);
}
}
void validateDescription(const ModelDescription& d) {
    require(!d.bodies.empty()&&d.bodies.size()<=256,"Model requires 1..256 bodies");
    std::map<std::string,bool> names;
    std::size_t roots=0, triangles=0, geometries=0;
    for (const auto& b:d.bodies) {
        require(!b.name.empty()&&names.emplace(b.name,true).second,"Empty or duplicate body name");
        if (!b.parent) ++roots;
        pose(b.parent_from_body);
        if (b.inertial) inertia(*b.inertial);
        for (const auto& g:b.visuals) {
            geometry(g);
            require(++geometries<=2048,"Too many visual geometries");
            if (auto* mesh=std::get_if<MeshReference>(&g.shape)) triangles+=mesh->mesh->triangles.size();
            require(triangles<=200000,"Instanced visual meshes exceed 200000 triangles");
        }
        for (const auto& g:b.collisions) geometry(g);
    }
    require(roots==1,"Model requires exactly one root body");
    (void)transforms(d);
}
RigidBodyAsset buildRigidBody(const ModelDescription& d,bool free_base,math::Quaternion q_BR) {
    validateDescription(d);
    require(d.root_motion==RootMotion::free || free_base,"Model root is not free; explicitly pass free_base=True to create a drone");
    q_BR=q_BR.normalized();
    const auto poses=transforms(d);
    RigidBodyAsset result;
    std::vector<Pose> inertials;
    Pose source_root;
    for (std::size_t n=0;n<d.bodies.size();++n) {
        const auto& b=d.bodies[n];
        require(b.inertial.has_value(),"Every physical body requires explicit mass and inertia");
        if (!b.parent) source_root=b.parent_from_body;
        inertials.push_back(compose(poses[n],b.inertial->body_from_inertial));
        result.mass+=b.inertial->mass;
        result.center_R+=inertials.back().position*b.inertial->mass;
    }
    result.center_R/=result.mass;
    require(std::isfinite(result.mass)&&result.center_R.isFinite(),"Aggregate mass/center overflow");
    math::Matrix3 J;
    for (std::size_t n=0;n<d.bodies.size();++n) {
        const auto& body=d.bodies[n]; const auto& i=*body.inertial;
        const Pose body_from_link{
            q_BR.rotate(poses[n].position-result.center_R),
            (q_BR*poses[n].orientation).normalized()};
        result.links.push_back({body.name,body.parent,body_from_link,i.body_from_inertial.position});
        const auto r=inertials[n].orientation.toRotationMatrix();
        const auto delta=inertials[n].position-result.center_R;
        math::Matrix3 shift;
        const double x[3]={delta.x,delta.y,delta.z};
        for (std::size_t a=0;a<3;++a) for (std::size_t b=0;b<3;++b)
            shift(a,b)=i.mass*((a==b?delta.squaredNorm():0)-x[a]*x[b]);
        J+=r*i.inertia*r.transposed()+shift;
        for (auto visual:body.visuals) {
            visual.body_from_geometry=compose(poses[n],visual.body_from_geometry);
            visual.body_from_geometry.position=q_BR.rotate(visual.body_from_geometry.position-result.center_R);
            visual.body_from_geometry.orientation=(q_BR*visual.body_from_geometry.orientation).normalized();
            result.visuals_B.push_back(std::move(visual));
        }
    }
    const auto r=q_BR.toRotationMatrix(); result.inertia_B=r*J*r.transposed();
    inertia({result.mass,{},result.inertia_B,{}});
    result.initial_pose_WB={source_root.position+source_root.orientation.rotate(result.center_R),
        (source_root.orientation*q_BR.conjugated()).normalized()};
    return result;
}
} // namespace csim::model
