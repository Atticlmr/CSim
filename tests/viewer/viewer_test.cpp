#include "camera.hpp"
#include "scene.hpp"

#include <cstdlib>
#include <iostream>
#include <limits>
#include <sstream>

namespace {
using namespace csim::viewer;
using csim::math::Vector3;
void check(bool value,const char* message) { if (!value) throw std::runtime_error(message); }
void near(double a,double b,double tol=1e-11) { check(std::isfinite(a) && std::abs(a-b)<=tol,"numeric mismatch"); }
void near(Vector3 a,Vector3 b,double tol=1e-11) { near(a.x,b.x,tol); near(a.y,b.y,tol); near(a.z,b.z,tol); }
template<class E,class F> void throws(F&& f) {
    try { f(); } catch (const E&) { return; }
    throw std::runtime_error("expected exception missing");
}
std::vector<Frame> recording(const std::string& rows) {
    std::istringstream input(std::string(trajectoryHeader)+"\n"+rows);
    return readTrajectory(input);
}
csim::math::Matrix<4,1> point(Vector3 p) { return csim::math::Matrix<4,1>{p.x,p.y,p.z,1}; }
void cameraGeometry() {
    Camera c;
    const auto eye=c.view()*point(c.position());
    near(eye(0,0),0); near(eye(1,0),0); near(eye(2,0),0); near(eye(3,0),1);
    const auto centre=c.view()*point(c.target());
    near(centre(0,0),0); near(centre(1,0),0); check(centre(2,0)<0,"camera must look down -Z");
    const auto v=c.view();
    Vector3 right{v(0,0),v(0,1),v(0,2)}, up{v(1,0),v(1,1),v(1,2)}, back{v(2,0),v(2,1),v(2,2)};
    near(right.cross(up),back); near(up.norm(),1);
    for (double depth:{0.05,2000.0}) {
        const auto clip=c.projection()*point({0,0,-depth});
        near(clip(2,0)/clip(3,0),depth==0.05?-1:1);
    }
    const auto packed=columnMajor(v);
    for (std::size_t r=0; r<4; ++r) for (std::size_t col=0; col<4; ++col)
        near(packed[col*4+r],v(r,col),1e-6);
    const auto old=c.position(); c.orbit(50,20); check((old-c.position()).norm()>0.1,"orbit did not move");
    const double distance=(c.position()-c.target()).norm(); c.zoom(3);
    check((c.position()-c.target()).norm()<distance,"zoom direction incorrect");
    const auto target=c.target(); c.pan(30,-40); check((target-c.target()).norm()>0.01,"pan did not move");
    c.orbit(1e6,1e6); check(c.view().isFinite(),"orbit pole singularity");
    c.setViewport(0,-1); near(c.aspectRatio(),1); check(c.projection().isFinite(),"minimized projection");
    c.setViewport(1000,500); near(c.aspectRatio(),2);
    throws<std::invalid_argument>([&]{ c.reset({},0); });
}
void csvContract() {
    const auto frames=recording("0,0,0,5,2,0,0,0,0,0,4,2\r\n0.1,1,0,5,-1,0,0,0,1,0,4,3\r\n");
    check(frames.size()==2,"CSV sample count"); near(frames[0].q_WB.norm(),1);
    near(frames[1].drone_position_W,{1,0,5}); near(frames[1].q_WB.w,-1);
    for (const auto& row:{"", "0,0,0,5,1,0,0,0,0,0,4,0\n", "0,0,0,5,0,0,0,0,0,0,4,2\n",
        "0,0,0,5,1,0,0,0,0,0,4,nan\n", "0,0,0,5,1,0,0,0,0,0,4,2,\n",
        "0,0,0,5,1,0,0,0,0,0,4,2,3\n", "0,0,0,5,1,0,0,0,0,0,5,2\n",
        "0,0,0,5,1,0,0,0,0,0,4,2x\n", "-1,0,0,5,1,0,0,0,0,0,4,2\n"})
        throws<std::invalid_argument>([&]{ recording(row); });
    throws<std::invalid_argument>([]{ recording("0,0,0,5,1,0,0,0,0,0,4,2\n0,0,0,5,1,0,0,0,0,0,4,2\n"); });
    throws<std::invalid_argument>([]{ recording("0,0,0,5,1,0,0,0,0,0,4,2\n1,0,0,5,1,0,0,0,0,0,3,2\n"); });
    std::istringstream bad("time,x,y\n"); throws<std::invalid_argument>([&]{ readTrajectory(bad); });
}
void playbackControls() {
    Scene scene(recording("3,0,0,5,1,0,0,0,0,0,4,2\n3.1,1,0,5,1,0,0,0,1,0,4,3\n3.3,2,0,5,1,0,0,0,2,0,4,4\n"));
    scene.advance(0.05); near(scene.frame().time,3); // Hold the recorded state, do not stretch the cable by interpolation.
    scene.advance(0.05); near(scene.frame().time,3.1);
    scene.setPaused(true); scene.advance(0.2); near(scene.frame().time,3.1);
    scene.singleStep(); near(scene.frame().time,3.3); check(scene.ended(),"recording must end");
    scene.setPaused(false); scene.advance(0.2); near(scene.frame().time,3.3);
    scene.reset(); near(scene.frame().time,3); check(scene.trail().size()==1,"reset trail");
    scene.setSpeed(2); scene.advance(0.05); near(scene.frame().time,3.1);
    throws<std::invalid_argument>([&]{ scene.setSpeed(0); });
    throws<std::invalid_argument>([&]{ scene.advance(-1); });
    throws<std::invalid_argument>([&]{ scene.advance(std::numeric_limits<double>::quiet_NaN()); });
    Scene one(recording("7,0,0,5,1,0,0,0,0,0,4,2\n")); one.singleStep(); near(one.frame().time,7);
}
void physicsIndependentOfRendering() {
    Scene fast,slow,direct;
    for (int i=0; i<200; ++i) fast.advance(0.01);
    for (int i=0; i<80; ++i) slow.advance(0.025);
    for (int i=0; i<1000; ++i) direct.singleStep();
    near(fast.frame().time,2); near(slow.frame().time,2);
    near(fast.frame().drone_position_W,direct.frame().drone_position_W,0);
    near(slow.frame().payload_position_W,direct.frame().payload_position_W,0);
    near(slow.frame().tension,fast.frame().tension,0);
    check(slow.trail().size()==fast.trail().size(),"trail sampling depends on FPS");
    fast.setPaused(true); fast.advance(0.2); near(fast.frame().time,2);
    fast.singleStep(); near(fast.frame().time,2.002);
    Scene limited; limited.setSpeed(4); limited.advance(10);
    check(limited.limited(),"wall clock stall must be reported");
    near(limited.frame().time,128*0.002);
    const double t=limited.frame().time; limited.setPaused(true); limited.setPaused(false);
    limited.advance(0); near(limited.frame().time,t,0);
}
}
int main() {
    struct Test { const char* name; void (*run)(); };
    const Test tests[]{{"camera basis, clipping and GPU matrix layout",cameraGeometry},
        {"CSV geometry and input validation",csvContract},{"recorded playback controls",playbackControls},
        {"physics and history independent of display cadence",physicsIndependentOfRendering}};
    int failed=0;
    for (const auto& test:tests) {
        try { test.run(); std::cout<<"PASS: "<<test.name<<'\n'; }
        catch (const std::exception& e) { ++failed; std::cerr<<"FAIL: "<<test.name<<": "<<e.what()<<'\n'; }
    }
    return failed?EXIT_FAILURE:EXIT_SUCCESS;
}
