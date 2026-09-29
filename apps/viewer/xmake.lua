-- Resolve installed libraries through pkg-config; do not download dependencies.
add_requires("pkgconfig::glfw3 >=3.3", {alias = "glfw", system = true})
if is_plat("linux") then
    add_requires("pkgconfig::gl", {alias = "opengl", system = true})
end

target("viewer")
    set_kind("binary")
    add_files("main.cpp", "camera.cpp", "renderer.cpp", "scene.cpp", "trajectory.cpp")
    add_headerfiles("*.hpp")
    add_deps("csim_simulation")
    add_packages("glfw")

    if is_plat("linux") then
        add_packages("opengl")
    elseif is_plat("macosx") then
        add_frameworks("OpenGL")
    elseif is_plat("windows", "mingw") then
        add_syslinks("opengl32")
    end

    after_build(function (target)
        local shaderdir = path.join(target:targetdir(), "shaders")
        os.mkdir(shaderdir)
        os.cp(path.join(os.projectdir(), "bindings/python/csim_viewer/shaders", "*"), shaderdir)
    end)
target_end()
