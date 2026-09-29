local function configure_python(target)
        import("core.base.json")
        import("core.project.config")
        import("lib.detect.find_tool")
        assert(target:is_plat("linux", "macosx") and target:plat() == os.host()
               and target:arch() == os.arch(),
               "Python bindings currently require a native Linux or macOS build")
        local python = assert(find_tool(config.get("python_executable"), {force = true}),
                              "Selected Python interpreter not found")
        local info = json.decode(os.iorunv(python.program, {
            path.join(os.projectdir(), "bindings/python/build_info.py")}))
        target:add("sysincludedirs", info.include_dirs)
        -- Use the same interpreter for headers, ABI suffix, tests and scripts.
        target:set("extension", info.extension)
        target:data_set("csim.python", python.program)
        if target:is_plat("macosx") then
            target:add("shflags", "-undefined dynamic_lookup", {force = true})
        end
end

target("csim_python")
    add_rules("python.module", {soabi = false})
    set_basename("csim")
    set_symbols("debug", "hidden")
    add_files("module.cpp", "model_loading.cpp", "link_binding.cpp", "environment_binding.cpp")
    add_deps("csim_simulation", "csim_io")

    if is_plat("linux") then add_rpathdirs("$ORIGIN/csim_viewer/.libs") end
    on_config(configure_python)
    after_build(function (target)
        os.cp(path.join(os.scriptdir(), "csim_experiments"), target:targetdir())
        os.cp(path.join(os.scriptdir(), "csim_control"), target:targetdir())
    end)

    -- xmake run csim_python script.py [args], or no args for an interactive shell.
    on_run(function (target)
        import("core.base.option")
        -- Refresh pure Python sources even when no native object needed rebuilding.
        os.cp(path.join(os.scriptdir(), "csim_experiments"), target:targetdir())
        os.cp(path.join(os.scriptdir(), "csim_control"), target:targetdir())
        os.addenv("PYTHONPATH", path.absolute(target:targetdir()))
        os.execv(target:data("csim.python"), option.get("arguments") or {})
    end)

    add_tests("bindings")
    on_test(function (target, opt)
        os.cp(path.join(os.scriptdir(), "csim_experiments"), target:targetdir())
        os.cp(path.join(os.scriptdir(), "csim_control"), target:targetdir())
        os.addenv("PYTHONPATH", path.absolute(target:targetdir()))
        local code, errors = os.execv(target:data("csim.python"), {
            "-m", "unittest", "discover", "-s",
            path.join(os.projectdir(), "tests/python"), "-v"
        }, {try = true})
        if code ~= 0 then
            opt.errors = errors or "Python simulation binding tests failed"
        end
        return code == 0
    end)
target_end()

if has_config("viewer") then
    target("csim_python_viewer")
        add_rules("python.module", {soabi = false})
        set_basename("_csim_viewer")
        set_symbols("debug", "hidden")
        add_files("viewer_module.cpp", "../../apps/viewer/window.cpp",
                  "../../apps/viewer/renderer.cpp", "../../apps/viewer/camera.cpp")
        add_includedirs("../../apps/viewer")
        add_deps("csim_simulation")
        add_packages("glfw")
        if is_plat("linux") then
            add_packages("opengl")
            add_rpathdirs("$ORIGIN/csim_viewer/.libs")
        elseif is_plat("macosx") then
            add_frameworks("OpenGL")
        end
        on_config(configure_python)
        after_build(function (target)
            os.cp(path.join(os.scriptdir(), "csim_viewer"), target:targetdir())
        end)
    target_end()
end
