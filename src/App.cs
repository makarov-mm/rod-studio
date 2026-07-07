using System.Diagnostics;
using System.Numerics;
using System.Runtime.Versioning;
using RodStudio.Gfx;
using RodStudio.Platform;
using RodStudio.Sim;

namespace RodStudio;

[SupportedOSPlatform("windows")]
internal static class App
{
    private static readonly Func<Scene>[] Factories =
    [
        Scenes.Cantilever, Scenes.Buckling, Scenes.Plectoneme,
        Scenes.Drape, Scenes.CoilDrop, Scenes.Whirl,
    ];

    private static Scene _scene = null!;
    private static DerSolver _solver = null!;
    private static int _sceneIndex;
    private static double _simTime;
    private static bool _paused;
    private static bool _wireframe;
    private static bool _showHelp = true;
    private static bool _vsync = true;

    private static readonly Vector3 FogColor = new(0.09f, 0.10f, 0.13f);
    private static readonly Vector3 LightDir = Vector3.Normalize(new Vector3(0.45f, 1.0f, 0.3f));

    public static int Run()
    {
        using var window = GlWindow.Create("Rod Studio", 1440, 810);
        GL.LoadFunctions();
        Console.WriteLine($"OpenGL {GL.GetString(GL.VERSION)} on {GL.GetString(GL.RENDERER)} " +
                          $"(requested {window.ContextVersion.Major}.{window.ContextVersion.Minor} core)");
        window.SetVSync(_vsync);

        GL.Enable(GL.DEPTH_TEST);
        GL.DepthFunc(GL.LEQUAL);
        GL.Enable(GL.CULL_FACE);
        GL.CullFace(GL.BACK);
        GL.FrontFace(GL.CCW);
        GL.Enable(GL.MULTISAMPLE);

        var tube = new TubeMesh();
        tube.Init();
        var ground = new GroundRenderer();
        ground.Init();
        var hud = new TextHud();
        hud.Init();
        var camera = new Camera();

        LoadScene(0, camera);

        var clock = Stopwatch.StartNew();
        double prev = 0, accumulator = 0;
        double fpsTimer = 0; int fpsFrames = 0; double fps = 0;
        int substepsThisFrame = 0;
        bool simLagging = false;

        while (!window.CloseRequested)
        {
            window.PumpMessages();
            double now = clock.Elapsed.TotalSeconds;
            double frameDt = Math.Min(now - prev, 0.05);
            prev = now;

            HandleInput(window, camera);

            // ---- fixed-dt substepping with a hard per-frame budget ----
            substepsThisFrame = 0;
            if (!_paused)
            {
                accumulator += frameDt;
                double dt = _scene.SubstepDt;
                int maxSteps = (int)(0.030 / dt); // never spend more than ~30 ms of sim per frame
                while (accumulator >= dt && substepsThisFrame < maxSteps)
                {
                    _scene.Driver?.Invoke(_scene.Rod, _simTime, dt, _scene.Param);
                    _solver.Step(_scene.Rod, dt);
                    _simTime += dt;
                    accumulator -= dt;
                    substepsThisFrame++;
                }
                simLagging = accumulator >= dt;
                if (simLagging) accumulator = 0; // drop the backlog: slow-mo instead of spiral of death
            }

            // ---- render ----
            int w = Math.Max(window.Width, 1), h = Math.Max(window.Height, 1);
            GL.Viewport(0, 0, w, h);
            GL.ClearColor(FogColor.X, FogColor.Y, FogColor.Z, 1f);
            GL.Clear(GL.COLOR_BUFFER_BIT | GL.DEPTH_BUFFER_BIT | GL.STENCIL_BUFFER_BIT);

            var viewProj = camera.ViewProj(w / (float)h);

            ground.Draw(viewProj, FogColor);
            tube.Update(_scene.Rod);
            tube.DrawShadow(viewProj, LightDir);
            if (_wireframe) GL.PolygonMode(GL.FRONT_AND_BACK, GL.LINE);
            tube.Draw(viewProj, camera.Eye, LightDir,
                      new Vector3(0.86f, 0.42f, 0.16f),   // base: warm copper
                      new Vector3(0.94f, 0.87f, 0.70f),   // stripes: pale brass
                      FogColor);
            if (_wireframe) GL.PolygonMode(GL.FRONT_AND_BACK, GL.FILL);

            DrawHud(hud, w, h, fps, substepsThisFrame, simLagging);

            window.Swap();

            // ---- fps ----
            fpsFrames++;
            fpsTimer += frameDt;
            if (fpsTimer >= 0.5)
            {
                fps = fpsFrames / fpsTimer;
                fpsFrames = 0; fpsTimer = 0;
                window.SetTitle($"Rod Studio — {_scene.Name}  |  {fps:f0} fps  |  " +
                                $"{_scene.Rod.NodeCount} nodes  |  dt {_scene.SubstepDt * 1000:f2} ms");
            }
        }
        return 0;
    }

    // ------------------------------------------------------------------ input

    private static void HandleInput(GlWindow win, Camera cam)
    {
        if (win.MouseLeft) cam.Orbit(win.MouseDx, win.MouseDy);
        if (win.MouseRight || win.MouseMiddle) cam.Pan(win.MouseDx, win.MouseDy);
        if (win.WheelDelta != 0) cam.Zoom(win.WheelDelta);

        while (win.KeyPresses.Count > 0)
        {
            int vk = win.KeyPresses.Dequeue();
            switch (vk)
            {
                case Win32.VK_ESCAPE: Win32.PostQuitMessage(0); return;
                case Win32.VK_SPACE: _paused = !_paused; break;
                case 'R': LoadScene(_sceneIndex, cam); break;
                case 'W': _wireframe = !_wireframe; break;
                case 'H': _showHelp = !_showHelp; break;
                case 'F': cam.Target = ToNumerics(_scene.CameraTarget); cam.Distance = (float)_scene.CameraDistance; break;
                case >= '1' and <= '6': LoadScene(vk - '1', cam); break;
                case Win32.VK_LEFT: AdjustParam(-1); break;
                case Win32.VK_RIGHT: AdjustParam(+1); break;
                case Win32.VK_UP: _scene.Rod.BendStiffness *= 1.5; break;
                case Win32.VK_DOWN: _scene.Rod.BendStiffness /= 1.5; break;
                case 'T': _scene.Rod.TwistStiffness *= 1.5; break;
                case 'G': _scene.Rod.TwistStiffness /= 1.5; break;
                case 'V': _vsync = !_vsync; win.SetVSync(_vsync); break;
            }
        }
    }

    private static void AdjustParam(int dir)
    {
        if (_scene.ParamName == null) return;
        if (_scene.ParamStep > 0)
            _scene.Param = Math.Clamp(_scene.Param + dir * _scene.ParamStep, _scene.ParamMin, _scene.ParamMax);
        else // geometric parameter (stiffness-like)
            _scene.Param = Math.Clamp(_scene.Param * (dir > 0 ? 1.3 : 1 / 1.3), _scene.ParamMin, _scene.ParamMax);
    }

    private static void LoadScene(int index, Camera cam)
    {
        _sceneIndex = index;
        _scene = Factories[index]();
        _solver = new DerSolver
        {
            SelfContact = _scene.SelfContact,
            GroundPlane = _scene.GroundPlane,
        };
        _simTime = 0;
        _paused = false;
        cam.Target = ToNumerics(_scene.CameraTarget);
        cam.Distance = (float)_scene.CameraDistance;
    }

    // ------------------------------------------------------------------- hud

    private static void DrawHud(TextHud hud, int w, int h, double fps, int substeps, bool lagging)
    {
        var cText = new Vector3(0.92f, 0.93f, 0.95f);
        var cDim = new Vector3(0.62f, 0.64f, 0.70f);
        var cAccent = new Vector3(0.98f, 0.72f, 0.35f);
        var cWarn = new Vector3(0.95f, 0.35f, 0.30f);

        hud.Begin();
        float y = 14;
        hud.Print(14, y, _scene.Name, cAccent, 2.4f); y += 26;
        hud.Print(14, y, _scene.Blurb, cDim, 1.6f); y += 22;

        if (_scene.ParamName != null)
        {
            hud.Print(14, y, $"{_scene.ParamName}: {_scene.Param:g4}   [Left/Right]", cText, 1.8f);
            y += 20;
        }
        hud.Print(14, y, $"EI {_scene.Rod.BendStiffness:g3} [Up/Down]   GJ {_scene.Rod.TwistStiffness:g3} [T/G]", cText, 1.8f);
        y += 20;

        double eb = DerSolver.BendingEnergy(_scene.Rod);
        double et = DerSolver.TwistEnergy(_scene.Rod);
        hud.Print(14, y, $"E_bend {eb:e2} J   E_twist {et:e2} J   substeps/frame {substeps}", cDim, 1.6f);
        y += 20;

        if (_paused) { hud.Print(14, y, "PAUSED [Space]", cWarn, 1.8f); y += 20; }
        if (lagging) { hud.Print(14, y, "sim budget exceeded: running slow-mo", cWarn, 1.6f); y += 20; }

        if (_showHelp)
        {
            string[] help =
            [
                "1-6 scenes   Space pause   R reset   F refocus",
                "LMB orbit   RMB pan   wheel zoom",
                "W wireframe   V vsync   H hide help   Esc quit",
            ];
            float hy = h - 14 - help.Length * 18;
            foreach (string line in help)
            {
                hud.Print(14, hy, line, cDim, 1.6f);
                hy += 18;
            }
        }
        hud.Flush(w, h);
    }

    private static Vector3 ToNumerics(Sim.Vec3 v) => new((float)v.X, (float)v.Y, (float)v.Z);
}
