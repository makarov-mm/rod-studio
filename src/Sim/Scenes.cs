namespace RodStudio.Sim;

/// <summary>A runnable scenario: a rod, solver flags, and a time-dependent boundary driver.</summary>
public sealed class Scene
{
    public required string Name;
    public required string Blurb;
    public required Rod Rod;
    public bool SelfContact;
    public bool GroundPlane = true;
    public double SubstepDt = 2e-4;
    /// <summary>Meaning of the Left/Right parameter for this scene, or null.</summary>
    public string? ParamName;
    public double Param;
    public double ParamMin, ParamMax, ParamStep;
    /// <summary>Called once per substep with (rod, simTime, dt, param).</summary>
    public Action<Rod, double, double, double>? Driver;
    public double CameraDistance = 2.2;
    public Vec3 CameraTarget = new(0, 0.4, 0);
}

public static class Scenes
{
    public static Scene[] All() =>
    [
        Cantilever(), Buckling(), Plectoneme(), Drape(), CoilDrop(), Whirl(),
    ];

    private static Vec3[] Line(int n, Vec3 a, Vec3 b)
    {
        var pts = new Vec3[n];
        for (int i = 0; i < n; i++) pts[i] = a + (b - a) * (i / (double)(n - 1));
        return pts;
    }

    // 1 ------------------------------------------------------------ cantilever

    public static Scene Cantilever()
    {
        var rod = Rod.Create(Line(60, new Vec3(-0.5, 0.9, 0), new Vec3(0.7, 0.9, 0)));
        rod.MassPerNode = 0.004;
        rod.BendStiffness = 6e-3;
        rod.TwistStiffness = 4e-3;
        rod.Damping = 0.8;
        rod.Radius = 0.014;
        rod.InitRestState();
        rod.ClampStart();
        return new Scene
        {
            Name = "1. Cantilever",
            Blurb = "Clamped beam under gravity. Left/Right scales EI; watch the sag respond.",
            Rod = rod,
            ParamName = "bend stiffness EI",
            Param = rod.BendStiffness, ParamMin = 5e-4, ParamMax = 5e-2, ParamStep = 0,
            Driver = (r, t, dt, p) => r.BendStiffness = p,
            CameraTarget = new Vec3(0.1, 0.6, 0),
        };
    }

    // 2 --------------------------------------------------------------- buckling

    public static Scene Buckling()
    {
        int n = 80;
        var rod = Rod.Create(Line(n, new Vec3(-0.6, 0.55, 0), new Vec3(0.6, 0.55, 0)));
        rod.MassPerNode = 0.002;
        rod.BendStiffness = 2.5e-3;
        rod.TwistStiffness = 2e-3;
        rod.Damping = 1.2;
        rod.Radius = 0.012;
        rod.InitRestState();
        // tiny out-of-plane seed so the bifurcation direction is not degenerate
        for (int i = 0; i < n; i++)
        {
            double s = i / (double)(n - 1);
            rod.X[i] += new Vec3(0, 0.002 * Math.Sin(Math.PI * s), 0.001 * Math.Sin(Math.PI * s));
        }
        rod.ClampStart();
        rod.ClampEnd();
        var x0 = (Vec3[])rod.X.Clone();
        double shift = 0; // accumulated compression, so changing the rate mid-run stays continuous
        return new Scene
        {
            Name = "2. Euler buckling",
            Blurb = "Right clamp driven inward. Left/Right sets compression speed. Watch the arch pop.",
            Rod = rod,
            ParamName = "compression speed",
            Param = 0.05, ParamMin = 0, ParamMax = 0.25, ParamStep = 0.01,
            Driver = (r, t, dt, p) =>
            {
                int nn = r.NodeCount;
                shift = Math.Min(shift + p * dt, 0.45); // cap at 37% of length
                r.X[nn - 1] = x0[nn - 1] - new Vec3(shift, 0, 0);
                r.X[nn - 2] = x0[nn - 2] - new Vec3(shift, 0, 0);
            },
            CameraTarget = new Vec3(0, 0.55, 0),
        };
    }

    // 3 ------------------------------------------------------------- plectoneme

    public static Rod BuildPlectoneme()
    {
        int n = 100;
        // 6% slack: rest length exceeds clamp separation, so twist converts to writhe
        var rod = Rod.Create(Line(n, new Vec3(-0.53, 0.5, 0), new Vec3(0.53, 0.5, 0)));
        rod.MassPerNode = 0.0015;
        rod.BendStiffness = 1.2e-3;
        rod.TwistStiffness = 1.6e-3;
        rod.Damping = 2.0;
        rod.Radius = 0.010;
        rod.InitRestState();
        // create slack by moving the clamps inward after rest lengths are recorded
        for (int i = 0; i < n; i++)
        {
            double s = i / (double)(n - 1);
            rod.X[i] = new Vec3(-0.5 + s * 1.0,
                                0.5 + 0.06 * Math.Sin(Math.PI * s),   // sag from the slack
                                0.004 * Math.Sin(2 * Math.PI * s));   // symmetry-breaking seed
        }
        rod.ClampStart();
        rod.ClampEnd();
        // The repositioned centerline is ~5% shorter than rest: project to a feasible
        // state at setup, otherwise the first substep's velocity correction injects a
        // violent length-restoring impulse.
        new DerSolver { ProjectionIterations = 30 }.FastProjection(rod);
        Array.Clear(rod.V);
        return rod;
    }

    public static Scene Plectoneme()
    {
        var rod = BuildPlectoneme();
        return new Scene
        {
            Name = "3. Plectonemes",
            Blurb = "Left clamp cranked about its axis. Twist -> writhe: DNA-style supercoiling. Left/Right = crank rate.",
            Rod = rod,
            SelfContact = true,
            SubstepDt = 1.5e-4,
            ParamName = "twist rate (rad/s)",
            Param = 4.0, ParamMin = -12, ParamMax = 12, ParamStep = 0.5,
            Driver = (r, t, dt, p) => r.Theta[0] += p * dt,
            CameraTarget = new Vec3(0, 0.45, 0),
        };
    }

    // 4 ------------------------------------------------------------------ drape

    public static Scene Drape()
    {
        int n = 90;
        var rod = Rod.Create(Line(n, new Vec3(-0.55, 1.05, 0), new Vec3(0.55, 1.05, 0)));
        rod.MassPerNode = 0.003;
        rod.BendStiffness = 3e-4;
        rod.TwistStiffness = 3e-4;
        rod.Damping = 0.5;
        rod.Radius = 0.011;
        rod.InitRestState();
        rod.Pin(0);
        rod.Pin(n - 1);
        var left0 = rod.X[0];
        double amp = 0; // smoothed swing amplitude
        return new Scene
        {
            Name = "4. Cable drape",
            Blurb = "Ends pinned, free to rotate: catenary with bending stiffness. Left/Right swings the left pin.",
            Rod = rod,
            ParamName = "swing amplitude",
            Param = 0.0, ParamMin = 0, ParamMax = 0.35, ParamStep = 0.02,
            Driver = (r, t, dt, p) =>
            {
                // low-pass the amplitude toward the target so Left/Right is jerk-free
                amp += (p - amp) * Math.Min(2.0 * dt, 1.0);
                if (amp > 1e-5)
                    r.X[0] = left0 + new Vec3(amp * Math.Sin(1.6 * t), 0, amp * 0.6 * Math.Cos(1.6 * t));
            },
            CameraTarget = new Vec3(0, 0.6, 0),
        };
    }

    // 5 -------------------------------------------------------------- coil drop

    public static Scene CoilDrop()
    {
        int n = 140;
        var pts = new Vec3[n];
        double turns = 5.0, coilR = 0.16, height0 = 0.55, pitch = 0.05;
        for (int i = 0; i < n; i++)
        {
            double s = i / (double)(n - 1);
            double ang = 2 * Math.PI * turns * s;
            pts[i] = new Vec3(coilR * Math.Cos(ang), height0 + pitch * turns * s, coilR * Math.Sin(ang));
        }
        var rod = Rod.Create(pts); // rest state = the coil itself is NOT rest: rest is per-edge lengths only
        rod.MassPerNode = 0.0025;
        rod.BendStiffness = 4e-4;
        rod.TwistStiffness = 5e-4;
        rod.Damping = 0.6;
        rod.Radius = 0.012;
        rod.InitRestState(); // straight rest curvature: the coil wants to unwind as it falls
        return new Scene
        {
            Name = "5. Coil drop",
            Blurb = "A coiled rod (straight rest shape) dropped on the floor: unwinding, writhing, self-contact.",
            Rod = rod,
            SelfContact = true,
            SubstepDt = 1.5e-4,
            ParamName = null,
            CameraTarget = new Vec3(0, 0.25, 0),
            CameraDistance = 1.9,
        };
    }

    // 6 ------------------------------------------------------------------ whirl

    public static Scene Whirl()
    {
        int n = 70;
        var rod = Rod.Create(Line(n, new Vec3(0, 1.35, 0), new Vec3(0, 0.15, 0)));
        rod.MassPerNode = 0.002;
        rod.BendStiffness = 8e-4;
        rod.TwistStiffness = 8e-4;
        rod.Damping = 0.35;
        rod.Radius = 0.010;
        rod.InitRestState();
        rod.ClampStart();
        var top0 = rod.X[0];
        var top1 = rod.X[1];
        return new Scene
        {
            Name = "6. Whirl",
            Blurb = "Top clamp moved in a circle: travelling helical waves. Left/Right = crank frequency.",
            Rod = rod,
            SelfContact = true,
            ParamName = "crank frequency (Hz)",
            Param = 1.2, ParamMin = 0, ParamMax = 4.0, ParamStep = 0.1,
            Driver = (r, t, dt, p) =>
            {
                double w = 2 * Math.PI * p;
                // ramp the crank radius over the first second: driving pinned nodes with a
                // positional jump would be converted into a velocity impulse by projection
                double ramp = Math.Min(t, 1.0);
                double rad = 0.06 * ramp * ramp * (3 - 2 * ramp); // smoothstep
                // rigid clamp translation: both pinned nodes get the same offset, so the
                // fully pinned edge keeps its rest length (it cannot be projected)
                var off = new Vec3(rad * Math.Cos(w * t), 0, rad * Math.Sin(w * t));
                r.X[0] = top0 + off;
                r.X[1] = top1 + off;
            },
            CameraTarget = new Vec3(0, 0.7, 0),
        };
    }
}
