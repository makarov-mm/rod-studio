namespace RodStudio.Sim;

/// <summary>
/// Numerical verification of the solver, runnable headless on any OS:
///     dotnet run -- --selftest
///
/// 1. Bending force  == -grad of the discrete bending energy (central FD)
/// 2. Twist force    == -grad of the twist energy under time-parallel frame transport
/// 3. Quasistatic theta solve drives dE/dtheta to zero
/// 4. Fast projection restores edge lengths
/// 5. Cantilever tip deflection matches Euler-Bernoulli small-deflection theory
/// 6. 20k-substep torture run stays finite and bounded
/// </summary>
public static class SelfTest
{
    private static int _failures;

    public static int Run()
    {
        _failures = 0;
        Console.WriteLine("RodStudio solver self-test");
        Console.WriteLine("--------------------------");
        TestBendingGradient();
        TestTwistGradient();
        TestQuasistaticTwist();
        TestFastProjection();
        TestCantilever();
        TestTortureRun();
        TestAllScenesSmoke();
        Console.WriteLine("--------------------------");
        Console.WriteLine(_failures == 0 ? "ALL TESTS PASSED" : $"{_failures} TEST(S) FAILED");
        return _failures == 0 ? 0 : 1;
    }

    private static Rod MakeBentRod(int n = 12)
    {
        // deterministic non-planar centerline: enough curvature to exercise every term
        var rng = new Random(42);
        var pts = new Vec3[n];
        for (int i = 0; i < n; i++)
        {
            double s = i * 0.1;
            pts[i] = new Vec3(s, 0.3 * Math.Sin(2.5 * s), 0.25 * Math.Cos(1.7 * s));
        }
        var rod = Rod.Create(pts);
        // perturb positions so lengths deviate from rest (tests length-variation terms)
        for (int i = 0; i < n; i++)
            pts[i] += new Vec3(Jit(rng), Jit(rng), Jit(rng));
        Array.Copy(pts, rod.X, n);
        // random thetas
        for (int j = 0; j < rod.EdgeCount; j++)
            rod.Theta[j] = (rng.NextDouble() - 0.5) * 1.6;
        return rod;

        static double Jit(Random r) => (r.NextDouble() - 0.5) * 0.02;
    }

    // ------------------------------------------------------- bending gradient

    private static void TestBendingGradient()
    {
        var rod = MakeBentRod();
        var solver = new DerSolver();
        solver.UpdateFrames(rod);

        var f = new Vec3[rod.NodeCount];
        Array.Clear(rod.F);
        DerSolver.AddBendingForces(rod);
        Array.Copy(rod.F, f, rod.NodeCount);

        double maxRel = 0;
        const double eps = 1e-7;
        for (int k = 0; k < rod.NodeCount; k++)
            for (int axis = 0; axis < 3; axis++)
            {
                double fd = CentralDiff(rod, k, axis, eps, DerSolver.BendingEnergy);
                double an = -Component(f[k], axis); // force = -grad E
                maxRel = Math.Max(maxRel, RelErr(an, -(-fd), fd));
                // (an vs fd: analytic force component should equal -fd)
                maxRel = Math.Max(maxRel, RelErr(Component(f[k], axis), -fd, fd));
            }
        Check("bending force vs FD", maxRel < 1e-5, $"max rel err = {maxRel:e2}");
    }

    // --------------------------------------------------------- twist gradient

    /// <summary>
    /// Twist energy as a function of positions under the dynamics convention:
    /// perturb x, time-parallel transport the *base* directors from base tangents to
    /// the perturbed tangents, recompute psi, evaluate E_t with theta held fixed.
    /// The analytic force must match -dE/dx of exactly this function.
    /// </summary>
    private static void TestTwistGradient()
    {
        var rod = MakeBentRod();
        var solver = new DerSolver();
        solver.UpdateFrames(rod); // establishes base frames on the perturbed centerline

        // snapshot base state
        int n = rod.NodeCount, m = rod.EdgeCount;
        var x0 = (Vec3[])rod.X.Clone();
        var t0 = (Vec3[])rod.T.Clone();
        var d10 = (Vec3[])rod.D1.Clone();

        Array.Clear(rod.F);
        DerSolver.AddTwistForces(rod);
        var f = (Vec3[])rod.F.Clone();

        double Energy(Vec3[] xs)
        {
            double e = 0;
            var t = new Vec3[m];
            var d1 = new Vec3[m];
            for (int j = 0; j < m; j++)
            {
                t[j] = (xs[j + 1] - xs[j]).Normalized();
                Vec3 d = Vec3.ParallelTransport(d10[j], t0[j], t[j]);
                d = (d - t[j] * Vec3.Dot(d, t[j])).Normalized();
                d1[j] = d;
            }
            for (int i = 1; i < n - 1; i++)
            {
                Vec3 v = Vec3.ParallelTransport(d1[i - 1], t[i - 1], t[i]);
                double psi = Vec3.SignedAngle(v, d1[i], t[i]);
                double tau = rod.Theta[i] - rod.Theta[i - 1] + psi;
                e += rod.TwistStiffness * tau * tau / (2.0 * rod.VorLen[i]);
            }
            return e;
        }

        double maxRel = 0;
        const double eps = 1e-7;
        var xs = (Vec3[])x0.Clone();
        for (int k = 0; k < n; k++)
            for (int axis = 0; axis < 3; axis++)
            {
                xs[k] = Nudge(x0[k], axis, +eps);
                double ep = Energy(xs);
                xs[k] = Nudge(x0[k], axis, -eps);
                double em = Energy(xs);
                xs[k] = x0[k];
                double fd = (ep - em) / (2 * eps);
                maxRel = Math.Max(maxRel, RelErr(Component(f[k], axis), -fd, fd));
            }
        Check("twist force vs FD (transported frames)", maxRel < 1e-4, $"max rel err = {maxRel:e2}");
    }

    // ------------------------------------------------------- quasistatic theta

    private static void TestQuasistaticTwist()
    {
        var rod = MakeBentRod();
        rod.ThetaFixed[0] = true;
        rod.Theta[0] = 0.7;
        rod.ThetaFixed[rod.EdgeCount - 1] = true;
        rod.Theta[rod.EdgeCount - 1] = -2.9;

        var solver = new DerSolver();
        solver.UpdateFrames(rod);
        solver.SolveQuasistaticTwist(rod);

        // stationarity: dE/dtheta_j = beta ( tau_j / l_j - tau_{j+1} / l_{j+1} ) = 0
        double maxGrad = 0;
        int mm = rod.EdgeCount;
        for (int j = 0; j < mm; j++)
        {
            if (rod.ThetaFixed[j]) continue;
            double g = 0;
            if (j >= 1)
                g += (rod.Theta[j] - rod.Theta[j - 1] + rod.Psi[j]) / rod.VorLen[j];
            if (j + 1 <= mm - 1)
                g -= (rod.Theta[j + 1] - rod.Theta[j] + rod.Psi[j + 1]) / rod.VorLen[j + 1];
            maxGrad = Math.Max(maxGrad, Math.Abs(rod.TwistStiffness * g));
        }
        Check("quasistatic twist stationarity", maxGrad < 1e-10, $"max |dE/dtheta| = {maxGrad:e2}");

        bool bcOk = Math.Abs(rod.Theta[0] - 0.7) < 1e-12 &&
                    Math.Abs(rod.Theta[mm - 1] + 2.9) < 1e-12;
        Check("theta boundary conditions preserved", bcOk, "");
    }

    // --------------------------------------------------------- fast projection

    private static void TestFastProjection()
    {
        var rod = MakeBentRod();
        var solver = new DerSolver { ProjectionIterations = 10 };
        // stretch it badly
        var rng = new Random(7);
        for (int i = 2; i < rod.NodeCount; i++)
            rod.X[i] += new Vec3(rng.NextDouble() * 0.05, rng.NextDouble() * 0.05, 0);
        rod.Pin(0); rod.Pin(1);

        solver.FastProjection(rod);

        // Edge 0 connects two pinned nodes: its constraint is infeasible by construction
        // (the test displaced both pins), so measure only edges with a free node.
        double maxErr = 0;
        for (int j = 0; j < rod.EdgeCount; j++)
        {
            if (rod.InvMass[j] == 0 && rod.InvMass[j + 1] == 0) continue;
            maxErr = Math.Max(maxErr,
                Math.Abs((rod.X[j + 1] - rod.X[j]).Length - rod.RestLen[j]) / rod.RestLen[j]);
        }
        Check("fast projection restores edge lengths", maxErr < 1e-8, $"max rel len err = {maxErr:e2}");
    }

    // -------------------------------------------------------------- cantilever

    /// <summary>
    /// Clamped horizontal rod under gravity. Small-deflection Euler-Bernoulli:
    /// tip deflection = q L^4 / (8 EI), q = weight per unit length.
    /// Stiffness chosen so deflection ~4% of L (small-deflection regime).
    /// </summary>
    private static void TestCantilever()
    {
        int n = 21;
        double L = 1.0, h = L / (n - 1);
        var pts = new Vec3[n];
        for (int i = 0; i < n; i++) pts[i] = new Vec3(i * h, 0.5, 0);
        var rod = Rod.Create(pts);
        rod.MassPerNode = 0.05 / n;      // 50 g total
        rod.BendStiffness = 1.2;         // EI chosen for ~5% tip deflection (linear regime)
        rod.TwistStiffness = 1.0;
        rod.Damping = 6.0;               // settle quickly
        rod.InitRestState();
        rod.ClampStart();

        var solver = new DerSolver { GroundPlane = false };
        double dt = 2e-5;                // explicit stability for the stiff beam
        for (int s = 0; s < 140000; s++) solver.Step(rod, dt); // 2.8 s sim time, damping 6 => settled

        // Clamp fixes nodes 0 and 1: the discrete wall sits at node 1. Superpose the
        // exact Euler-Bernoulli tip deflections of the actual point loads (one per
        // free node): delta_tip(P at a) = P a^2 (3 Leff - a) / (6 EI).
        double lEff = L - h;
        double P = rod.MassPerNode * 9.81;
        double predicted = 0;
        for (int i = 2; i < n; i++)
        {
            double a = i * h - h; // distance from the wall (node 1)
            predicted += P * a * a * (3 * lEff - a) / (6.0 * rod.BendStiffness);
        }
        double measured = 0.5 - rod.X[n - 1].Y;
        double rel = Math.Abs(measured - predicted) / predicted;
        // The discrete clamp (two pinned nodes) represents the wall to O(h): the
        // measured error halves when h halves (10.3% at n=21, 4.9% at n=41), i.e.
        // the solver converges to the analytic solution at first order in the
        // boundary. n=21 keeps the self-test fast; the threshold reflects that.
        Check("cantilever tip deflection vs Euler-Bernoulli (n=21, O(h) clamp)",
              rel < 0.13, $"predicted {predicted * 1000:f1} mm, measured {measured * 1000:f1} mm, err {rel:p1}");
    }

    // -------------------------------------------------------------- torture run

    private static void TestTortureRun()
    {
        // twisted, slack, self-contacting rod on the ground: the worst case we ship
        var rod = Scenes.BuildPlectoneme();
        var solver = new DerSolver { SelfContact = true };
        double dt = 1.5e-4;
        double injected = 0;
        bool finite = true;
        double maxSpeed = 0;
        for (int s = 0; s < 20000 && finite; s++)
        {
            injected += 6.0 * dt; // aggressive twist injection, rad/s
            rod.Theta[0] = injected;
            solver.Step(rod, dt);
            if (s % 500 == 0)
                for (int i = 0; i < rod.NodeCount; i++)
                {
                    double v = rod.V[i].Length;
                    if (double.IsNaN(v) || double.IsInfinity(v)) { finite = false; break; }
                    maxSpeed = Math.Max(maxSpeed, v);
                }
        }
        Check("20k-substep twisted-rod torture run stays finite", finite && maxSpeed < 100,
              $"max node speed {maxSpeed:f2} m/s");
    }

    // ------------------------------------------------------------- scene smoke

    /// <summary>Every shipped scene, driver included, must survive ~1.2 s of simulation.</summary>
    private static void TestAllScenesSmoke()
    {
        foreach (var scene in Scenes.All())
        {
            var solver = new DerSolver
            {
                SelfContact = scene.SelfContact,
                GroundPlane = scene.GroundPlane,
            };
            double dt = scene.SubstepDt, t = 0;
            int steps = (int)(1.2 / dt);
            bool ok = true;
            double maxSpeed = 0;
            for (int s = 0; s < steps && ok; s++)
            {
                scene.Driver?.Invoke(scene.Rod, t, dt, scene.Param);
                solver.Step(scene.Rod, dt);
                t += dt;
                if (s % 400 == 0)
                    for (int i = 0; i < scene.Rod.NodeCount; i++)
                    {
                        double v = scene.Rod.V[i].Length;
                        if (!double.IsFinite(v) || v > 200) { ok = false; break; }
                        maxSpeed = Math.Max(maxSpeed, v);
                    }
            }
            Check($"scene smoke: {scene.Name}", ok, $"max speed {maxSpeed:f2} m/s over {steps} substeps");
        }
    }

    // ------------------------------------------------------------------- utils

    private static double CentralDiff(Rod rod, int node, int axis, double eps, Func<Rod, double> energy)
    {
        Vec3 orig = rod.X[node];
        rod.X[node] = Nudge(orig, axis, +eps);
        double ep = energy(rod);
        rod.X[node] = Nudge(orig, axis, -eps);
        double em = energy(rod);
        rod.X[node] = orig;
        return (ep - em) / (2 * eps);
    }

    private static Vec3 Nudge(Vec3 v, int axis, double eps) => axis switch
    {
        0 => new Vec3(v.X + eps, v.Y, v.Z),
        1 => new Vec3(v.X, v.Y + eps, v.Z),
        _ => new Vec3(v.X, v.Y, v.Z + eps),
    };

    private static double Component(Vec3 v, int axis) => axis switch { 0 => v.X, 1 => v.Y, _ => v.Z };

    private static double RelErr(double a, double b, double scale)
    {
        double denom = Math.Max(Math.Abs(scale), 1e-6);
        return Math.Abs(a - b) / denom;
    }

    private static void Check(string name, bool ok, string detail)
    {
        Console.WriteLine($"  [{(ok ? "PASS" : "FAIL")}] {name}{(detail.Length > 0 ? "  (" + detail + ")" : "")}");
        if (!ok) _failures++;
    }
}
