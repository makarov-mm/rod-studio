namespace RodStudio.Sim;

/// <summary>
/// Discrete Elastic Rods solver (isotropic Kirchhoff rod, circular cross-section).
///
/// References:
///   [1] Bergou, Wardetzky, Robinson, Audoly, Grinspun.
///       "Discrete Elastic Rods", ACM SIGGRAPH 2008.
///   [2] Bergou, Audoly, Vouga, Wardetzky, Grinspun.
///       "Discrete Viscous Threads", ACM SIGGRAPH 2010.
///   [3] Goldenthal, Harmon, Fattal, Bercovier, Grinspun.
///       "Efficient Simulation of Inextensible Cloth", SIGGRAPH 2007 (fast projection).
///
/// Per substep:
///   1. update tangents; time-parallel transport of the reference frames
///   2. reference twist psi_i from frame holonomy
///   3. quasistatic material-frame solve: tridiagonal system for theta
///   4. elastic forces: bending  E_b = sum_i  alpha |kb_i|^2       / (2 l_i)
///                      twisting E_t = sum_i  beta (tau_i)^2       / (2 l_i),
///                      tau_i = theta_i - theta_{i-1} + psi_i
///      (isotropic bending is independent of theta, which keeps the theta-solve linear
///       and removes frame-rotation terms from the bending gradient)
///   5. symplectic Euler
///   6. fast projection for inextensibility (tridiagonal Thomas solve, few Newton iterations)
///   7. contacts: ground plane + rod self-contact (spatial hash, segment-segment penalty)
///
/// All gradients are exact gradients of the discrete energies above and are verified
/// against central finite differences in SelfTest.cs.
/// </summary>
public sealed class DerSolver
{
    public int ProjectionIterations = 4;
    public bool SelfContact = false;
    public bool GroundPlane = true;
    public double GroundY = 0;
    public double GroundFriction = 0.35;
    public double ContactStiffness = 60.0;   // N/m per contact
    public double ContactDamping = 4.0;

    // scratch buffers for tridiagonal solves (grown on demand, zero steady-state allocation)
    private double[] _a = new double[64], _b = new double[64], _c = new double[64], _d = new double[64];
    private double[] _lambda = new double[64];
    private Vec3[] _xPre = new Vec3[64];

    // spatial hash scratch
    private readonly Dictionary<long, List<int>> _hash = new();

    // ------------------------------------------------------------------ step

    public void Step(Rod rod, double dt)
    {
        UpdateFrames(rod);
        SolveQuasistaticTwist(rod);
        ComputeForces(rod);
        if (GroundPlane) AddGroundPenalty(rod);
        if (SelfContact) AddSelfContact(rod);

        int n = rod.NodeCount;
        EnsureScratch(n);
        for (int i = 0; i < n; i++)
        {
            _xPre[i] = rod.X[i];
            if (rod.InvMass[i] == 0) { rod.V[i] = Vec3.Zero; continue; }
            rod.V[i] += rod.F[i] * (rod.InvMass[i] * dt);
            rod.V[i] *= Math.Max(0.0, 1.0 - rod.Damping * dt);
            rod.X[i] += rod.V[i] * dt;
        }

        FastProjection(rod);

        // velocity correction from the projection displacement
        double invDt = 1.0 / dt;
        for (int i = 0; i < n; i++)
            if (rod.InvMass[i] > 0)
                rod.V[i] = (rod.X[i] - _xPre[i]) * invDt;

        if (GroundPlane) ResolveGroundPosition(rod);
    }

    // ---------------------------------------------------------------- frames

    /// <summary>Tangents + time-parallel transport of reference directors, then psi.</summary>
    public void UpdateFrames(Rod rod)
    {
        int m = rod.EdgeCount;
        for (int j = 0; j < m; j++)
        {
            Vec3 t = (rod.X[j + 1] - rod.X[j]).Normalized();
            Vec3 d = Vec3.ParallelTransport(rod.D1[j], rod.PrevT[j], t);
            d = (d - t * Vec3.Dot(d, t)).Normalized();
            rod.T[j] = t;
            rod.D1[j] = d;
            rod.D2[j] = Vec3.Cross(t, d);
            rod.PrevT[j] = t;
        }
        ComputeRefTwist(rod);
        ComputeKb(rod);
    }

    public static void ComputeRefTwist(Rod rod)
    {
        for (int i = 1; i < rod.NodeCount - 1; i++)
        {
            Vec3 v = Vec3.ParallelTransport(rod.D1[i - 1], rod.T[i - 1], rod.T[i]);
            rod.Psi[i] = Vec3.SignedAngle(v, rod.D1[i], rod.T[i]);
        }
    }

    public static void ComputeKb(Rod rod)
    {
        for (int i = 1; i < rod.NodeCount - 1; i++)
        {
            Vec3 a = rod.X[i] - rod.X[i - 1];
            Vec3 b = rod.X[i + 1] - rod.X[i];
            double chi = a.Length * b.Length + Vec3.Dot(a, b);
            rod.Kb[i] = Vec3.Cross(a, b) * (2.0 / Math.Max(chi, 1e-300));
        }
    }

    public static void UpdateMaterialFrames(Rod rod)
    {
        for (int j = 0; j < rod.EdgeCount; j++)
        {
            double c = Math.Cos(rod.Theta[j]), s = Math.Sin(rod.Theta[j]);
            rod.M1[j] = rod.D1[j] * c + rod.D2[j] * s;
            rod.M2[j] = rod.D1[j] * (-s) + rod.D2[j] * c;
        }
    }

    // ------------------------------------------------------ quasistatic twist

    /// <summary>
    /// Minimize twist energy over free thetas. For an isotropic rod the bending energy
    /// does not depend on theta, so stationarity dE/dtheta_j = 0 is linear:
    ///     tau_j / l_j - tau_{j+1} / l_{j+1} = 0   (interior edges)
    /// with tau_i = theta_i - theta_{i-1} + psi_i. Tridiagonal, solved with Thomas.
    /// </summary>
    public void SolveQuasistaticTwist(Rod rod)
    {
        int m = rod.EdgeCount;
        EnsureScratch(m);
        var (a, b, c, d) = (_a, _b, _c, _d);

        // Build equations for every edge; fixed thetas become identity rows.
        for (int j = 0; j < m; j++)
        {
            if (rod.ThetaFixed[j])
            {
                a[j] = 0; b[j] = 1; c[j] = 0; d[j] = rod.Theta[j];
                continue;
            }
            double diag = 0, lower = 0, upper = 0, rhs = 0;
            if (j >= 1) // tau_j exists (vertex j couples edges j-1, j)
            {
                double w = 1.0 / rod.VorLen[j];
                diag += w; lower -= w; rhs -= w * rod.Psi[j];
            }
            if (j + 1 <= m - 1) // tau_{j+1} exists
            {
                double w = 1.0 / rod.VorLen[j + 1];
                diag += w; upper -= w; rhs += w * rod.Psi[j + 1];
            }
            if (diag == 0) { a[j] = 0; b[j] = 1; c[j] = 0; d[j] = rod.Theta[j]; continue; }
            a[j] = lower; b[j] = diag; c[j] = upper; d[j] = rhs;
        }

        // If nothing anchors the system (both ends free), fix theta[0] to avoid the
        // rank deficiency of the pure-Neumann problem.
        bool anyFixed = false;
        for (int j = 0; j < m; j++) if (rod.ThetaFixed[j]) { anyFixed = true; break; }
        if (!anyFixed) { a[0] = 0; b[0] = 1; c[0] = 0; d[0] = rod.Theta[0]; }

        ThomasSolve(a, b, c, d, rod.Theta, m);
        UpdateMaterialFrames(rod);
    }

    // ------------------------------------------------------------------ forces

    public void ComputeForces(Rod rod)
    {
        int n = rod.NodeCount;
        for (int i = 0; i < n; i++)
            rod.F[i] = rod.InvMass[i] > 0 ? rod.Gravity * rod.MassPerNode : Vec3.Zero;
        AddBendingForces(rod);
        AddTwistForces(rod);
    }

    /// <summary>
    /// Bending: E_b = sum_i alpha |kb_i|^2 / (2 l_i),  kb = 2 (a x b) / chi,
    /// chi = |a||b| + a.b, a = e_{i-1}, b = e_i (current lengths; the gradient below is
    /// the exact gradient of this discrete energy, including length variation).
    ///
    ///   d(kb)/da . da = [ 2 (da x b) - kb (g_a . da) ] / chi,  g_a = (|b|/|a|) a + b
    ///   d(kb)/db . db = [ 2 (a x db) - kb (g_b . db) ] / chi,  g_b = (|a|/|b|) b + a
    ///
    /// J_a^T kb = ( 2 b x kb - g_a |kb|^2 ) / chi
    /// J_b^T kb = (-2 a x kb - g_b |kb|^2 ) / chi
    /// with  d/dx_{i-1} = -J_a,  d/dx_i = J_a - J_b,  d/dx_{i+1} = J_b.
    /// </summary>
    public static void AddBendingForces(Rod rod)
    {
        double alpha = rod.BendStiffness;
        for (int i = 1; i < rod.NodeCount - 1; i++)
        {
            Vec3 a = rod.X[i] - rod.X[i - 1];
            Vec3 b = rod.X[i + 1] - rod.X[i];
            double la = a.Length, lb = b.Length;
            double chi = la * lb + Vec3.Dot(a, b);
            if (chi < 1e-300) continue;
            Vec3 kb = Vec3.Cross(a, b) * (2.0 / chi);
            double kb2 = kb.LengthSq;

            Vec3 ga = a * (lb / la) + b;
            Vec3 gb = b * (la / lb) + a;

            Vec3 jaTkb = (Vec3.Cross(b, kb) * 2.0 - ga * kb2) / chi;
            Vec3 jbTkb = (Vec3.Cross(a, kb) * (-2.0) - gb * kb2) / chi;

            double c = alpha / rod.VorLen[i];
            // F = -dE/dx = -c * (grad kb)^T kb
            rod.F[i - 1] += jaTkb * c;              // -(-J_a)^T kb * c
            rod.F[i] -= (jaTkb - jbTkb) * c;
            rod.F[i + 1] -= jbTkb * c;
        }
    }

    /// <summary>
    /// Twisting: E_t = sum_i beta tau_i^2 / (2 l_i). The centerline feels twist through
    /// the holonomy of the reference frame: grad psi_i (Bergou et al. 2008, sec. 7):
    ///   d(psi_i)/dx_{i-1} = -kb_i / (2 |e_{i-1}|)
    ///   d(psi_i)/dx_{i+1} =  kb_i / (2 |e_i|)
    ///   d(psi_i)/dx_i     = -(sum of the above)
    /// Signs verified against central finite differences of the transported-frame
    /// energy (SelfTest.TestTwistGradient), which arbitrates the sign convention.
    /// </summary>
    public static void AddTwistForces(Rod rod)
    {
        double beta = rod.TwistStiffness;
        for (int i = 1; i < rod.NodeCount - 1; i++)
        {
            double tau = rod.Theta[i] - rod.Theta[i - 1] + rod.Psi[i];
            double coeff = beta * tau / rod.VorLen[i];
            if (coeff == 0) continue;

            double la = (rod.X[i] - rod.X[i - 1]).Length;
            double lb = (rod.X[i + 1] - rod.X[i]).Length;
            Vec3 gPrev = rod.Kb[i] * (-0.5 / la);
            Vec3 gNext = rod.Kb[i] * (0.5 / lb);
            Vec3 gMid = -(gPrev + gNext);

            rod.F[i - 1] -= gPrev * coeff;
            rod.F[i] -= gMid * coeff;
            rod.F[i + 1] -= gNext * coeff;
        }
    }

    public static double BendingEnergy(Rod rod)
    {
        double e = 0;
        for (int i = 1; i < rod.NodeCount - 1; i++)
        {
            Vec3 a = rod.X[i] - rod.X[i - 1];
            Vec3 b = rod.X[i + 1] - rod.X[i];
            double chi = a.Length * b.Length + Vec3.Dot(a, b);
            Vec3 kb = Vec3.Cross(a, b) * (2.0 / chi);
            e += rod.BendStiffness * kb.LengthSq / (2.0 * rod.VorLen[i]);
        }
        return e;
    }

    public static double TwistEnergy(Rod rod)
    {
        double e = 0;
        for (int i = 1; i < rod.NodeCount - 1; i++)
        {
            double tau = rod.Theta[i] - rod.Theta[i - 1] + rod.Psi[i];
            e += rod.TwistStiffness * tau * tau / (2.0 * rod.VorLen[i]);
        }
        return e;
    }

    // --------------------------------------------------------- fast projection

    /// <summary>
    /// Enforce |e_j| = restLen_j (Goldenthal et al. 2007). Constraints couple only
    /// neighboring edges, so J W J^T is tridiagonal; each Newton iteration is one
    /// Thomas solve. Pinned nodes participate with w = 0.
    /// </summary>
    public void FastProjection(Rod rod)
    {
        int m = rod.EdgeCount;
        EnsureScratch(m + 1);
        var (a, b, c, d) = (_a, _b, _c, _d);

        for (int iter = 0; iter < ProjectionIterations; iter++)
        {
            double maxC = 0;
            for (int j = 0; j < m; j++)
            {
                Vec3 e = rod.X[j + 1] - rod.X[j];
                double len = e.Length;
                double C = len - rod.RestLen[j];
                d[j] = C;
                maxC = Math.Max(maxC, Math.Abs(C));
                rod.T[j] = e / Math.Max(len, 1e-300); // reuse T as unit-edge scratch
            }
            if (maxC < 1e-9) break;

            // (J W J^T): diag_j = w_j + w_{j+1};  off(j, j+1) = -w_{j+1} * (t_j . t_{j+1})
            for (int j = 0; j < m; j++)
            {
                double diag = rod.InvMass[j] + rod.InvMass[j + 1];
                b[j] = diag > 0 ? diag : 1.0; // fully pinned edge: identity row, rhs 0
                if (diag == 0) d[j] = 0;
                a[j] = j > 0 ? -rod.InvMass[j] * Vec3.Dot(rod.T[j - 1], rod.T[j]) : 0;
                c[j] = j < m - 1 ? -rod.InvMass[j + 1] * Vec3.Dot(rod.T[j], rod.T[j + 1]) : 0;
            }

            ThomasSolve(a, b, c, d, _lambda, m);

            for (int j = 0; j < m; j++)
            {
                double lam = _lambda[j];
                if (lam == 0) continue;
                Vec3 dx = rod.T[j] * lam;
                if (rod.InvMass[j] > 0) rod.X[j] += dx * rod.InvMass[j];
                if (rod.InvMass[j + 1] > 0) rod.X[j + 1] -= dx * rod.InvMass[j + 1];
            }
        }
    }

    // ----------------------------------------------------------------- contact

    private void AddGroundPenalty(Rod rod)
    {
        double r = rod.Radius;
        for (int i = 0; i < rod.NodeCount; i++)
        {
            double pen = GroundY + r - rod.X[i].Y;
            if (pen <= 0) continue;
            double fn = ContactStiffness * pen - ContactDamping * rod.V[i].Y;
            if (fn <= 0) continue;
            rod.F[i] += new Vec3(0, fn, 0);
            // Coulomb-style friction against tangential velocity
            Vec3 vt = new(rod.V[i].X, 0, rod.V[i].Z);
            double vtLen = vt.Length;
            if (vtLen > 1e-9)
                rod.F[i] -= vt * (Math.Min(GroundFriction * fn / vtLen, rod.MassPerNode * 60.0));
        }
    }

    private void ResolveGroundPosition(Rod rod)
    {
        double floor = GroundY + rod.Radius;
        for (int i = 0; i < rod.NodeCount; i++)
        {
            if (rod.InvMass[i] == 0 || rod.X[i].Y >= floor) continue;
            rod.X[i] = new Vec3(rod.X[i].X, floor, rod.X[i].Z);
            if (rod.V[i].Y < 0)
                rod.V[i] = new Vec3(rod.V[i].X * 0.9, 0, rod.V[i].Z * 0.9);
        }
    }

    /// <summary>
    /// Segment-segment self-contact via a uniform spatial hash over segment AABBs.
    /// Penalty + normal damping when closest distance &lt; 2r. Needed for plectonemes
    /// and dropped coils; without it strands pass through each other.
    /// The cell dictionary and its lists persist across substeps (lists are cleared,
    /// not reallocated), so the steady-state allocation rate is zero.
    /// </summary>
    private void AddSelfContact(Rod rod)
    {
        int m = rod.EdgeCount;
        double r2 = 2.0 * rod.Radius;
        double cell = Math.Max(r2 * 2.0, rod.RestLen[0] * 1.5);
        double inv = 1.0 / cell;

        foreach (var kv in _hash) kv.Value.Clear(); // reuse lists: no per-substep allocation

        for (int j = 0; j < m; j++)
        {
            Vec3 p = rod.X[j], q = rod.X[j + 1];
            int x0 = (int)Math.Floor(Math.Min(p.X, q.X) * inv), x1 = (int)Math.Floor(Math.Max(p.X, q.X) * inv);
            int y0 = (int)Math.Floor(Math.Min(p.Y, q.Y) * inv), y1 = (int)Math.Floor(Math.Max(p.Y, q.Y) * inv);
            int z0 = (int)Math.Floor(Math.Min(p.Z, q.Z) * inv), z1 = (int)Math.Floor(Math.Max(p.Z, q.Z) * inv);
            for (int x = x0; x <= x1; x++)
                for (int y = y0; y <= y1; y++)
                    for (int z = z0; z <= z1; z++)
                    {
                        long key = Key(x, y, z);
                        if (!_hash.TryGetValue(key, out var list))
                            _hash[key] = list = new List<int>(8); // amortized: only for never-seen cells
                        list.Add(j);
                    }
        }

        foreach (var list in _hash.Values)
        {
            int cnt = list.Count;
            if (cnt < 2) continue;
            for (int u = 0; u < cnt; u++)
                for (int w = u + 1; w < cnt; w++)
                {
                    int j = list[u], k = list[w];
                    if (Math.Abs(j - k) <= 2) continue;   // skip topological neighbors
                    if (j > k) (j, k) = (k, j);
                    ContactPair(rod, j, k, r2);
                }
        }
    }

    private void ContactPair(Rod rod, int j, int k, double minDist)
    {
        ClosestSegmentSegment(rod.X[j], rod.X[j + 1], rod.X[k], rod.X[k + 1],
                              out double s, out double t, out Vec3 pj, out Vec3 pk);
        Vec3 n = pj - pk;
        double dist = n.Length;
        if (dist >= minDist || dist < 1e-12) return;
        n /= dist;
        double pen = minDist - dist;

        Vec3 vj = rod.V[j] * (1 - s) + rod.V[j + 1] * s;
        Vec3 vk = rod.V[k] * (1 - t) + rod.V[k + 1] * t;
        double vn = Vec3.Dot(vj - vk, n);

        double fmag = ContactStiffness * pen - ContactDamping * vn;
        if (fmag <= 0) return;
        Vec3 f = n * fmag;

        rod.F[j] += f * (1 - s);
        rod.F[j + 1] += f * s;
        rod.F[k] -= f * (1 - t);
        rod.F[k + 1] -= f * t;
    }

    /// <summary>Closest points between segments p1p2 and q1q2 (Ericson, Real-Time Collision Detection).</summary>
    public static void ClosestSegmentSegment(Vec3 p1, Vec3 p2, Vec3 q1, Vec3 q2,
                                             out double s, out double t, out Vec3 cp, out Vec3 cq)
    {
        Vec3 d1 = p2 - p1, d2 = q2 - q1, r = p1 - q1;
        double a = d1.LengthSq, e = d2.LengthSq, f = Vec3.Dot(d2, r);
        double eps = 1e-14;
        if (a <= eps && e <= eps) { s = t = 0; cp = p1; cq = q1; return; }
        if (a <= eps) { s = 0; t = Math.Clamp(f / e, 0, 1); }
        else
        {
            double cDot = Vec3.Dot(d1, r);
            if (e <= eps) { t = 0; s = Math.Clamp(-cDot / a, 0, 1); }
            else
            {
                double b = Vec3.Dot(d1, d2);
                double denom = a * e - b * b;
                s = denom > eps ? Math.Clamp((b * f - cDot * e) / denom, 0, 1) : 0;
                t = (b * s + f) / e;
                if (t < 0) { t = 0; s = Math.Clamp(-cDot / a, 0, 1); }
                else if (t > 1) { t = 1; s = Math.Clamp((b - cDot) / a, 0, 1); }
            }
        }
        cp = p1 + d1 * s;
        cq = q1 + d2 * t;
    }

    // ------------------------------------------------------------------ misc

    private static long Key(int x, int y, int z) =>
        unchecked(((long)x * 73856093) ^ ((long)y * 19349663) ^ ((long)z * 83492791));

    private void EnsureScratch(int n)
    {
        if (_a.Length >= n) { if (_xPre.Length < n) Array.Resize(ref _xPre, n); return; }
        int cap = Math.Max(n, _a.Length * 2);
        Array.Resize(ref _a, cap);
        Array.Resize(ref _b, cap);
        Array.Resize(ref _c, cap);
        Array.Resize(ref _d, cap);
        Array.Resize(ref _lambda, cap);
        Array.Resize(ref _xPre, cap);
    }

    /// <summary>Thomas algorithm for a tridiagonal system; overwrites scratch d.</summary>
    public static void ThomasSolve(double[] a, double[] b, double[] c, double[] d, double[] x, int n)
    {
        // forward sweep (in-place on c', d' via local copies to keep a,b intact for reuse)
        Span<double> cp = n <= 512 ? stackalloc double[n] : new double[n];
        cp[0] = c[0] / b[0];
        d[0] = d[0] / b[0];
        for (int i = 1; i < n; i++)
        {
            double mFac = 1.0 / (b[i] - a[i] * cp[i - 1]);
            cp[i] = c[i] * mFac;
            d[i] = (d[i] - a[i] * d[i - 1]) * mFac;
        }
        x[n - 1] = d[n - 1];
        for (int i = n - 2; i >= 0; i--)
            x[i] = d[i] - cp[i] * x[i + 1];
    }
}
