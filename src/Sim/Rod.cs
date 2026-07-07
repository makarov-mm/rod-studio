namespace RodStudio.Sim;

/// <summary>
/// State of a single discrete elastic rod.
///
/// Discretization (Bergou et al., "Discrete Elastic Rods", SIGGRAPH 2008):
///   nodes    x[0..N-1]                 (centerline)
///   edges    e[j] = x[j+1] - x[j],     j = 0..M-1, M = N-1
///   per edge: unit tangent t[j], twist-free reference frame (d1[j], d2[j]),
///             scalar twist angle theta[j], material frame (m1[j], m2[j])
///   per interior vertex i = 1..N-2:
///             curvature binormal kb[i], reference twist psi[i],
///             Voronoi rest length l[i] = (rest[i-1] + rest[i]) / 2
/// </summary>
public sealed class Rod
{
    public int NodeCount;
    public int EdgeCount => NodeCount - 1;

    // --- dynamic state ---
    public Vec3[] X = [];        // positions
    public Vec3[] V = [];        // velocities
    public Vec3[] F = [];        // force accumulator
    public double[] InvMass = []; // 0 => pinned/driven node
    public double[] Theta = [];   // per-edge material frame angle (unbounded)
    public bool[] ThetaFixed = []; // per-edge: true => boundary condition on theta

    // --- frames (per edge) ---
    public Vec3[] T = [];
    public Vec3[] D1 = [];
    public Vec3[] D2 = [];
    public Vec3[] M1 = [];
    public Vec3[] M2 = [];
    public Vec3[] PrevT = [];    // tangents of previous step, for time-parallel transport

    // --- per interior vertex ---
    public Vec3[] Kb = [];       // curvature binormal
    public double[] Psi = [];     // reference twist (holonomy of the reference frame)
    public double[] VorLen = [];  // Voronoi rest length

    // --- rest quantities ---
    public double[] RestLen = []; // per edge

    // --- material parameters ---
    public double BendStiffness = 2e-3;   // alpha = E*I  [N*m^2]
    public double TwistStiffness = 1.5e-3; // beta  = G*J  [N*m^2]
    public double Radius = 0.012;          // cross-section radius [m], also used for contact & rendering
    public double MassPerNode = 0.01;      // [kg]
    public double Damping = 0.15;          // viscous drag coefficient [1/s]

    public Vec3 Gravity = new(0, -9.81, 0);

    /// <summary>Allocate all arrays and initialize rest state from current positions.</summary>
    public static Rod Create(Vec3[] positions)
    {
        int n = positions.Length;
        if (n < 3) throw new ArgumentException("rod needs at least 3 nodes");
        var r = new Rod
        {
            NodeCount = n,
            X = (Vec3[])positions.Clone(),
            V = new Vec3[n],
            F = new Vec3[n],
            InvMass = new double[n],
            Theta = new double[n - 1],
            ThetaFixed = new bool[n - 1],
            T = new Vec3[n - 1],
            D1 = new Vec3[n - 1],
            D2 = new Vec3[n - 1],
            M1 = new Vec3[n - 1],
            M2 = new Vec3[n - 1],
            PrevT = new Vec3[n - 1],
            Kb = new Vec3[n],
            Psi = new double[n],
            VorLen = new double[n],
            RestLen = new double[n - 1],
        };
        for (int i = 0; i < n; i++) r.InvMass[i] = 1.0; // scaled by mass later
        r.InitRestState();
        return r;
    }

    public void InitRestState()
    {
        int m = EdgeCount;
        for (int j = 0; j < m; j++)
        {
            Vec3 e = X[j + 1] - X[j];
            RestLen[j] = e.Length;
            T[j] = e.Normalized();
            PrevT[j] = T[j];
        }
        for (int i = 1; i < NodeCount - 1; i++)
            VorLen[i] = 0.5 * (RestLen[i - 1] + RestLen[i]);

        // Space-parallel transport of an initial director along the rod:
        // yields a twist-free (Bishop) reference frame, so psi == 0 in the rest state.
        Vec3 up = Math.Abs(T[0].Y) < 0.9 ? Vec3.UnitY : Vec3.UnitX;
        D1[0] = (up - T[0] * Vec3.Dot(up, T[0])).Normalized();
        D2[0] = Vec3.Cross(T[0], D1[0]);
        for (int j = 1; j < m; j++)
        {
            Vec3 d = Vec3.ParallelTransport(D1[j - 1], T[j - 1], T[j]);
            d = (d - T[j] * Vec3.Dot(d, T[j])).Normalized();
            D1[j] = d;
            D2[j] = Vec3.Cross(T[j], D1[j]);
        }
        Array.Clear(Theta);
        for (int i = 0; i < NodeCount; i++) V[i] = Vec3.Zero;

        double invM = 1.0 / MassPerNode;
        for (int i = 0; i < NodeCount; i++)
            if (InvMass[i] > 0) InvMass[i] = invM;
    }

    public void Pin(int node) => InvMass[node] = 0;

    /// <summary>Pin nodes 0 and 1 (or last two) => clamped end: position + direction fixed.</summary>
    public void ClampStart() { Pin(0); Pin(1); ThetaFixed[0] = true; }
    public void ClampEnd() { Pin(NodeCount - 1); Pin(NodeCount - 2); ThetaFixed[EdgeCount - 1] = true; }

    public double TotalRestLength()
    {
        double s = 0;
        for (int j = 0; j < EdgeCount; j++) s += RestLen[j];
        return s;
    }
}
