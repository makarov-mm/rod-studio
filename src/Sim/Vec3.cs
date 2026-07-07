using System.Runtime.CompilerServices;

namespace RodStudio.Sim;

/// <summary>
/// Double-precision 3D vector. The solver runs in double for robustness of the
/// stiff bending/twist forces and the constraint projection; conversion to
/// float happens only at the rendering boundary.
/// </summary>
public readonly struct Vec3
{
    public readonly double X, Y, Z;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vec3(double x, double y, double z) { X = x; Y = y; Z = z; }

    public static readonly Vec3 Zero = new(0, 0, 0);
    public static readonly Vec3 UnitX = new(1, 0, 0);
    public static readonly Vec3 UnitY = new(0, 1, 0);
    public static readonly Vec3 UnitZ = new(0, 0, 1);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vec3 operator +(Vec3 a, Vec3 b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vec3 operator -(Vec3 a, Vec3 b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vec3 operator -(Vec3 a) => new(-a.X, -a.Y, -a.Z);
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vec3 operator *(Vec3 a, double s) => new(a.X * s, a.Y * s, a.Z * s);
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vec3 operator *(double s, Vec3 a) => new(a.X * s, a.Y * s, a.Z * s);
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vec3 operator /(Vec3 a, double s) => new(a.X / s, a.Y / s, a.Z / s);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double Dot(Vec3 a, Vec3 b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vec3 Cross(Vec3 a, Vec3 b) =>
        new(a.Y * b.Z - a.Z * b.Y,
            a.Z * b.X - a.X * b.Z,
            a.X * b.Y - a.Y * b.X);

    public double Length => Math.Sqrt(X * X + Y * Y + Z * Z);
    public double LengthSq => X * X + Y * Y + Z * Z;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vec3 Normalized()
    {
        double len = Length;
        return len > 1e-300 ? this / len : UnitX;
    }

    /// <summary>Signed angle from a to b around unit axis (right-hand rule).</summary>
    public static double SignedAngle(Vec3 a, Vec3 b, Vec3 axis) =>
        Math.Atan2(Dot(Cross(a, b), axis), Dot(a, b));

    /// <summary>
    /// Parallel transport of v from unit tangent t1 to unit tangent t2:
    /// the minimal rotation taking t1 to t2, applied to v. Twist-free by construction.
    /// </summary>
    public static Vec3 ParallelTransport(Vec3 v, Vec3 t1, Vec3 t2)
    {
        Vec3 axis = Cross(t1, t2);
        double s2 = axis.LengthSq;
        if (s2 < 1e-24) return v; // tangents (anti)parallel or identical
        double c = Dot(t1, t2);
        // Rodrigues with unnormalized axis: R v = c*v + axis×v + (axis·v)/(1+c) * axis
        // Valid for c > -1; for near-antiparallel tangents fall back to identity
        // (never happens for adjacent edges of a resolved rod).
        if (c < -0.999999) return v;
        return v * c + Cross(axis, v) + axis * (Dot(axis, v) / (1.0 + c));
    }

    public override string ToString() => $"({X:g5}, {Y:g5}, {Z:g5})";
}
