using System.Numerics;

namespace RodStudio.Gfx;

/// <summary>Orbit camera: LMB rotate, RMB/MMB pan, wheel zoom.</summary>
internal sealed class Camera
{
    public Vector3 Target = new(0, 0.4f, 0);
    public float Distance = 2.2f;
    public float Yaw = 0.7f;      // radians
    public float Pitch = 0.35f;
    public float Fov = 50f * MathF.PI / 180f;

    public Vector3 Eye
    {
        get
        {
            float cp = MathF.Cos(Pitch), sp = MathF.Sin(Pitch);
            float cy = MathF.Cos(Yaw), sy = MathF.Sin(Yaw);
            return Target + new Vector3(cp * cy, sp, cp * sy) * Distance;
        }
    }

    public void Orbit(float dx, float dy)
    {
        Yaw += dx * 0.008f;
        Pitch = Math.Clamp(Pitch + dy * 0.008f, -1.45f, 1.45f);
    }

    public void Pan(float dx, float dy)
    {
        var view = ViewMatrix();
        // rows of the rotation part = camera axes in world space (row-vector convention)
        var right = new Vector3(view.M11, view.M21, view.M31);
        var up = new Vector3(view.M12, view.M22, view.M32);
        Target += (-right * dx + up * dy) * (Distance * 0.0016f);
    }

    public void Zoom(float wheel) => Distance = Math.Clamp(Distance * MathF.Pow(0.88f, wheel), 0.2f, 30f);

    public Matrix4x4 ViewMatrix() => Matrix4x4.CreateLookAt(Eye, Target, Vector3.UnitY);

    public Matrix4x4 ProjMatrix(float aspect) =>
        Matrix4x4.CreatePerspectiveFieldOfView(Fov, aspect, 0.02f, 100f);

    public Matrix4x4 ViewProj(float aspect) => ViewMatrix() * ProjMatrix(aspect);
}
