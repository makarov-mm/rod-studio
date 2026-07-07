using System.Numerics;
using RodStudio.Sim;

namespace RodStudio.Gfx;

/// <summary>
/// Renders a rod as a tube extruded along the centerline using the *material* frames:
/// the cross-section is anchored to (m1, m2), so the helical stripes in the fragment
/// shader rotate with the material of the rod, making twist directly visible.
/// The vertex buffer is rebuilt on the CPU every frame (a few thousand vertices) and
/// streamed with glBufferSubData; the index buffer is static per topology.
/// </summary>
internal sealed class TubeMesh : IDisposable
{
    private const int Sides = 16;

    private uint _vao, _vbo, _ebo;
    private int _indexCount;
    private int _nodeCount;
    private float[] _verts = [];
    private Shader _shader = null!;
    private Shader _shadow = null!;

    private const string TubeVs = """
        #version 330 core
        layout(location = 0) in vec3 aPos;
        layout(location = 1) in vec3 aNormal;
        layout(location = 2) in vec2 aUv;      // (arclength, material angle)
        uniform mat4 uViewProj;
        out vec3 vNormal;
        out vec3 vWorld;
        out vec2 vUv;
        void main() {
            vNormal = aNormal;
            vWorld = aPos;
            vUv = aUv;
            gl_Position = uViewProj * vec4(aPos, 1.0);
        }
        """;

    private const string TubeFs = """
        #version 330 core
        in vec3 vNormal;
        in vec3 vWorld;
        in vec2 vUv;
        uniform vec3 uEye;
        uniform vec3 uLightDir;   // direction TOWARD the light
        uniform vec3 uBaseColor;
        uniform vec3 uStripeColor;
        uniform vec3 uFogColor;
        out vec4 fragColor;
        void main() {
            vec3 n = normalize(vNormal);
            vec3 l = normalize(uLightDir);
            vec3 v = normalize(uEye - vWorld);
            vec3 h = normalize(l + v);

            // material-frame stripes: 4 bands around the circumference; they shear
            // visibly wherever the rod carries twist
            float band = fract(vUv.y * 0.63661977);            // angle / (pi/2)
            float stripe = smoothstep(0.42, 0.5, band) - smoothstep(0.92, 1.0, band);
            vec3 albedo = mix(uBaseColor, uStripeColor, stripe * 0.85);

            float ndl = max(dot(n, l), 0.0);
            float hemi = 0.5 + 0.5 * n.y;
            vec3 ambient = mix(vec3(0.18, 0.19, 0.23), vec3(0.34, 0.35, 0.38), hemi);
            float spec = pow(max(dot(n, h), 0.0), 48.0) * 0.35;
            vec3 col = albedo * (ambient + vec3(0.95, 0.93, 0.88) * ndl) + vec3(spec);

            float fog = clamp(length(uEye - vWorld) * 0.055 - 0.08, 0.0, 1.0);
            col = mix(col, uFogColor, fog * fog);
            fragColor = vec4(col, 1.0);
        }
        """;

    private const string ShadowVs = """
        #version 330 core
        layout(location = 0) in vec3 aPos;
        layout(location = 1) in vec3 aNormal;
        layout(location = 2) in vec2 aUv;
        uniform mat4 uViewProj;
        uniform vec3 uLightDir;   // direction TOWARD the light
        void main() {
            // project onto the ground plane y = 0 along the light rays
            vec3 d = -uLightDir;
            float t = aPos.y / max(-d.y, 1e-4);
            vec3 p = aPos + d * t;
            p.y = 0.0015;
            gl_Position = uViewProj * vec4(p, 1.0);
        }
        """;

    private const string ShadowFs = """
        #version 330 core
        out vec4 fragColor;
        void main() { fragColor = vec4(0.08, 0.09, 0.12, 0.42); }
        """;

    public void Init()
    {
        _shader = new Shader(TubeVs, TubeFs, "tube");
        _shadow = new Shader(ShadowVs, ShadowFs, "tubeShadow");
        _vao = GL.GenVertexArray();
        _vbo = GL.GenBuffer();
        _ebo = GL.GenBuffer();
    }

    /// <summary>(Re)allocate buffers for a rod with the given node count.</summary>
    public void SetTopology(int nodeCount)
    {
        _nodeCount = nodeCount;
        int ringVerts = nodeCount * (Sides + 1);
        int capVerts = 2 * (Sides + 2); // ring copy + center, per cap
        int vertCount = ringVerts + capVerts;
        _verts = new float[vertCount * 8];

        var idx = new List<uint>(nodeCount * Sides * 6 + Sides * 6);
        for (int i = 0; i < nodeCount - 1; i++)
        {
            uint r0 = (uint)(i * (Sides + 1));
            uint r1 = (uint)((i + 1) * (Sides + 1));
            for (uint s = 0; s < Sides; s++)
            {
                // CCW seen from outside: (m1, m2, t) is right-handed, rings advance along +t
                idx.Add(r0 + s); idx.Add(r1 + s + 1); idx.Add(r1 + s);
                idx.Add(r0 + s); idx.Add(r0 + s + 1); idx.Add(r1 + s + 1);
            }
        }
        // caps: [ring of Sides+1][center]
        uint capBase = (uint)ringVerts;
        for (uint s = 0; s < Sides; s++) // start cap (fan, reversed winding: faces -t)
        {
            idx.Add(capBase + Sides + 1); idx.Add(capBase + s + 1); idx.Add(capBase + s);
        }
        uint cap2 = capBase + (uint)(Sides + 2);
        for (uint s = 0; s < Sides; s++) // end cap
        {
            idx.Add(cap2 + Sides + 1); idx.Add(cap2 + s); idx.Add(cap2 + s + 1);
        }
        _indexCount = idx.Count;

        GL.BindVertexArray(_vao);
        GL.BindBuffer(GL.ARRAY_BUFFER, _vbo);
        GL.BufferDataEmpty(GL.ARRAY_BUFFER, _verts.Length * sizeof(float), GL.STREAM_DRAW);
        GL.BindBuffer(GL.ELEMENT_ARRAY_BUFFER, _ebo);
        GL.BufferData(GL.ELEMENT_ARRAY_BUFFER, idx.ToArray(), GL.STATIC_DRAW);
        int stride = 8 * sizeof(float);
        GL.VertexAttribPointer(0, 3, GL.FLOAT, false, stride, 0);
        GL.VertexAttribPointer(1, 3, GL.FLOAT, false, stride, 3 * sizeof(float));
        GL.VertexAttribPointer(2, 2, GL.FLOAT, false, stride, 6 * sizeof(float));
        GL.EnableVertexAttribArray(0);
        GL.EnableVertexAttribArray(1);
        GL.EnableVertexAttribArray(2);
        GL.BindVertexArray(0);
    }

    /// <summary>Rebuild vertices from the current rod state.</summary>
    public void Update(Rod rod)
    {
        int n = rod.NodeCount;
        if (n != _nodeCount) SetTopology(n);
        float r = (float)rod.Radius;
        int k = 0;
        double s = 0;

        for (int i = 0; i < n; i++)
        {
            // averaged frame at the node (edge frames live on segments)
            Sim.Vec3 t, m1;
            if (i == 0) { t = rod.T[0]; m1 = rod.M1[0]; }
            else if (i == n - 1) { t = rod.T[n - 2]; m1 = rod.M1[n - 2]; }
            else
            {
                t = (rod.T[i - 1] + rod.T[i]).Normalized();
                m1 = rod.M1[i - 1] + rod.M1[i];
                s += rod.RestLen[i - 1];
            }
            m1 = (m1 - t * Sim.Vec3.Dot(m1, t)).Normalized();
            Sim.Vec3 m2 = Sim.Vec3.Cross(t, m1);
            var p = rod.X[i];

            for (int j = 0; j <= Sides; j++)
            {
                double ang = 2.0 * Math.PI * j / Sides;
                Sim.Vec3 nrm = m1 * Math.Cos(ang) + m2 * Math.Sin(ang);
                Sim.Vec3 v = p + nrm * r;
                _verts[k++] = (float)v.X; _verts[k++] = (float)v.Y; _verts[k++] = (float)v.Z;
                _verts[k++] = (float)nrm.X; _verts[k++] = (float)nrm.Y; _verts[k++] = (float)nrm.Z;
                _verts[k++] = (float)s; _verts[k++] = (float)ang;
            }
        }

        // caps
        k = WriteCap(rod, k, 0, -1);
        k = WriteCap(rod, k, n - 1, +1);

        GL.BindBuffer(GL.ARRAY_BUFFER, _vbo);
        GL.BufferDataEmpty(GL.ARRAY_BUFFER, _verts.Length * sizeof(float), GL.STREAM_DRAW); // orphan
        GL.BufferSubData(GL.ARRAY_BUFFER, 0, _verts, _verts.Length);
    }

    private int WriteCap(Rod rod, int k, int node, int dir)
    {
        int edge = node == 0 ? 0 : rod.EdgeCount - 1;
        Sim.Vec3 t = rod.T[edge] * dir;
        Sim.Vec3 m1 = (rod.M1[edge] - rod.T[edge] * Sim.Vec3.Dot(rod.M1[edge], rod.T[edge])).Normalized();
        Sim.Vec3 m2 = Sim.Vec3.Cross(rod.T[edge], m1);
        float r = (float)rod.Radius;
        var p = rod.X[node];
        for (int j = 0; j <= Sides; j++)
        {
            double ang = 2.0 * Math.PI * j / Sides;
            Sim.Vec3 rad = m1 * Math.Cos(ang) + m2 * Math.Sin(ang);
            Sim.Vec3 v = p + rad * r;
            _verts[k++] = (float)v.X; _verts[k++] = (float)v.Y; _verts[k++] = (float)v.Z;
            _verts[k++] = (float)t.X; _verts[k++] = (float)t.Y; _verts[k++] = (float)t.Z;
            _verts[k++] = 0; _verts[k++] = (float)ang;
        }
        _verts[k++] = (float)p.X; _verts[k++] = (float)p.Y; _verts[k++] = (float)p.Z;
        _verts[k++] = (float)t.X; _verts[k++] = (float)t.Y; _verts[k++] = (float)t.Z;
        _verts[k++] = 0; _verts[k++] = 0;
        return k;
    }

    public void Draw(Matrix4x4 viewProj, Vector3 eye, Vector3 lightDir, Vector3 baseColor, Vector3 stripeColor, Vector3 fogColor)
    {
        _shader.Use();
        GL.UniformMatrix4(_shader.U("uViewProj"), viewProj);
        GL.Uniform3(_shader.U("uEye"), eye);
        GL.Uniform3(_shader.U("uLightDir"), lightDir);
        GL.Uniform3(_shader.U("uBaseColor"), baseColor);
        GL.Uniform3(_shader.U("uStripeColor"), stripeColor);
        GL.Uniform3(_shader.U("uFogColor"), fogColor);
        GL.BindVertexArray(_vao);
        GL.DrawElements(GL.TRIANGLES, _indexCount, GL.UNSIGNED_INT, 0);
        GL.BindVertexArray(0);
    }

    /// <summary>
    /// Planar shadow onto y = 0, stencil-guarded so overlapping shadow triangles
    /// darken the ground exactly once.
    /// </summary>
    public void DrawShadow(Matrix4x4 viewProj, Vector3 lightDir)
    {
        _shadow.Use();
        GL.UniformMatrix4(_shadow.U("uViewProj"), viewProj);
        GL.Uniform3(_shadow.U("uLightDir"), lightDir);

        GL.Enable(GL.STENCIL_TEST);
        GL.StencilFunc(GL.EQUAL, 0, 0xFF);
        GL.StencilOp(GL.KEEP, GL.KEEP, GL.INCR);
        GL.Enable(GL.BLEND);
        GL.BlendFunc(GL.SRC_ALPHA, GL.ONE_MINUS_SRC_ALPHA);
        GL.DepthMask(false);
        GL.Disable(GL.CULL_FACE);

        GL.BindVertexArray(_vao);
        GL.DrawElements(GL.TRIANGLES, _indexCount, GL.UNSIGNED_INT, 0);
        GL.BindVertexArray(0);

        GL.Enable(GL.CULL_FACE);
        GL.DepthMask(true);
        GL.Disable(GL.BLEND);
        GL.Disable(GL.STENCIL_TEST);
    }

    public void Dispose() { }
}
