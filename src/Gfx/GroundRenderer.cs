using System.Numerics;

namespace RodStudio.Gfx;

/// <summary>A large quad at y = 0 with a procedural, fwidth-antialiased grid and radial fade.</summary>
internal sealed class GroundRenderer
{
    private uint _vao, _vbo;
    private Shader _shader = null!;

    private const string Vs = """
        #version 330 core
        layout(location = 0) in vec3 aPos;
        uniform mat4 uViewProj;
        out vec3 vWorld;
        void main() {
            vWorld = aPos;
            gl_Position = uViewProj * vec4(aPos, 1.0);
        }
        """;

    private const string Fs = """
        #version 330 core
        in vec3 vWorld;
        uniform vec3 uFogColor;
        out vec4 fragColor;

        float gridLine(vec2 p, float scale) {
            vec2 q = p * scale;
            vec2 g = abs(fract(q - 0.5) - 0.5) / fwidth(q);
            return 1.0 - min(min(g.x, g.y), 1.0);
        }

        void main() {
            vec3 base = vec3(0.16, 0.17, 0.20);
            float minor = gridLine(vWorld.xz, 10.0) * 0.16;  // 10 cm cells
            float major = gridLine(vWorld.xz, 2.0)  * 0.30;  // 50 cm cells
            vec3 col = base + vec3(minor + major);
            float d = length(vWorld.xz);
            float fade = clamp(d * 0.10, 0.0, 1.0);
            col = mix(col, uFogColor, fade * fade);
            fragColor = vec4(col, 1.0);
        }
        """;

    public void Init()
    {
        _shader = new Shader(Vs, Fs, "ground");
        _vao = GL.GenVertexArray();
        _vbo = GL.GenBuffer();
        const float S = 40f;
        float[] quad =
        {
            -S, 0, -S,   -S, 0, S,   S, 0, S,
            -S, 0, -S,    S, 0, S,   S, 0, -S,
        };
        GL.BindVertexArray(_vao);
        GL.BindBuffer(GL.ARRAY_BUFFER, _vbo);
        GL.BufferData(GL.ARRAY_BUFFER, quad, GL.STATIC_DRAW);
        GL.VertexAttribPointer(0, 3, GL.FLOAT, false, 3 * sizeof(float), 0);
        GL.EnableVertexAttribArray(0);
        GL.BindVertexArray(0);
    }

    public void Draw(Matrix4x4 viewProj, Vector3 fogColor)
    {
        _shader.Use();
        GL.UniformMatrix4(_shader.U("uViewProj"), viewProj);
        GL.Uniform3(_shader.U("uFogColor"), fogColor);
        GL.Disable(GL.CULL_FACE);
        GL.BindVertexArray(_vao);
        GL.DrawArrays(GL.TRIANGLES, 0, 6);
        GL.BindVertexArray(0);
        GL.Enable(GL.CULL_FACE);
    }
}
