using System.Numerics;

namespace RodStudio.Gfx;

/// <summary>
/// Pixel-space text overlay built from the embedded public-domain 8x8 bitmap font.
/// The font is baked into a 128x48 R8 atlas (16x6 glyphs) at init; each frame the
/// requested strings are batched into one dynamic vertex buffer and drawn in a single
/// call per color group.
/// </summary>
internal sealed class TextHud
{
    private const int AtlasCols = 16;
    private const int AtlasRows = 6;
    private const int AtlasW = AtlasCols * 8;   // 128
    private const int AtlasH = AtlasRows * 8;   // 48

    private uint _vao, _vbo, _texture;
    private Shader _shader = null!;
    private readonly List<float> _batch = new(4096);
    private readonly List<(int Start, int Count, Vector3 Color, float Alpha)> _runs = new();

    private const string Vs = """
        #version 330 core
        layout(location = 0) in vec2 aPos;   // pixels, origin top-left
        layout(location = 1) in vec2 aUv;
        uniform vec3 uScreen; // (w, h, unused) — uploaded with glUniform3f
        out vec2 vUv;
        void main() {
            vec2 ndc = vec2(aPos.x / uScreen.x * 2.0 - 1.0, 1.0 - aPos.y / uScreen.y * 2.0);
            vUv = aUv;
            gl_Position = vec4(ndc, 0.0, 1.0);
        }
        """;

    private const string Fs = """
        #version 330 core
        in vec2 vUv;
        uniform sampler2D uFont;
        uniform vec3 uColor;
        uniform float uAlpha;
        out vec4 fragColor;
        void main() {
            float a = texture(uFont, vUv).r;
            if (a < 0.5) discard;
            fragColor = vec4(uColor, uAlpha);
        }
        """;

    public void Init()
    {
        _shader = new Shader(Vs, Fs, "hud");
        _texture = GL.GenTexture();
        GL.BindTexture(GL.TEXTURE_2D, _texture);

        var pixels = new byte[AtlasW * AtlasH];
        for (int g = 0; g < Font8x8.GlyphCount; g++)
        {
            int cx = (g % AtlasCols) * 8;
            int cy = (g / AtlasCols) * 8;
            for (int row = 0; row < 8; row++)
            {
                byte bits = Font8x8.Data[g * 8 + row];
                for (int col = 0; col < 8; col++)
                    if (((bits >> col) & 1) != 0)          // LSB = leftmost pixel
                        pixels[(cy + row) * AtlasW + cx + col] = 255;
            }
        }
        GL.PixelStore(GL.UNPACK_ALIGNMENT, 1);
        GL.TexImage2D_R8(AtlasW, AtlasH, pixels);
        GL.TexParameter(GL.TEXTURE_2D, GL.TEXTURE_MIN_FILTER, (int)GL.NEAREST);
        GL.TexParameter(GL.TEXTURE_2D, GL.TEXTURE_MAG_FILTER, (int)GL.NEAREST);
        GL.TexParameter(GL.TEXTURE_2D, GL.TEXTURE_WRAP_S, (int)GL.CLAMP_TO_EDGE);
        GL.TexParameter(GL.TEXTURE_2D, GL.TEXTURE_WRAP_T, (int)GL.CLAMP_TO_EDGE);

        _vao = GL.GenVertexArray();
        _vbo = GL.GenBuffer();
        GL.BindVertexArray(_vao);
        GL.BindBuffer(GL.ARRAY_BUFFER, _vbo);
        GL.VertexAttribPointer(0, 2, GL.FLOAT, false, 4 * sizeof(float), 0);
        GL.VertexAttribPointer(1, 2, GL.FLOAT, false, 4 * sizeof(float), 2 * sizeof(float));
        GL.EnableVertexAttribArray(0);
        GL.EnableVertexAttribArray(1);
        GL.BindVertexArray(0);
    }

    public void Begin() { _batch.Clear(); _runs.Clear(); }

    public void Print(float x, float y, string text, Vector3 color, float scale = 2f, float alpha = 1f)
    {
        int start = _batch.Count / 4;
        float cw = 8 * scale;
        float cx = x;
        foreach (char ch in text)
        {
            int g = ch - Font8x8.FirstChar;
            if (g < 0 || g >= Font8x8.GlyphCount) { cx += cw; continue; }
            if (ch != ' ')
            {
                float u0 = (g % AtlasCols) * 8f / AtlasW;
                float v0 = (g / AtlasCols) * 8f / AtlasH;
                float u1 = u0 + 8f / AtlasW;
                float v1 = v0 + 8f / AtlasH;
                Quad(cx, y, cw, cw, u0, v0, u1, v1);
            }
            cx += cw;
        }
        int count = _batch.Count / 4 - start;
        if (count > 0) _runs.Add((start, count, color, alpha));
    }

    private void Quad(float x, float y, float w, float h, float u0, float v0, float u1, float v1)
    {
        // two triangles, 6 vertices of (x, y, u, v)
        _batch.AddRange([x, y, u0, v0, x, y + h, u0, v1, x + w, y + h, u1, v1]);
        _batch.AddRange([x, y, u0, v0, x + w, y + h, u1, v1, x + w, y, u1, v0]);
    }

    public void Flush(int screenW, int screenH)
    {
        if (_runs.Count == 0) return;
        float[] data = _batch.ToArray();

        GL.Disable(GL.DEPTH_TEST);
        GL.Enable(GL.BLEND);
        GL.BlendFunc(GL.SRC_ALPHA, GL.ONE_MINUS_SRC_ALPHA);

        _shader.Use();
        GL.Uniform3(_shader.U("uScreen"), screenW, screenH, 0); // z unused
        GL.Uniform1(_shader.U("uFont"), 0);
        GL.ActiveTexture(GL.TEXTURE0);
        GL.BindTexture(GL.TEXTURE_2D, _texture);

        GL.BindVertexArray(_vao);
        GL.BindBuffer(GL.ARRAY_BUFFER, _vbo);
        GL.BufferData(GL.ARRAY_BUFFER, data, GL.STREAM_DRAW);

        foreach (var (start, count, color, alpha) in _runs)
        {
            GL.Uniform3(_shader.U("uColor"), color);
            GL.Uniform1(_shader.U("uAlpha"), alpha);
            GL.DrawArrays(GL.TRIANGLES, start, count);
        }

        GL.BindVertexArray(0);
        GL.Disable(GL.BLEND);
        GL.Enable(GL.DEPTH_TEST);
    }
}
