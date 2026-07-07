namespace RodStudio.Gfx;

internal sealed class Shader
{
    public uint Program { get; }
    private readonly Dictionary<string, int> _uniforms = new();

    public Shader(string vertexSrc, string fragmentSrc, string name)
    {
        uint vs = Compile(GL.VERTEX_SHADER, vertexSrc, name + ".vs");
        uint fs = Compile(GL.FRAGMENT_SHADER, fragmentSrc, name + ".fs");
        Program = GL.CreateProgram();
        GL.AttachShader(Program, vs);
        GL.AttachShader(Program, fs);
        GL.LinkProgram(Program);
        if (GL.GetProgrami(Program, GL.LINK_STATUS) == 0)
            throw new InvalidOperationException($"link {name}: {GL.GetProgramInfoLog(Program)}");
        GL.DeleteShader(vs);
        GL.DeleteShader(fs);
    }

    private static uint Compile(uint type, string src, string name)
    {
        uint s = GL.CreateShader(type);
        GL.ShaderSource(s, src);
        GL.CompileShader(s);
        if (GL.GetShaderi(s, GL.COMPILE_STATUS) == 0)
            throw new InvalidOperationException($"compile {name}: {GL.GetShaderInfoLog(s)}");
        return s;
    }

    public void Use() => GL.UseProgram(Program);

    public int U(string name)
    {
        if (!_uniforms.TryGetValue(name, out int loc))
            _uniforms[name] = loc = GL.GetUniformLocation(Program, name);
        return loc;
    }
}
