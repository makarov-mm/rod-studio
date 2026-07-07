using System.Runtime.InteropServices;
using RodStudio.Platform;

namespace RodStudio.Gfx;

/// <summary>
/// Minimal OpenGL 3.3/4.5-core binding, hand-rolled: every entry point is resolved at
/// runtime through wglGetProcAddress (modern) or opengl32.dll exports (GL 1.1 legacy).
/// No external binding library.
/// </summary>
internal static unsafe class GL
{
    // ---- enums (only what we use) ----
    public const uint COLOR_BUFFER_BIT = 0x4000, DEPTH_BUFFER_BIT = 0x0100, STENCIL_BUFFER_BIT = 0x0400;
    public const uint DEPTH_TEST = 0x0B71, CULL_FACE = 0x0B44, BLEND = 0x0BE2, MULTISAMPLE = 0x809D;
    public const uint STENCIL_TEST = 0x0B90, POLYGON_OFFSET_FILL = 0x8037, LINE_SMOOTH = 0x0B20;
    public const uint LEQUAL = 0x0203, LESS = 0x0201, ALWAYS = 0x0207, EQUAL = 0x0202;
    public const uint KEEP = 0x1E00, INCR = 0x1E02, REPLACE = 0x1E01;
    public const uint SRC_ALPHA = 0x0302, ONE_MINUS_SRC_ALPHA = 0x0303;
    public const uint BACK = 0x0405, FRONT_AND_BACK = 0x0408, FILL = 0x1B02, LINE = 0x1B01, CCW = 0x0901;
    public const uint TRIANGLES = 0x0004, UNSIGNED_INT = 0x1405, UNSIGNED_BYTE = 0x1401, FLOAT = 0x1406;
    public const uint ARRAY_BUFFER = 0x8892, ELEMENT_ARRAY_BUFFER = 0x8893;
    public const uint STATIC_DRAW = 0x88E4, DYNAMIC_DRAW = 0x88E8, STREAM_DRAW = 0x88E0;
    public const uint VERTEX_SHADER = 0x8B31, FRAGMENT_SHADER = 0x8B30;
    public const uint COMPILE_STATUS = 0x8B81, LINK_STATUS = 0x8B82, INFO_LOG_LENGTH = 0x8B84;
    public const uint TEXTURE_2D = 0x0DE1, TEXTURE0 = 0x84C0;
    public const uint TEXTURE_MIN_FILTER = 0x2801, TEXTURE_MAG_FILTER = 0x2800;
    public const uint TEXTURE_WRAP_S = 0x2802, TEXTURE_WRAP_T = 0x2803;
    public const uint NEAREST = 0x2600, LINEAR = 0x2601, CLAMP_TO_EDGE = 0x812F, REPEAT = 0x2901;
    public const uint R8 = 0x8229, RED = 0x1903;
    public const uint UNPACK_ALIGNMENT = 0x0CF5;
    public const uint VERSION = 0x1F02, RENDERER = 0x1F01;

    // ---- delegate types ----
    private delegate void D_v();
    private delegate void D_ui(uint a);
    private delegate void D_f(float a);
    private delegate void D_ui_ui(uint a, uint b);
    private delegate void D_ui_i(uint a, int b);
    private delegate void D_i_i(int a, int b);
    private delegate uint D_ret_ui(uint a);
    private delegate uint D_ret_v();
    private delegate void D_i4(int a, int b, int c, int d);
    private delegate void D_f4(float r, float g, float b, float a);
    private delegate void D_ui3(uint a, uint b, uint c);
    private delegate void D_b4(byte r, byte g, byte b, byte a);
    private delegate void D_ui_i_ui(uint a, int b, uint c);

    private delegate void D_genBuffers(int n, uint* ids);
    private delegate void D_bufferData(uint target, nint size, void* data, uint usage);
    private delegate void D_bufferSubData(uint target, nint offset, nint size, void* data);
    private delegate void D_vertexAttribPointer(uint index, int size, uint type, byte normalized, int stride, nint offset);
    private delegate void D_drawElements(uint mode, int count, uint type, nint offset);
    private delegate void D_drawArrays(uint mode, int first, int count);
    private delegate void D_shaderSource(uint shader, int count, byte** src, int* length);
    private delegate void D_getShaderiv(uint shader, uint pname, int* p);
    private delegate void D_getInfoLog(uint obj, int maxLen, int* len, byte* log);
    private delegate int D_getUniformLocation(uint program, byte* name);
    private delegate void D_uniformMatrix4fv(int loc, int count, byte transpose, float* value);
    private delegate void D_uniform1f(int loc, float v);
    private delegate void D_uniform1i(int loc, int v);
    private delegate void D_uniform3f(int loc, float x, float y, float z);
    private delegate void D_uniform4f(int loc, float x, float y, float z, float w);
    private delegate void D_texImage2D(uint target, int level, int internalFormat, int w, int h, int border, uint format, uint type, void* data);
    private delegate void D_texParameteri(uint target, uint pname, int param);
    private delegate void D_pixelStorei(uint pname, int param);
    private delegate void D_stencilFunc(uint func, int refv, uint mask);
    private delegate nint D_getString(uint name);

    // ---- function slots ----
    private static D_f4 _clearColor = null!;
    private static D_ui _clear = null!, _enable = null!, _disable = null!, _depthFunc = null!,
                        _cullFace = null!, _frontFace = null!, _useProgram = null!,
                        _compileShader = null!, _linkProgram = null!, _deleteShader = null!,
                        _bindVertexArray = null!, _enableVertexAttribArray = null!,
                        _activeTexture = null!, _depthMaskB = null!;
    private static D_i4 _viewport = null!;
    private static D_ui_ui _blendFunc = null!, _bindTexture2 = null!, _attachShader = null!,
                           _polygonMode = null!, _bindBuffer2 = null!;
    private static D_ret_ui _createShader = null!;
    private static D_ret_v _createProgram = null!;
    private static D_genBuffers _genBuffers = null!, _genVertexArrays = null!, _genTextures = null!;
    private static D_bufferData _bufferData = null!;
    private static D_bufferSubData _bufferSubData = null!;
    private static D_vertexAttribPointer _vertexAttribPointer = null!;
    private static D_drawElements _drawElements = null!;
    private static D_drawArrays _drawArrays = null!;
    private static D_shaderSource _shaderSource = null!;
    private static D_getShaderiv _getShaderiv = null!, _getProgramiv = null!;
    private static D_getInfoLog _getShaderInfoLog = null!, _getProgramInfoLog = null!;
    private static D_getUniformLocation _getUniformLocation = null!;
    private static D_uniformMatrix4fv _uniformMatrix4fv = null!;
    private static D_uniform1f _uniform1f = null!;
    private static D_uniform1i _uniform1i = null!;
    private static D_uniform3f _uniform3f = null!;
    private static D_uniform4f _uniform4f = null!;
    private static D_texImage2D _texImage2D = null!;
    private static D_texParameteri _texParameteri = null!;
    private static D_pixelStorei _pixelStorei = null!;
    private static D_stencilFunc _stencilFunc = null!;
    private static D_ui3 _stencilOp = null!;
    private static D_b4 _colorMask = null!;
    private static D_f _lineWidth = null!;
    private static D_getString _getString = null!;

    private static readonly List<Delegate> _keepAlive = new(); // prevent GC of delegates
    private static nint _opengl32;

    public static void LoadFunctions()
    {
        _opengl32 = Win32.LoadLibraryW("opengl32.dll");

        _clearColor = Load<D_f4>("glClearColor");
        _clear = Load<D_ui>("glClear");
        _enable = Load<D_ui>("glEnable");
        _disable = Load<D_ui>("glDisable");
        _depthFunc = Load<D_ui>("glDepthFunc");
        _depthMaskB = Load<D_ui>("glDepthMask");
        _cullFace = Load<D_ui>("glCullFace");
        _frontFace = Load<D_ui>("glFrontFace");
        _viewport = Load<D_i4>("glViewport");
        _blendFunc = Load<D_ui_ui>("glBlendFunc");
        _polygonMode = Load<D_ui_ui>("glPolygonMode");
        _lineWidth = Load<D_f>("glLineWidth");
        _stencilFunc = Load<D_stencilFunc>("glStencilFunc");
        _stencilOp = Load<D_ui3>("glStencilOp");
        _colorMask = Load<D_b4>("glColorMask");
        _drawArrays = Load<D_drawArrays>("glDrawArrays");
        _drawElements = Load<D_drawElements>("glDrawElements");
        _pixelStorei = Load<D_pixelStorei>("glPixelStorei");
        _genTextures = Load<D_genBuffers>("glGenTextures");
        _bindTexture2 = Load<D_ui_ui>("glBindTexture");
        _texImage2D = Load<D_texImage2D>("glTexImage2D");
        _texParameteri = Load<D_texParameteri>("glTexParameteri");
        _getString = Load<D_getString>("glGetString");

        _createShader = Load<D_ret_ui>("glCreateShader");
        _shaderSource = Load<D_shaderSource>("glShaderSource");
        _compileShader = Load<D_ui>("glCompileShader");
        _getShaderiv = Load<D_getShaderiv>("glGetShaderiv");
        _getShaderInfoLog = Load<D_getInfoLog>("glGetShaderInfoLog");
        _createProgram = Load<D_ret_v>("glCreateProgram");
        _attachShader = Load<D_ui_ui>("glAttachShader");
        _linkProgram = Load<D_ui>("glLinkProgram");
        _getProgramiv = Load<D_getShaderiv>("glGetProgramiv");
        _getProgramInfoLog = Load<D_getInfoLog>("glGetProgramInfoLog");
        _deleteShader = Load<D_ui>("glDeleteShader");
        _useProgram = Load<D_ui>("glUseProgram");
        _getUniformLocation = Load<D_getUniformLocation>("glGetUniformLocation");
        _uniformMatrix4fv = Load<D_uniformMatrix4fv>("glUniformMatrix4fv");
        _uniform1f = Load<D_uniform1f>("glUniform1f");
        _uniform1i = Load<D_uniform1i>("glUniform1i");
        _uniform3f = Load<D_uniform3f>("glUniform3f");
        _uniform4f = Load<D_uniform4f>("glUniform4f");
        _genVertexArrays = Load<D_genBuffers>("glGenVertexArrays");
        _bindVertexArray = Load<D_ui>("glBindVertexArray");
        _genBuffers = Load<D_genBuffers>("glGenBuffers");
        _bindBuffer2 = Load<D_ui_ui>("glBindBuffer");
        _bufferData = Load<D_bufferData>("glBufferData");
        _bufferSubData = Load<D_bufferSubData>("glBufferSubData");
        _vertexAttribPointer = Load<D_vertexAttribPointer>("glVertexAttribPointer");
        _enableVertexAttribArray = Load<D_ui>("glEnableVertexAttribArray");
        _activeTexture = Load<D_ui>("glActiveTexture");
    }

    private static T Load<T>(string name) where T : Delegate
    {
        nint p = Win32.wglGetProcAddress(name);
        // wglGetProcAddress can return 0, 1, 2, 3, -1 on failure
        if (p == 0 || p == 1 || p == 2 || p == 3 || p == -1)
            p = Win32.GetProcAddress(_opengl32, name);
        if (p == 0)
            throw new InvalidOperationException($"OpenGL function not found: {name}");
        var d = Marshal.GetDelegateForFunctionPointer<T>(p);
        _keepAlive.Add(d);
        return d;
    }

    // ---- public wrappers ----
    public static void ClearColor(float r, float g, float b, float a) => _clearColor(r, g, b, a);
    public static void Clear(uint mask) => _clear(mask);
    public static void Enable(uint cap) => _enable(cap);
    public static void Disable(uint cap) => _disable(cap);
    public static void DepthFunc(uint f) => _depthFunc(f);
    public static void DepthMask(bool on) => _depthMaskB(on ? 1u : 0u);
    public static void CullFace(uint f) => _cullFace(f);
    public static void FrontFace(uint f) => _frontFace(f);
    public static void Viewport(int x, int y, int w, int h) => _viewport(x, y, w, h);
    public static void BlendFunc(uint s, uint d) => _blendFunc(s, d);
    public static void PolygonMode(uint face, uint mode) => _polygonMode(face, mode);
    public static void LineWidth(float w) => _lineWidth(w);
    public static void StencilFunc(uint func, int r, uint mask) => _stencilFunc(func, r, mask);
    public static void StencilOp(uint fail, uint zfail, uint zpass) => _stencilOp(fail, zfail, zpass);
    public static void ColorMask(bool r, bool g, bool b, bool a) =>
        _colorMask((byte)(r ? 1 : 0), (byte)(g ? 1 : 0), (byte)(b ? 1 : 0), (byte)(a ? 1 : 0));
    public static void DrawArrays(uint mode, int first, int count) => _drawArrays(mode, first, count);
    public static void DrawElements(uint mode, int count, uint type, nint offset) => _drawElements(mode, count, type, offset);

    public static string GetString(uint name) => Marshal.PtrToStringAnsi(_getString(name)) ?? "";

    public static uint GenVertexArray() { uint id; _genVertexArrays(1, &id); return id; }
    public static void BindVertexArray(uint vao) => _bindVertexArray(vao);
    public static uint GenBuffer() { uint id; _genBuffers(1, &id); return id; }
    public static void BindBuffer(uint target, uint buf) => _bindBuffer2(target, buf);

    public static void BufferData(uint target, float[] data, uint usage)
    {
        fixed (float* p = data) _bufferData(target, data.Length * sizeof(float), p, usage);
    }
    public static void BufferData(uint target, uint[] data, uint usage)
    {
        fixed (uint* p = data) _bufferData(target, data.Length * sizeof(uint), p, usage);
    }
    public static void BufferDataEmpty(uint target, nint bytes, uint usage) => _bufferData(target, bytes, null, usage);
    public static void BufferSubData(uint target, nint offset, float[] data, int count)
    {
        fixed (float* p = data) _bufferSubData(target, offset, count * sizeof(float), p);
    }

    public static void VertexAttribPointer(uint index, int size, uint type, bool norm, int stride, nint offset) =>
        _vertexAttribPointer(index, size, type, (byte)(norm ? 1 : 0), stride, offset);
    public static void EnableVertexAttribArray(uint index) => _enableVertexAttribArray(index);

    public static uint CreateShader(uint type) => _createShader(type);
    public static void ShaderSource(uint shader, string src)
    {
        byte[] bytes = System.Text.Encoding.ASCII.GetBytes(src);
        fixed (byte* p = bytes)
        {
            byte* pp = p;
            int len = bytes.Length;
            _shaderSource(shader, 1, &pp, &len);
        }
    }
    public static void CompileShader(uint s) => _compileShader(s);
    public static int GetShaderi(uint s, uint pname) { int v; _getShaderiv(s, pname, &v); return v; }
    public static string GetShaderInfoLog(uint s)
    {
        int len = GetShaderi(s, INFO_LOG_LENGTH);
        if (len <= 1) return "";
        byte[] buf = new byte[len];
        fixed (byte* p = buf) { int outLen; _getShaderInfoLog(s, len, &outLen, p); }
        return System.Text.Encoding.ASCII.GetString(buf).TrimEnd('\0');
    }
    public static uint CreateProgram() => _createProgram();
    public static void AttachShader(uint prog, uint shader) => _attachShader(prog, shader);
    public static void LinkProgram(uint prog) => _linkProgram(prog);
    public static int GetProgrami(uint p, uint pname) { int v; _getProgramiv(p, pname, &v); return v; }
    public static string GetProgramInfoLog(uint prog)
    {
        int len = GetProgrami(prog, INFO_LOG_LENGTH);
        if (len <= 1) return "";
        byte[] buf = new byte[len];
        fixed (byte* p = buf) { int outLen; _getProgramInfoLog(prog, len, &outLen, p); }
        return System.Text.Encoding.ASCII.GetString(buf).TrimEnd('\0');
    }
    public static void DeleteShader(uint s) => _deleteShader(s);
    public static void UseProgram(uint p) => _useProgram(p);
    public static int GetUniformLocation(uint prog, string name)
    {
        byte[] bytes = System.Text.Encoding.ASCII.GetBytes(name + "\0");
        fixed (byte* p = bytes) return _getUniformLocation(prog, p);
    }
    public static void UniformMatrix4(int loc, System.Numerics.Matrix4x4 m)
    {
        // System.Numerics stores row-major with row-vector convention (v * M);
        // uploading without transpose makes the GLSL mat4 equal M^T, so
        // `uM * vec4(v,1)` in the shader computes exactly v * M. No transpose needed.
        float* p = (float*)&m;
        _uniformMatrix4fv(loc, 1, 0, p);
    }
    public static void Uniform1(int loc, float v) => _uniform1f(loc, v);
    public static void Uniform1(int loc, int v) => _uniform1i(loc, v);
    public static void Uniform3(int loc, float x, float y, float z) => _uniform3f(loc, x, y, z);
    public static void Uniform3(int loc, System.Numerics.Vector3 v) => _uniform3f(loc, v.X, v.Y, v.Z);
    public static void Uniform4(int loc, float x, float y, float z, float w) => _uniform4f(loc, x, y, z, w);

    public static uint GenTexture() { uint id; _genTextures(1, &id); return id; }
    public static void BindTexture(uint target, uint tex) => _bindTexture2(target, tex);
    public static void ActiveTexture(uint unit) => _activeTexture(unit);
    public static void PixelStore(uint pname, int v) => _pixelStorei(pname, v);
    public static void TexImage2D_R8(int w, int h, byte[] data)
    {
        fixed (byte* p = data) _texImage2D(TEXTURE_2D, 0, (int)R8, w, h, 0, RED, UNSIGNED_BYTE, p);
    }
    public static void TexParameter(uint target, uint pname, int v) => _texParameteri(target, pname, v);
}
