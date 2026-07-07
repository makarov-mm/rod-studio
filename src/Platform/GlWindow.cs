using System.Runtime.InteropServices;

namespace RodStudio.Platform;

/// <summary>
/// Win32 window with a modern OpenGL core-profile context, created the canonical way:
/// a throwaway window/context first (to reach wglChoosePixelFormatARB and
/// wglCreateContextAttribsARB, since SetPixelFormat is once-per-window), then the real
/// window with a multisampled pixel format and a 4.5 core context (fallbacks: 4.1, 3.3).
/// </summary>
internal sealed class GlWindow : IDisposable
{
    public nint Hwnd { get; private set; }
    public nint Hdc { get; private set; }
    public nint Hglrc { get; private set; }
    public int Width { get; private set; }
    public int Height { get; private set; }
    public bool CloseRequested { get; private set; }
    public (int Major, int Minor) ContextVersion { get; private set; }

    // ---- input state, updated by the message pump ----
    public readonly bool[] KeyDown = new bool[256];
    public readonly Queue<int> KeyPresses = new(); // WM_KEYDOWN without autorepeat
    public bool MouseLeft, MouseRight, MouseMiddle;
    public int MouseX, MouseY, MouseDx, MouseDy;
    public float WheelDelta;

    private Win32.WndProcDelegate? _wndProc; // rooted: the OS holds a raw pointer to this
    private const string ClassName = "RodStudioWindow";

    // WGL_ARB constants
    private const int WGL_DRAW_TO_WINDOW_ARB = 0x2001, WGL_SUPPORT_OPENGL_ARB = 0x2010,
        WGL_DOUBLE_BUFFER_ARB = 0x2011, WGL_PIXEL_TYPE_ARB = 0x2013, WGL_TYPE_RGBA_ARB = 0x202B,
        WGL_COLOR_BITS_ARB = 0x2014, WGL_DEPTH_BITS_ARB = 0x2022, WGL_STENCIL_BITS_ARB = 0x2023,
        WGL_SAMPLE_BUFFERS_ARB = 0x2041, WGL_SAMPLES_ARB = 0x2042,
        WGL_CONTEXT_MAJOR_VERSION_ARB = 0x2091, WGL_CONTEXT_MINOR_VERSION_ARB = 0x2092,
        WGL_CONTEXT_PROFILE_MASK_ARB = 0x9126, WGL_CONTEXT_CORE_PROFILE_BIT_ARB = 0x0001;

    private delegate bool WglChoosePixelFormatArb(nint hdc, int[] iAttribs, float[]? fAttribs,
        uint maxFormats, int[] formats, out uint numFormats);
    private delegate nint WglCreateContextAttribsArb(nint hdc, nint shareContext, int[] attribs);
    private delegate bool WglSwapIntervalExt(int interval);

    private WglChoosePixelFormatArb? _choosePixelFormat;
    private WglCreateContextAttribsArb? _createContextAttribs;
    private WglSwapIntervalExt? _swapInterval;

    public static GlWindow Create(string title, int clientWidth, int clientHeight)
    {
        Win32.SetProcessDPIAware();
        var w = new GlWindow();
        w.LoadWglExtensionsViaDummy();
        w.CreateRealWindow(title, clientWidth, clientHeight);
        return w;
    }

    // ------------------------------------------------------------ dummy phase

    private void LoadWglExtensionsViaDummy()
    {
        nint instance = Win32.GetModuleHandleW(null);
        _wndProc = StaticWndProcHolder.Get(this);
        var wc = new Win32.WNDCLASSEXW
        {
            cbSize = (uint)Marshal.SizeOf<Win32.WNDCLASSEXW>(),
            style = Win32.CS_OWNDC | Win32.CS_HREDRAW | Win32.CS_VREDRAW,
            lpfnWndProc = Marshal.GetFunctionPointerForDelegate(_wndProc),
            hInstance = instance,
            hCursor = Win32.LoadCursorW(0, Win32.IDC_ARROW),
            lpszClassName = Marshal.StringToHGlobalUni(ClassName),
        };
        if (Win32.RegisterClassExW(ref wc) == 0)
            throw new InvalidOperationException($"RegisterClassExW failed: {Marshal.GetLastWin32Error()}");

        nint dummy = Win32.CreateWindowExW(0, ClassName, "dummy", 0,
            0, 0, 32, 32, 0, 0, instance, 0);
        nint dc = Win32.GetDC(dummy);

        var pfd = new Win32.PIXELFORMATDESCRIPTOR
        {
            nSize = (ushort)Marshal.SizeOf<Win32.PIXELFORMATDESCRIPTOR>(),
            nVersion = 1,
            dwFlags = Win32.PFD_DRAW_TO_WINDOW | Win32.PFD_SUPPORT_OPENGL | Win32.PFD_DOUBLEBUFFER,
            iPixelType = Win32.PFD_TYPE_RGBA,
            cColorBits = 32,
            cDepthBits = 24,
            cStencilBits = 8,
        };
        int fmt = Win32.ChoosePixelFormat(dc, ref pfd);
        Win32.SetPixelFormat(dc, fmt, ref pfd);
        nint rc = Win32.wglCreateContext(dc);
        Win32.wglMakeCurrent(dc, rc);

        _choosePixelFormat = LoadWgl<WglChoosePixelFormatArb>("wglChoosePixelFormatARB");
        _createContextAttribs = LoadWgl<WglCreateContextAttribsArb>("wglCreateContextAttribsARB");
        _swapInterval = LoadWgl<WglSwapIntervalExt>("wglSwapIntervalEXT");

        Win32.wglMakeCurrent(0, 0);
        Win32.wglDeleteContext(rc);
        Win32.ReleaseDC(dummy, dc);
        Win32.DestroyWindow(dummy);
    }

    private static T? LoadWgl<T>(string name) where T : Delegate
    {
        nint p = Win32.wglGetProcAddress(name);
        return p is 0 or 1 or 2 or 3 or -1 ? null : Marshal.GetDelegateForFunctionPointer<T>(p);
    }

    // ------------------------------------------------------------- real phase

    private void CreateRealWindow(string title, int clientWidth, int clientHeight)
    {
        nint instance = Win32.GetModuleHandleW(null);
        var rect = new Win32.RECT { Left = 0, Top = 0, Right = clientWidth, Bottom = clientHeight };
        Win32.AdjustWindowRect(ref rect, Win32.WS_OVERLAPPEDWINDOW, false);

        Hwnd = Win32.CreateWindowExW(0, ClassName, title,
            Win32.WS_OVERLAPPEDWINDOW, 80, 60,
            rect.Right - rect.Left, rect.Bottom - rect.Top, 0, 0, instance, 0);
        if (Hwnd == 0) throw new InvalidOperationException("CreateWindowExW failed");
        Hdc = Win32.GetDC(Hwnd);

        int format = 0;
        if (_choosePixelFormat != null)
        {
            foreach (int samples in new[] { 8, 4, 0 })
            {
                var attribs = new List<int>
                {
                    WGL_DRAW_TO_WINDOW_ARB, 1,
                    WGL_SUPPORT_OPENGL_ARB, 1,
                    WGL_DOUBLE_BUFFER_ARB, 1,
                    WGL_PIXEL_TYPE_ARB, WGL_TYPE_RGBA_ARB,
                    WGL_COLOR_BITS_ARB, 32,
                    WGL_DEPTH_BITS_ARB, 24,
                    WGL_STENCIL_BITS_ARB, 8,
                };
                if (samples > 0)
                {
                    attribs.AddRange(new[] { WGL_SAMPLE_BUFFERS_ARB, 1, WGL_SAMPLES_ARB, samples });
                }
                attribs.Add(0);
                var fmts = new int[1];
                if (_choosePixelFormat(Hdc, attribs.ToArray(), null, 1, fmts, out uint num) && num > 0)
                {
                    format = fmts[0];
                    break;
                }
            }
        }
        var pfd = new Win32.PIXELFORMATDESCRIPTOR();
        pfd.nSize = (ushort)Marshal.SizeOf<Win32.PIXELFORMATDESCRIPTOR>();
        if (format == 0)
        {
            pfd.nVersion = 1;
            pfd.dwFlags = Win32.PFD_DRAW_TO_WINDOW | Win32.PFD_SUPPORT_OPENGL | Win32.PFD_DOUBLEBUFFER;
            pfd.iPixelType = Win32.PFD_TYPE_RGBA;
            pfd.cColorBits = 32; pfd.cDepthBits = 24; pfd.cStencilBits = 8;
            format = Win32.ChoosePixelFormat(Hdc, ref pfd);
        }
        else
        {
            Win32.DescribePixelFormat(Hdc, format, (uint)Marshal.SizeOf<Win32.PIXELFORMATDESCRIPTOR>(), ref pfd);
        }
        if (!Win32.SetPixelFormat(Hdc, format, ref pfd))
            throw new InvalidOperationException("SetPixelFormat failed");

        if (_createContextAttribs != null)
        {
            foreach (var (major, minor) in new[] { (4, 5), (4, 1), (3, 3) })
            {
                Hglrc = _createContextAttribs(Hdc, 0, new[]
                {
                    WGL_CONTEXT_MAJOR_VERSION_ARB, major,
                    WGL_CONTEXT_MINOR_VERSION_ARB, minor,
                    WGL_CONTEXT_PROFILE_MASK_ARB, WGL_CONTEXT_CORE_PROFILE_BIT_ARB,
                    0
                });
                if (Hglrc != 0) { ContextVersion = (major, minor); break; }
            }
        }
        if (Hglrc == 0)
        {
            Hglrc = Win32.wglCreateContext(Hdc); // last-resort legacy context
            ContextVersion = (2, 1);
        }
        Win32.wglMakeCurrent(Hdc, Hglrc);

        Win32.GetClientRect(Hwnd, out var cr);
        Width = cr.Right - cr.Left;
        Height = cr.Bottom - cr.Top;
        Win32.ShowWindow(Hwnd, Win32.SW_SHOW);
    }

    public void SetVSync(bool on) => _swapInterval?.Invoke(on ? 1 : 0);
    public void SetTitle(string t) => Win32.SetWindowTextW(Hwnd, t);
    public void Swap() => Win32.SwapBuffers(Hdc);

    // ------------------------------------------------------------ message pump

    public void PumpMessages()
    {
        MouseDx = 0; MouseDy = 0; WheelDelta = 0;
        while (Win32.PeekMessageW(out var msg, 0, 0, 0, Win32.PM_REMOVE))
        {
            if (msg.message == 0x0012) { CloseRequested = true; continue; } // WM_QUIT
            Win32.TranslateMessage(ref msg);
            Win32.DispatchMessageW(ref msg);
        }
    }

    private nint WndProc(nint hWnd, uint msg, nint wParam, nint lParam)
    {
        switch (msg)
        {
            case Win32.WM_CLOSE:
            case Win32.WM_DESTROY:
                // the WGL dummy window shares this WndProc; its destruction during
                // extension loading must not close the app (Hwnd is still 0 then)
                if (hWnd == Hwnd) CloseRequested = true;
                return 0;
            case Win32.WM_SIZE:
                Width = (int)((long)lParam & 0xFFFF);
                Height = (int)(((long)lParam >> 16) & 0xFFFF);
                return 0;
            case Win32.WM_KEYDOWN:
            case Win32.WM_SYSKEYDOWN:
            {
                int vk = (int)wParam & 0xFF;
                bool wasDown = (((long)lParam >> 30) & 1) != 0;
                KeyDown[vk] = true;
                if (!wasDown) KeyPresses.Enqueue(vk);
                return 0;
            }
            case Win32.WM_KEYUP:
                KeyDown[(int)wParam & 0xFF] = false;
                return 0;
            case Win32.WM_LBUTTONDOWN: MouseLeft = true; Win32.SetCapture(hWnd); UpdateMouse(lParam, false); return 0;
            case Win32.WM_LBUTTONUP: MouseLeft = false; Win32.ReleaseCapture(); return 0;
            case Win32.WM_RBUTTONDOWN: MouseRight = true; Win32.SetCapture(hWnd); UpdateMouse(lParam, false); return 0;
            case Win32.WM_RBUTTONUP: MouseRight = false; Win32.ReleaseCapture(); return 0;
            case Win32.WM_MBUTTONDOWN: MouseMiddle = true; Win32.SetCapture(hWnd); UpdateMouse(lParam, false); return 0;
            case Win32.WM_MBUTTONUP: MouseMiddle = false; Win32.ReleaseCapture(); return 0;
            case Win32.WM_MOUSEMOVE: UpdateMouse(lParam, true); return 0;
            case Win32.WM_MOUSEWHEEL:
                WheelDelta += (short)(((long)wParam >> 16) & 0xFFFF) / 120f;
                return 0;
        }
        return Win32.DefWindowProcW(hWnd, msg, wParam, lParam);
    }

    private void UpdateMouse(nint lParam, bool accumulateDelta)
    {
        int x = (short)((long)lParam & 0xFFFF);
        int y = (short)(((long)lParam >> 16) & 0xFFFF);
        if (accumulateDelta)
        {
            MouseDx += x - MouseX;
            MouseDy += y - MouseY;
        }
        MouseX = x; MouseY = y;
    }

    public void Dispose()
    {
        if (Hglrc != 0) { Win32.wglMakeCurrent(0, 0); Win32.wglDeleteContext(Hglrc); Hglrc = 0; }
        if (Hdc != 0) { Win32.ReleaseDC(Hwnd, Hdc); Hdc = 0; }
        if (Hwnd != 0) { Win32.DestroyWindow(Hwnd); Hwnd = 0; }
    }

    /// <summary>Routes the static WndProc callback to the single window instance.</summary>
    private static class StaticWndProcHolder
    {
        private static GlWindow? _instance;
        public static Win32.WndProcDelegate Get(GlWindow w)
        {
            _instance = w;
            return Callback;
        }
        private static nint Callback(nint hWnd, uint msg, nint wParam, nint lParam) =>
            _instance?.WndProc(hWnd, msg, wParam, lParam) ?? Win32.DefWindowProcW(hWnd, msg, wParam, lParam);
    }
}
