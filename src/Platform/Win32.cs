using System.Runtime.InteropServices;

namespace RodStudio.Platform;

internal static class Win32
{
    // ---- window styles / messages ----
    public const uint WS_OVERLAPPEDWINDOW = 0x00CF0000;
    public const uint WS_VISIBLE = 0x10000000;
    public const uint CS_OWNDC = 0x0020;
    public const uint CS_HREDRAW = 0x0002;
    public const uint CS_VREDRAW = 0x0001;

    public const uint WM_DESTROY = 0x0002;
    public const uint WM_SIZE = 0x0005;
    public const uint WM_CLOSE = 0x0010;
    public const uint WM_KEYDOWN = 0x0100;
    public const uint WM_KEYUP = 0x0101;
    public const uint WM_SYSKEYDOWN = 0x0104;
    public const uint WM_MOUSEMOVE = 0x0200;
    public const uint WM_LBUTTONDOWN = 0x0201;
    public const uint WM_LBUTTONUP = 0x0202;
    public const uint WM_RBUTTONDOWN = 0x0204;
    public const uint WM_RBUTTONUP = 0x0205;
    public const uint WM_MBUTTONDOWN = 0x0207;
    public const uint WM_MBUTTONUP = 0x0208;
    public const uint WM_MOUSEWHEEL = 0x020A;
    public const uint PM_REMOVE = 0x0001;
    public const int GWLP_USERDATA = -21;
    public const int SW_SHOW = 5;
    public const nint IDC_ARROW = 32512;

    // ---- virtual keys ----
    public const int VK_ESCAPE = 0x1B;
    public const int VK_SPACE = 0x20;
    public const int VK_LEFT = 0x25;
    public const int VK_UP = 0x26;
    public const int VK_RIGHT = 0x27;
    public const int VK_DOWN = 0x28;

    [UnmanagedFunctionPointer(CallingConvention.Winapi)]
    public delegate nint WndProcDelegate(nint hWnd, uint msg, nint wParam, nint lParam);

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
    public struct WNDCLASSEXW
    {
        public uint cbSize;
        public uint style;
        public nint lpfnWndProc; // marshalled function pointer
        public int cbClsExtra;
        public int cbWndExtra;
        public nint hInstance;
        public nint hIcon;
        public nint hCursor;
        public nint hbrBackground;
        public nint lpszMenuName;
        public nint lpszClassName;
        public nint hIconSm;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct MSG
    {
        public nint hwnd;
        public uint message;
        public nint wParam;
        public nint lParam;
        public uint time;
        public int ptX, ptY;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct RECT { public int Left, Top, Right, Bottom; }

    [StructLayout(LayoutKind.Sequential)]
    public struct PIXELFORMATDESCRIPTOR
    {
        public ushort nSize, nVersion;
        public uint dwFlags;
        public byte iPixelType, cColorBits, cRedBits, cRedShift, cGreenBits, cGreenShift,
                    cBlueBits, cBlueShift, cAlphaBits, cAlphaShift, cAccumBits,
                    cAccumRedBits, cAccumGreenBits, cAccumBlueBits, cAccumAlphaBits,
                    cDepthBits, cStencilBits, cAuxBuffers, iLayerType, bReserved;
        public uint dwLayerMask, dwVisibleMask, dwDamageMask;
    }

    public const uint PFD_DRAW_TO_WINDOW = 0x00000004;
    public const uint PFD_SUPPORT_OPENGL = 0x00000020;
    public const uint PFD_DOUBLEBUFFER = 0x00000001;
    public const byte PFD_TYPE_RGBA = 0;

    // ---- user32 ----
    [DllImport("user32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
    public static extern ushort RegisterClassExW(ref WNDCLASSEXW wc);

    [DllImport("user32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
    public static extern nint CreateWindowExW(uint exStyle, string className, string windowName,
        uint style, int x, int y, int width, int height,
        nint parent, nint menu, nint instance, nint param);

    [DllImport("user32.dll")] public static extern nint DefWindowProcW(nint hWnd, uint msg, nint wParam, nint lParam);
    [DllImport("user32.dll")] public static extern bool DestroyWindow(nint hWnd);
    [DllImport("user32.dll")] public static extern bool ShowWindow(nint hWnd, int cmd);
    [DllImport("user32.dll")] public static extern bool PeekMessageW(out MSG msg, nint hWnd, uint min, uint max, uint remove);
    [DllImport("user32.dll")] public static extern bool TranslateMessage(ref MSG msg);
    [DllImport("user32.dll")] public static extern nint DispatchMessageW(ref MSG msg);
    [DllImport("user32.dll")] public static extern void PostQuitMessage(int code);
    [DllImport("user32.dll", CharSet = CharSet.Unicode)] public static extern nint LoadCursorW(nint instance, nint cursorName);
    [DllImport("user32.dll")] public static extern nint GetDC(nint hWnd);
    [DllImport("user32.dll")] public static extern int ReleaseDC(nint hWnd, nint hDC);
    [DllImport("user32.dll")] public static extern bool GetClientRect(nint hWnd, out RECT rect);
    [DllImport("user32.dll")] public static extern bool AdjustWindowRect(ref RECT rect, uint style, bool menu);
    [DllImport("user32.dll", CharSet = CharSet.Unicode)] public static extern bool SetWindowTextW(nint hWnd, string text);
    [DllImport("user32.dll")] public static extern nint SetCapture(nint hWnd);
    [DllImport("user32.dll")] public static extern bool ReleaseCapture();
    [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();

    // ---- kernel32 ----
    [DllImport("kernel32.dll", CharSet = CharSet.Unicode)] public static extern nint GetModuleHandleW(string? name);
    [DllImport("kernel32.dll", CharSet = CharSet.Ansi, SetLastError = true)]
    public static extern nint GetProcAddress(nint module, string name);
    [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
    public static extern nint LoadLibraryW(string name);

    // ---- gdi32 ----
    [DllImport("gdi32.dll")] public static extern int ChoosePixelFormat(nint hdc, ref PIXELFORMATDESCRIPTOR pfd);
    [DllImport("gdi32.dll")] public static extern bool SetPixelFormat(nint hdc, int format, ref PIXELFORMATDESCRIPTOR pfd);
    [DllImport("gdi32.dll")] public static extern int DescribePixelFormat(nint hdc, int format, uint bytes, ref PIXELFORMATDESCRIPTOR pfd);
    [DllImport("gdi32.dll")] public static extern bool SwapBuffers(nint hdc);

    // ---- opengl32 (WGL core) ----
    [DllImport("opengl32.dll")] public static extern nint wglCreateContext(nint hdc);
    [DllImport("opengl32.dll")] public static extern bool wglMakeCurrent(nint hdc, nint hglrc);
    [DllImport("opengl32.dll")] public static extern bool wglDeleteContext(nint hglrc);
    [DllImport("opengl32.dll", CharSet = CharSet.Ansi)] public static extern nint wglGetProcAddress(string name);
}
