using RodStudio.Sim;

namespace RodStudio;

internal static class Program
{
    [STAThread]
    private static int Main(string[] args)
    {
        if (args.Contains("--selftest"))
            return SelfTest.Run();

        if (!OperatingSystem.IsWindows())
        {
            Console.Error.WriteLine("The interactive viewer is Windows-only (Win32/WGL/OpenGL).");
            Console.Error.WriteLine("The solver itself is cross-platform: run `dotnet run -- --selftest`.");
            return 2;
        }

        return App.Run();
    }
}
