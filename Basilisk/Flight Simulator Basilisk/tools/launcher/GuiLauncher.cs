// Double-click launcher for a windowed Python application -- no console.
//
// Starts the script with pythonw.exe (or pyw.exe) next to the interpreter
// Launcher.cs would find, then exits; the application owns its own window and
// reports its own errors.  Only a missing script or interpreter is reported
// here, in a message box.
//
// @@SCRIPT@@ and @@TITLE@@ are substituted by build_launchers.ps1.  Arguments
// given to the .exe (e.g. a file dropped onto it) are passed to the script.

using System;
using System.Diagnostics;
using System.IO;
using System.Text;
using System.Windows.Forms;

static class GuiLauncher
{
    const string ScriptName  = "@@SCRIPT@@";
    const string WindowTitle = "@@TITLE@@";

    [STAThread]
    static int Main(string[] args)
    {
        string dir = AppDomain.CurrentDomain.BaseDirectory.TrimEnd('\\');
        string script = Path.Combine(dir, ScriptName);
        if (!File.Exists(script))
        {
            Fail("Cannot find " + ScriptName + " next to this launcher:\r\n\r\n" + script);
            return 2;
        }

        string python = FindWindowedPython();
        if (python == null)
        {
            Fail("No Python interpreter found.\r\n\r\n" +
                 "Install Python 3 and tick \"Add python.exe to PATH\", or set the\r\n" +
                 "ADCS_PYTHON environment variable to the full path of python.exe.");
            return 3;
        }

        StringBuilder line = new StringBuilder();
        if (Path.GetFileName(python).ToLowerInvariant() == "pyw.exe")
            line.Append("-3 ");
        line.Append(Quote(script));
        foreach (string arg in args)
            line.Append(' ').Append(Quote(arg));

        try
        {
            ProcessStartInfo psi = new ProcessStartInfo(python, line.ToString());
            psi.WorkingDirectory = dir;
            psi.UseShellExecute = false;
            Process.Start(psi);
        }
        catch (Exception ex)
        {
            Fail("Failed to start Python.\r\n\r\n" + ex.Message);
            return 4;
        }
        return 0;
    }

    /// <summary>pythonw.exe beside the python.exe Launcher.cs would use.</summary>
    static string FindWindowedPython()
    {
        string configured = Environment.GetEnvironmentVariable("ADCS_PYTHON");
        if (!string.IsNullOrEmpty(configured) && File.Exists(configured))
        {
            string windowed = Path.Combine(Path.GetDirectoryName(configured), "pythonw.exe");
            return File.Exists(windowed) ? windowed : configured;
        }

        string path = Environment.GetEnvironmentVariable("PATH") ?? "";
        foreach (string pair in new string[] { "python.exe|pythonw.exe", "py.exe|pyw.exe" })
        {
            string[] names = pair.Split('|');
            foreach (string entry in path.Split(';'))
            {
                string folder = entry.Trim().Trim('"');
                if (folder.Length == 0)
                    continue;
                try
                {
                    if (File.Exists(Path.Combine(folder, names[0])) &&
                        File.Exists(Path.Combine(folder, names[1])))
                        return Path.Combine(folder, names[1]);
                }
                catch { }   // PATH can hold entries with invalid characters
            }
        }

        foreach (Environment.SpecialFolder root in new Environment.SpecialFolder[] {
                     Environment.SpecialFolder.ProgramFiles,
                     Environment.SpecialFolder.LocalApplicationData })
        {
            string baseDir = Environment.GetFolderPath(root);
            if (root == Environment.SpecialFolder.LocalApplicationData)
                baseDir = Path.Combine(baseDir, "Programs\\Python");
            if (!Directory.Exists(baseDir))
                continue;
            string[] dirs = Directory.GetDirectories(baseDir, "Python3*");
            Array.Sort(dirs);
            for (int i = dirs.Length - 1; i >= 0; i--)
            {
                string candidate = Path.Combine(dirs[i], "pythonw.exe");
                if (File.Exists(candidate))
                    return candidate;
            }
        }
        return null;
    }

    static string Quote(string value)
    {
        return value.IndexOf(' ') >= 0 ? "\"" + value + "\"" : value;
    }

    static void Fail(string message)
    {
        MessageBox.Show(message, WindowTitle, MessageBoxButtons.OK, MessageBoxIcon.Error);
    }
}
