// Double-click launcher for one Python entry point.
//
// Windows opens a console for this program, so a user with no terminal gets
// live output, working keyboard input (the simulator's live mode needs it) and
// a window that stays open at the end instead of vanishing with the result.
//
// The script name, window title, default arguments and start-up menu are
// substituted into the @@...@@ placeholders by build_launchers.ps1, which
// compiles one .exe per folder.  Any arguments given to the .exe are passed
// straight through to the script, replacing the defaults and skipping the menu
// -- so scripted callers keep full control.

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Text;

static class Launcher
{
    // ---- substituted at build time ----------------------------------------
    const string ScriptName  = "@@SCRIPT@@";
    const string WindowTitle = "@@TITLE@@";
    const string DefaultArgs = "@@DEFAULTARGS@@";

    // Start-up menu: "label|description|args" entries joined by ";;".  Empty
    // means no menu -- run DefaultArgs straight away.  The first entry is the
    // default (bare Enter, and what a non-interactive caller gets).  An entry
    // whose args are "?" prompts for an argument line instead.
    const string MenuSpec = "@@MENU@@";
    // -----------------------------------------------------------------------

    /// <summary>Args value marking a menu entry that prompts for its own.</summary>
    const string PromptMarker = "?";

    struct MenuOption
    {
        public string Label;
        public string Description;
        public string Args;
    }

    static int Main(string[] args)
    {
        try { Console.Title = WindowTitle; }
        catch { }

        string banner = new string('-', 72);
        Console.WriteLine(banner);
        Console.WriteLine("  " + WindowTitle);
        Console.WriteLine(banner);
        Console.WriteLine();

        string dir = AppDomain.CurrentDomain.BaseDirectory.TrimEnd('\\');
        string script = Path.Combine(dir, ScriptName);

        if (!File.Exists(script))
        {
            Fail("Cannot find " + ScriptName + " next to this launcher.\r\n\r\n" +
                 "Expected it at:\r\n  " + script + "\r\n\r\n" +
                 "Keep this .exe in the same folder as the script it runs.");
            return 2;
        }

        string python = FindPython();
        if (python == null)
        {
            Fail("No Python interpreter found.\r\n\r\n" +
                 "Install Python 3 and tick \"Add python.exe to PATH\", or set the\r\n" +
                 "ADCS_PYTHON environment variable to the full path of python.exe.");
            return 3;
        }

        // Explicit arguments win outright: no menu, no defaults.
        string passed;
        if (args.Length > 0)
        {
            passed = JoinArgs(args);
        }
        else
        {
            bool cancelled;
            passed = ChooseArgs(out cancelled);
            if (cancelled)
                return 0;
        }

        // py.exe is the version launcher, not an interpreter -- ask it for 3.x.
        string prefix = Path.GetFileName(python).ToLowerInvariant() == "py.exe" ? "-3 " : "";

        string commandLine = prefix + Quote(script);
        if (passed.Length > 0)
            commandLine += " " + passed;

        Console.WriteLine(banner);
        Console.WriteLine("  python  : " + python);
        Console.WriteLine("  folder  : " + dir);
        Console.WriteLine("  running : " + ScriptName + (passed.Length > 0 ? " " + passed : ""));
        Console.WriteLine(banner);
        Console.WriteLine();

        int exitCode;
        try
        {
            ProcessStartInfo psi = new ProcessStartInfo(python, commandLine);
            psi.WorkingDirectory = dir;
            // Inherit this console rather than redirecting: output appears as it
            // is produced and the child can still read the keyboard.
            psi.UseShellExecute = false;

            using (Process child = Process.Start(psi))
            {
                child.WaitForExit();
                exitCode = child.ExitCode;
            }
        }
        catch (Exception ex)
        {
            Fail("Failed to start Python.\r\n\r\n" + ex.Message);
            return 4;
        }

        Console.WriteLine();
        Console.WriteLine(banner);
        if (exitCode == 0)
        {
            Console.WriteLine("  Finished successfully.");
        }
        else
        {
            ConsoleColor previous = Console.ForegroundColor;
            try { Console.ForegroundColor = ConsoleColor.Red; }
            catch { }
            Console.WriteLine("  Finished with exit code " + exitCode + " -- see the messages above.");
            try { Console.ForegroundColor = previous; }
            catch { }
        }
        Console.WriteLine(banner);

        Pause();
        return exitCode;
    }

    /// <summary>
    /// Ask which way to run, when this launcher was built with a menu.  Falls
    /// back to the first entry (or the plain defaults) whenever there is no
    /// menu or nobody to answer it.
    /// </summary>
    static string ChooseArgs(out bool cancelled)
    {
        cancelled = false;

        MenuOption[] options = ParseMenu(MenuSpec);
        if (options.Length == 0)
            return DefaultArgs;

        // A piped or scheduled caller cannot answer; take the default silently.
        try
        {
            if (Console.IsInputRedirected)
                return options[0].Args == PromptMarker ? DefaultArgs : options[0].Args;
        }
        catch { }

        Console.WriteLine("  How do you want to run it?");
        Console.WriteLine();

        for (int i = 0; i < options.Length; i++)
        {
            string marker = i == 0 ? "  (default)" : "";
            Console.WriteLine("    [" + (i + 1) + "]  " + options[i].Label + marker);
            if (options[i].Description.Length > 0)
                Console.WriteLine("         " + options[i].Description);
            Console.WriteLine();
        }
        Console.WriteLine("    [q]  quit without running anything");
        Console.WriteLine();

        while (true)
        {
            Console.Write("  Choice [1]: ");
            string answer = Console.ReadLine();

            if (answer == null)          // end of input: behave like the default
                return options[0].Args;

            answer = answer.Trim();
            Console.WriteLine();

            if (answer.Length == 0)
                return Selected(options[0], out cancelled);

            if (answer.Equals("q", StringComparison.OrdinalIgnoreCase) ||
                answer.Equals("quit", StringComparison.OrdinalIgnoreCase))
            {
                cancelled = true;
                return "";
            }

            int choice;
            if (int.TryParse(answer, out choice) && choice >= 1 && choice <= options.Length)
                return Selected(options[choice - 1], out cancelled);

            Console.WriteLine("  Not one of the choices -- type a number from 1 to " +
                              options.Length + ", or q to quit.");
            Console.WriteLine();
        }
    }

    /// <summary>Resolve a chosen entry, prompting for arguments if it asks to.</summary>
    static string Selected(MenuOption option, out bool cancelled)
    {
        cancelled = false;
        if (option.Args != PromptMarker)
            return option.Args;

        Console.WriteLine("  Type the arguments to pass to " + ScriptName + ".");
        Console.WriteLine("  Leave it empty to run with no arguments, or type q to quit.");
        Console.WriteLine();
        Console.Write("  Arguments: ");

        string typed = Console.ReadLine();
        Console.WriteLine();

        if (typed == null)
            return "";

        typed = typed.Trim();
        if (typed.Equals("q", StringComparison.OrdinalIgnoreCase))
        {
            cancelled = true;
            return "";
        }
        return typed;
    }

    static MenuOption[] ParseMenu(string spec)
    {
        List<MenuOption> options = new List<MenuOption>();
        if (string.IsNullOrEmpty(spec))
            return options.ToArray();

        foreach (string entry in spec.Split(new string[] { ";;" }, StringSplitOptions.RemoveEmptyEntries))
        {
            string[] fields = entry.Split('|');
            if (fields.Length < 1 || fields[0].Trim().Length == 0)
                continue;

            MenuOption option = new MenuOption();
            option.Label       = fields[0].Trim();
            option.Description = fields.Length > 1 ? fields[1].Trim() : "";
            option.Args        = fields.Length > 2 ? fields[2].Trim() : "";
            options.Add(option);
        }
        return options.ToArray();
    }

    /// <summary>ADCS_PYTHON, then PATH, then the usual install locations.</summary>
    static string FindPython()
    {
        string configured = Environment.GetEnvironmentVariable("ADCS_PYTHON");
        if (!string.IsNullOrEmpty(configured) && File.Exists(configured))
            return configured;

        foreach (string name in new string[] { "python.exe", "py.exe" })
        {
            string found = SearchPath(name);
            if (found != null)
                return found;
        }

        return SearchInstallRoots();
    }

    static string SearchPath(string fileName)
    {
        string path = Environment.GetEnvironmentVariable("PATH");
        if (string.IsNullOrEmpty(path))
            return null;

        foreach (string entry in path.Split(';'))
        {
            string trimmed = entry.Trim().Trim('"');
            if (trimmed.Length == 0)
                continue;

            string candidate;
            try { candidate = Path.Combine(trimmed, fileName); }
            catch { continue; }   // PATH can hold entries with invalid characters

            if (File.Exists(candidate))
                return candidate;
        }
        return null;
    }

    /// <summary>Newest PythonNNN\python.exe under the standard install roots.</summary>
    static string SearchInstallRoots()
    {
        List<string> roots = new List<string>();
        roots.Add(Environment.GetFolderPath(Environment.SpecialFolder.ProgramFiles));
        roots.Add(Environment.GetFolderPath(Environment.SpecialFolder.ProgramFilesX86));
        roots.Add(Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
            "Programs\\Python"));

        string best = null;
        int bestVersion = -1;

        foreach (string root in roots)
        {
            if (string.IsNullOrEmpty(root) || !Directory.Exists(root))
                continue;

            string[] directories;
            try { directories = Directory.GetDirectories(root, "Python3*"); }
            catch { continue; }

            foreach (string directory in directories)
            {
                string exe = Path.Combine(directory, "python.exe");
                if (!File.Exists(exe))
                    continue;

                int version = TrailingDigits(Path.GetFileName(directory));
                if (version > bestVersion)
                {
                    bestVersion = version;
                    best = exe;
                }
            }
        }
        return best;
    }

    /// <summary>"Python312" -> 312, so the newest install sorts highest.</summary>
    static int TrailingDigits(string name)
    {
        StringBuilder digits = new StringBuilder();
        foreach (char c in name)
        {
            if (char.IsDigit(c))
                digits.Append(c);
        }

        int value;
        return int.TryParse(digits.ToString(), out value) ? value : 0;
    }

    static string JoinArgs(string[] args)
    {
        StringBuilder joined = new StringBuilder();
        foreach (string arg in args)
        {
            if (joined.Length > 0)
                joined.Append(' ');
            joined.Append(Quote(arg));
        }
        return joined.ToString();
    }

    static string Quote(string value)
    {
        return value.IndexOf(' ') >= 0 ? "\"" + value + "\"" : value;
    }

    static void Fail(string message)
    {
        ConsoleColor previous = Console.ForegroundColor;
        try { Console.ForegroundColor = ConsoleColor.Red; }
        catch { }

        Console.WriteLine();
        Console.WriteLine("ERROR");
        Console.WriteLine();
        Console.WriteLine(message);

        try { Console.ForegroundColor = previous; }
        catch { }

        Pause();
    }

    /// <summary>
    /// Hold the window open so a double-click user can read the result.  Skipped
    /// when the launcher is driven from a script or a pipe, where a blocking
    /// read would hang the caller instead of informing anyone.
    /// </summary>
    static void Pause()
    {
        try
        {
            if (Console.IsInputRedirected)
                return;
        }
        catch { }

        Console.WriteLine();
        Console.Write("Press any key to close this window . . . ");
        try { Console.ReadKey(true); }
        catch { Console.ReadLine(); }   // no interactive console available
        Console.WriteLine();
    }
}
