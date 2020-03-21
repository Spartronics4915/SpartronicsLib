package com.spartronics4915.lib.util;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.nio.file.Paths;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.UUID;

/**
 * Tracks start-up and caught crash events, logging them to a file which dosn't
 * roll over
 */
public class Logger
{
    /**
     * a ClientWrite subclass can intercept logging messages usually sent
     * to stdout. This may be of use for unit testing.
     */
    public static abstract class ClientWriter
    {
        public void writeln(String str, Throwable Exception) {}
    }

    private static final UUID sRunInstanceUUID = UUID.randomUUID();
    public static int sVerbosity = 2; // 0: notices and above, 1: info and above, 2: all
    private static final DateFormat sDateFormat = new SimpleDateFormat("hh:mm:ss");
    private static ClientWriter sClientWriter = null;

    public static void installClientWriter(ClientWriter w)
    {
        sClientWriter = w;
    }

    public static void setVerbosity(String nm)
    {
        if (nm.equals("NOTICE"))
            sVerbosity = 0;
        else if (nm.equals("INFO"))
            sVerbosity = 1;
        else if (nm.equals("DEBUG"))
            sVerbosity = 2;
        else
            error("Logger: unknown verbosity level:" + nm);
    }

    public static void logRobotStartup()
    {
        notice("robot startup");
    }

    public static void logRobotConstruction()
    {
        notice("robot construction");
    }

    public static void logRobotInit()
    {
        notice("robot init");
    }

    public static void logTeleopInit()
    {
        notice("teleop init");
    }

    public static void logAutoInit()
    {
        notice("auto init");
    }

    public static void logDisabledInit()
    {
        notice("disabled init");
    }

    public static void logTestInit()
    {
        notice("test init");
    }

    public static void logThrowableCrash(Throwable throwable)
    {
        logMarker("Exception", throwable);
    }

    public static void logThrowableCrash(String msg, Throwable throwable)
    {
        logMarker("ERROR " + msg, throwable);
    }

    public static void exception(Throwable e)
    {
        StringWriter sw = new StringWriter();
        e.printStackTrace(new PrintWriter(sw));
        logMarker("EXCEPT  " + e.getMessage() + " trace:\n" + sw.toString());
    }

    public static void error(String m)
    {
        logMarker("ERROR   " + m);
    }

    public static void warning(String m)
    {
        logMarker("WARNING " + m);
    }

    public static void notice(String m)
    {
        logMarker("NOTICE  " + m);
    }

    public static void info(String m)
    {
        if (sVerbosity > 0)
        {
            printMarker("INFO    " + m);
        }
    }

    public static void debug(String m)
    {
        if (sVerbosity > 1)
        {
            printMarker("DEBUG    " + m);
        }
    }

    private static String getTimeStamp()
    {
        Date now = new Date();
        String nowstr = sDateFormat.format(now) + " ";
        return nowstr;
    }

    private static void logMarker(String mark)
    {
        logMarker(mark, null);
    }

    private static void printMarker(String mark)
    {
        if(sClientWriter != null)
        {
            sClientWriter.writeln(mark, null);
        }
        else
            System.out.println(mark);
    }

    private static void logMarker(String mark, Throwable nullableException)
    {
        if(sClientWriter != null)
        {
            sClientWriter.writeln(mark, nullableException);
        }
        else
        {
            printMarker(mark);
            if (nullableException != null)
                nullableException.printStackTrace();
        }
        try (PrintWriter writer = new PrintWriter(new FileWriter(
            Paths.get(System.getProperty("user.home"), "crash_tracking.txt").toString(), true)))
        {
            writer.print(sRunInstanceUUID.toString());
            writer.print(", ");
            writer.print(mark);
            if (nullableException != null)
            {
                writer.print(", ");
                nullableException.printStackTrace(writer);
            }
            writer.println();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }
}
