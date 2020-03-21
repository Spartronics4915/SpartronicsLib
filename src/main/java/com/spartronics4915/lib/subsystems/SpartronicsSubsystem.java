package com.spartronics4915.lib.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.spartronics4915.lib.util.Logger;


/**
 * We introduce an abstract SpartronicsSubsystem class to provide uniformity
 * in logging, smartdashboard and initialization behavior to all subclasses.
 *
 * The Subsystem abstract class, which serves as a basic framework for all robot
 * subsystems. Each subsystem outputs commands to SmartDashboard, has a stop
 * routine (for after each match), and a routine to zero all sensors, which helps
 * with calibration.
 * <p>
 * All Subsystems only have one instance (after all, one robot does not have two
 * drivetrains), and functions get the instance of the drivetrain and act
 * accordingly.
 */
public abstract class SpartronicsSubsystem extends SubsystemBase
{
    // We install a default command for diagnostic purposes.  Subclasses
    // should override the default command during construction.
    private class SpartronicsMissingDefaultCommand extends CommandBase
    {
        SpartronicsMissingDefaultCommand()
        {
            this.addRequirements(SpartronicsSubsystem.this);
        }
    };

    // All subsystems should set mInitialized upon successful init.
    private boolean mInitialized = false;
    private String mName = null;

    public boolean isInitialized()
    {
        return mInitialized;
    }

    protected SpartronicsSubsystem()
    {
        // NB: SubsystemBase posts some interesting info to dashboard
        // using our class name.
        String classname = this.getClass().getName();
        int tail = classname.lastIndexOf('.');
        if (tail == -1)
        {
            this.mName = classname;
        }
        else
        {
            this.mName = classname.substring(tail + 1);
        }
        this.setDefaultCommand(new SpartronicsMissingDefaultCommand());
    }

    public String getClassName()
    {
        return this.mName;
    }

    public void logInitialized(boolean success)
    {
        this.mInitialized = success;
        if (success)
            this.logNotice("init SUCCEEDED");
        else
            this.logWarning("init FAILED");
        SmartDashboard.putString(this.mName + "/Status", this.mInitialized ? "OK" : "ERROR");
    }

    public void dashboardPutString(String nm, String value)
    {
        SmartDashboard.putString(this.mName + "/" + nm, value);
    }

    public String dashboardGetString(String nm, String defValue)
    {
        return SmartDashboard.getString(this.mName + "/" + nm, defValue);
    }

    public void dashboardPutNumber(String nm, Number value)
    {
        SmartDashboard.putNumber(this.mName + "/" + nm, value.doubleValue());
    }

    public Number dashboardGetNumber(String nm, Number defaultValue)
    {
        return SmartDashboard.getNumber(this.mName + "/" + nm, defaultValue.doubleValue());
    }

    public void dashboardPutBoolean(String nm, Boolean value)
    {
        SmartDashboard.putBoolean(this.mName + "/" + nm, value);
    }

    public boolean dashboardGetBoolean(String nm, Boolean defValue)
    {
        return SmartDashboard.getBoolean(this.mName + "/" + nm, defValue);
    }

    // log methods are for conventionalizing format across subsystems
    public void logException(String msg, Throwable e)
    {
        Logger.logThrowableCrash(this.getClassName() + " " + msg, e);
    }

    public void logError(String msg)
    {
        Logger.error(this.getClassName() + " " + msg);
    }

    public void logWarning(String msg)
    {
        Logger.warning(this.getClassName() + " " + msg);
    }

    public void logNotice(String msg)
    {
        Logger.notice(this.getClassName() + " " + msg);
    }

    public void logInfo(String msg)
    {
        Logger.info(this.getClassName() + " " + msg);
    }

    public void logDebug(String msg)
    {
        Logger.debug(this.getClassName() + " " + msg);
    }

    public void zeroSensors()
    {
    }

    @Override
    public void periodic()
    {
        Command currentCommand = CommandScheduler.getInstance().requiring(this);

        // this.mName/currentCommand contains the current command for
        // a subsystem.
        dashboardPutString("currentCommand",
            currentCommand != null ? currentCommand.getName() : "none");
    }
}
