package com.spartronics4915.lib.subsystems;

import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Subsystem abstract class, which serves as a basic framework for all robot
 * subsystems. Each subsystem outputs
 * commands to SmartDashboard, has a stop routine (for after each match), and a
 * routine to zero all sensors, which helps
 * with calibration.
 * <p>
 * All Subsystems only have one instance (after all, one robot does not have two
 * drivetrains), and functions get the
 * instance of the drivetrain and act accordingly.
 */
public abstract class SpartronicsSubsystem implements Subsystem
{

    // All subsystems should set mInitialized upon successful init.
    private boolean mInitialized = false;
    private String mName = null;

    public boolean isInitialized()
    {
        return mInitialized;
    }

    protected SpartronicsSubsystem()
    {
        String classname = this.getClass().getName();
        int tail = classname.lastIndexOf('.');
        if (tail == -1)
        {
            mName = classname;
        }
        else
        {
            mName = classname.substring(tail + 1);
        }
    }

    public String getClassName()
    {
        return mName;
    }

    public void logInitialized(boolean success)
    {
        mInitialized = success;
        if (success)
            this.logNotice("init SUCCEEDED");
        else
            this.logWarning("init FAILED");
        SmartDashboard.putString(mName + "/Status", mInitialized ? "OK" : "ERROR");
    }

    public void dashboardPutString(String nm, String value)
    {
        SmartDashboard.putString(mName + "/" + nm, value);
    }

    public String dashboardGetString(String nm, String defValue)
    {
        return SmartDashboard.getString(mName + "/" + nm, defValue);
    }

    public void dashboardPutNumber(String nm, Number value)
    {
        SmartDashboard.putNumber(mName + "/" + nm, value.doubleValue());
    }

    public Number dashboardGetNumber(String nm, Number defaultValue)
    {
        return SmartDashboard.getNumber(mName + "/" + nm, defaultValue.doubleValue());
    }

    public void dashboardPutBoolean(String nm, Boolean value)
    {
        SmartDashboard.putBoolean(mName + "/" + nm, value);
    }

    public boolean dashboardGetBoolean(String nm, Boolean defValue)
    {
        return SmartDashboard.getBoolean(mName + "/" + nm, defValue);
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
    public void periodic() {
        dashboardPutString("currentCommand", CommandScheduler.getInstance().requiring(this).getName());
    }
}
