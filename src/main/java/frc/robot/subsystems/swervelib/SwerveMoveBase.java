// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib;

/** Add your docs here. */
public abstract class SwerveMoveBase {

     /**
     * Set the speed of the drive motor in percent duty cycle
     * 
     * @param dutyCycle a number between -1.0 and 1.0, where 0.0 is not moving, as
     *                  percent duty cycle
     */
    public abstract void setDriveMotor(double dutyCycle);

    /**
     * Set the speed of the drive motor in meter per second, this relies on the
     * PIDController built into the TalonFX.
     * 
     * @param speed a speed in meters per second
     */
    public abstract void setDriveSpeed(double speed); 

    /**
     * A method to set the drive motor to brake
     * @param brakeOn
     */
    public abstract void setDriveMotorBrake(boolean brakeOn);

     /**
     * @return the distance the drive wheel has traveled
     */
    public abstract double getDriveDistance();

    /**
     * Returns the speed of the drive wheel in Meters per second
     * 
     * @return speed of the drive wheel
     */
    public abstract double getDriveVelocity();

    /**
     * A method to set the position of the drive encoder to zero,
     * essentially resetting it. 
     */
    public abstract void resetDriveMotorEncoder();

    /**
     * sets the drive motor's PIDF for the PIDF controller on the controller
     * 
     * @param P value of the P constant
     * @param I value of the I constant
     * @param D value of the D constant
     * @param F value of the F constant
     */
    public abstract void setDriveMotorPIDF(double P, double I, double D, double F);

    public abstract void enableVoltageCompensation(double maximumVoltage);

}
