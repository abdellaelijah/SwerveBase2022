// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib;

/** Add your docs here. */
public abstract class SwerveMoveBase {

    public abstract void setDriveMotor(double dutyCycle);

    public abstract void setDriveSpeed(double speed);

    public abstract void setDriveMotorBrake(boolean brakeOn);

    public abstract double getDriveDistance();

    public abstract double getDriveVelocity();

    public abstract void resetDriveMotorEncoder();

    public abstract void setDriveMotorPIDF(double P, double I, double D, double F);

}
