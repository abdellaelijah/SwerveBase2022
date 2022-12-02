// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib;

/** Add your docs here. */
public abstract class SwerveRotateMotorBase {

    /**
     * sets the rotation motor's PIDF for the PIDF controller on the TalonFX
     * 
     * @param P value of the P constant
     * @param I value of the I constant
     * @param D value of the D constant
     * @param F value of the F constant
     */
    public abstract void setRotationMotorPIDF(double P, double I, double D, double F);

        /**
     * This is a method meant for testing by getting the count from the 
     * rotational encoder which is internal to the NEO550. This encoder 
     * is relative, and does not easily translate to a specific rotational 
     * position of the swerve module.
     * 
     * @return the encoder count(no units, naturally just the count)
     */
    public abstract double getRelEncCount();

    /**
     * This is a testing method, used to drive the module's rotation.
     * It takes pure motor duty cycle(percent output). Positive input 
     * should result in counter-clockwise rotation. If not, the motor
     * output must be inverted.
     * 
     * @param speed a percent output from -1.0 to 1.0, where 0.0 is stopped
     */
    public abstract void driveRotateMotor(double speed);

    /**
     * set the rotation motor to a position based on the motors internal units.
     * @param output
     */
    public abstract void setRotationMotorPosition(double output);

}
