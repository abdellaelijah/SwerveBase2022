// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib;

/** Add your docs here. */
public class SwerveRotationController {
    SwerveRotateMotorBase rotatorMotor;
    SwerveAbsoluteSensor absSensor;
    public boolean useNeoABS;
    public SwerveRotationController(SwerveRotateMotorBase rotatorMotor, SwerveAbsoluteSensor absSensor){
        this.rotatorMotor = rotatorMotor;
        this.absSensor = absSensor;
        useNeoABS = false;
    }

    public SwerveRotationController(SwerveRotateNEO rotatorMotor){
        this.rotatorMotor = rotatorMotor;
        useNeoABS = true;
    }
}
