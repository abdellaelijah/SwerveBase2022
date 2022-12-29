// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib;

/**
 * This is the class given both motor controller object, 
 * the absolute sensor and contains all functions needed to run one 
 * swerve module. This class handles all access to these objects.
 * 
 * 
 * More on swerve found here:
 * https://docs.google.com/presentation/d/1feVl0L5lgIKSZhKCheWgWhkOydIu-ibgdp7oqA0yqAQ/edit?usp=sharing
 */
public class SwerveModule {

    /**
     * Creates a new SwerveModule object
     * 
     * @param driveMotor a SwerveDriveBase object, which
     */
    public SwerveModule(SwerveMoveBase driveMotor, SwerveRotateMotorBase rotationMotor, SwerveAbsoluteSensor rotateAbsSensor) {
        this(driveMotor, new SwerveRotationController(rotationMotor,rotateAbsSensor));
    }

    public SwerveModule(SwerveMoveBase driveMotor, SwerveRotationController rotationController){

    }
    
}
