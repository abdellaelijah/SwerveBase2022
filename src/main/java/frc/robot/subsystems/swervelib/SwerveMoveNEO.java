// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import frc.robot.Constants;

/** Add your docs here. */
public class SwerveMoveNEO extends SwerveMoveBase{

    private CANSparkMax driveMotor;

    public SwerveMoveNEO(int driveMotorID){
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        // use the integrated sensor with the primary closed loop and timeout is 0.
        // driveMotor.//Constants.DRIVE_ENC_TO_METERS_FACTOR);
        // above uses configSelectedFeedbackCoefficient(), to scale the
        // driveMotor to real distance, DRIVE_ENC_TO_METERS_FACTOR
        driveMotor.setInverted(false);// Set motor inverted(set to false)
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 1000);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20); //TODO: rethink if we need this speed, I don't think we do
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        driveMotor.burnFlash();
    }


    public void setDriveMotor(double dutyCycle){
        driveMotor.set(dutyCycle);
    }

    public void setDriveSpeed(double speed){
        driveMotor.set(speed);
    }

    public double getDriveDistance(){
        return driveMotor.getEncoder().getPosition();
    }

    public void setDriveMotorBrake(boolean brakeOn){
        if(brakeOn){
            driveMotor.setIdleMode(IdleMode.kBrake);
        }
        else{
            driveMotor.setIdleMode(IdleMode.kCoast);
        }

    
    }
    public double getDriveVelocity(){
        return driveMotor.getEncoder().getVelocity();
    }

    public void resetDriveMotorEncoder(){
        driveMotor.getEncoder().setPosition(0.0);
    }
    
    public void setDriveMotorPIDF(double P, double I, double D, double F){
        driveMotor.getPIDController().setP(P);
        driveMotor.getPIDController().setI(I);
        driveMotor.getPIDController().setD(D);
        driveMotor.getPIDController().setFF(F);
    }

    public void enableVoltageCompensation(double maximumVoltage){
        driveMotor.enableVoltageCompensation(maximumVoltage);

    }
}
