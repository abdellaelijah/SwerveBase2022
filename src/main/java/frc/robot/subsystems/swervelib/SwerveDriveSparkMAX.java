// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

/** Add your docs here. */
public class SwerveDriveSparkMAX extends SwerveMoveBase{

    private CANSparkMax driveMotor;
    private boolean areValuesUpdated = false;

    public SwerveDriveSparkMAX(int driveMotorID){
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);

        // above uses configSelectedFeedbackCoefficient(), to scale the
        // driveMotor to real distance, DRIVE_ENC_TO_METERS_FACTOR
        driveMotor.setInverted(false);// Set motor inverted(set to false)
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 1000);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20); 
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        driveMotor.burnFlash();
    }

    public SwerveDriveSparkMAX(int driveMotorID,double P, double I, double D, double F){
        areValuesUpdated = false;
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);

        // above uses configSelectedFeedbackCoefficient(), to scale the
        // driveMotor to real distance, DRIVE_ENC_TO_METERS_FACTOR
        driveMotor.setInverted(false);// Set motor inverted(set to false)
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 1000);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20); 
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);
        driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        driveMotor.burnFlash();
    }


    public void setDriveMotor(double dutyCycle){
        driveMotor.set(dutyCycle);
    }

    public void setDriveSpeed(double speed){
        driveMotor.set(speed);//TODO: this is not right, pid contoller
    }

    public double getDriveDistance(){
        return driveMotor.getEncoder().getPosition();//TODO: this is not scalled to meters
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
        return driveMotor.getEncoder().getVelocity();//TODO: this is not scalled to meters per second
    }

    public void resetDriveMotorEncoder(){
        driveMotor.getEncoder().setPosition(0.0);
    }
    
    public void setDriveMotorPIDF(double P, double I, double D, double F){
        if(driveMotor.getPIDController().getP() != P){
            driveMotor.getPIDController().setP(P);
            areValuesUpdated = true;
        }
        if(driveMotor.getPIDController().getI() != I){
            driveMotor.getPIDController().setI(I);
            areValuesUpdated = true;
        }
        if(driveMotor.getPIDController().getD() != D){
            driveMotor.getPIDController().setD(D);
            areValuesUpdated = true;
        }
        if(driveMotor.getPIDController().getFF() != F){
            driveMotor.getPIDController().setFF(F);
            areValuesUpdated = true;
        }
    }

    public void enableVoltageCompensation(double maximumVoltage){
        driveMotor.enableVoltageCompensation(maximumVoltage);

    }
}
