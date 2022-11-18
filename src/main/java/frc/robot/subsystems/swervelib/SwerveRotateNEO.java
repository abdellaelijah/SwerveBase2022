// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

/** Add your docs here. */
public class SwerveRotateNEO extends SwerveRotateBase {
    private CANSparkMax rotationMotor;

    public SwerveRotateNEO(int rotationMotorID){
          //contruct and setup rotation falcon
          rotationMotor = new CANSparkMax(rotationMotorID, MotorType.kBrushless);
          // use the integrated sensor with the primary closed loop and timeout is 0.
          // rotationMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
          // rotationMotor.configSelectedFeedbackCoefficient(1);
          rotationMotor.setIdleMode(IdleMode.kBrake);
          rotationMotor.setInverted(true);// Set motor inverted(set to true)
          rotationMotor.enableVoltageCompensation(Constants.MAXIMUM_VOLTAGE);
        
          rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 1000);
          rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20); //TODO: rethink if we need this speed, I don't think we do
          rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);
          rotationMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
          rotationMotor.burnFlash();
    }

    public void setRotationMotorPIDF(double P, double I, double D, double F){
        rotationMotor.getPIDController().setP(P);
        rotationMotor.getPIDController().setI(I);
        rotationMotor.getPIDController().setD(D);
        rotationMotor.getPIDController().setFF(F);
    }

    public double getRelEncCount(){
        return rotationMotor.getEncoder().getPosition();
    }

    public void driveRotateMotor(double speed){
        this.rotationMotor.set(speed);
    }

    public void setRotationMotorPosition(double output){
        rotationMotor.getPIDController().setReference(output, ControlType.kPosition);
    }

}
