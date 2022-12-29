// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class SwerveRotationController {
    private SwerveRotateMotorBase rotatorMotor;
    private SwerveAbsoluteSensor absSensor;
    public boolean useNeoABS;

    /**
     * the states for the BallHandler state machine
     */
    private enum AbsSensorType {
        kRemoteSensor, kNEOABS
    }

    private AbsSensorType sensorType;

    public SwerveRotationController(SwerveRotateMotorBase rotatorMotor, SwerveAbsoluteSensor absSensor){
        this.rotatorMotor = rotatorMotor;
        this.absSensor = absSensor;
        sensorType = AbsSensorType.kRemoteSensor;
    }

    public SwerveRotationController(SwerveRotateNEO rotatorMotor){
        this.rotatorMotor = rotatorMotor;
        sensorType = AbsSensorType.kNEOABS;
    }

    //functions for abs sensor

    /**
     * The CANCoder has a mechanical zero point, this is hard 
     * to move, so this method is used to change the offset of 
     * the CANCoder so we dictate the zero position as the 
     * current position of the module.
     */
    public void zeroAbsPositionSensor(){
        switch(sensorType){
            case kRemoteSensor:
                absSensor.zeroAbsPositionSensor();
                break;
            case kNEOABS:
                //TODO: fixme
                break;
        }
    }

    public double getAbsPosInDeg(){
        switch(sensorType){
            case kRemoteSensor:
                return absSensor.getAbsPosInDeg();
            case kNEOABS:
                return 0.0;//TODO: fixme
        }
        //Just incase we need a default return
        return 0.0;
    }

    public double getPosInRad() {
        switch(sensorType){
            case kRemoteSensor:
                return absSensor.getPosInRad();
            case kNEOABS:
                return 0.0;//TODO: fixme
        }
        //Just incase we need a default return
        return 0.0;
    }

    public Rotation2d getCurRot2d(){
        switch(sensorType){
            case kRemoteSensor:
                return absSensor.getCurRot2d();
            case kNEOABS:
                // return 0.0;//TODO: fixme
        }
        //Just incase we need a default return
        return new Rotation2d(0);
    }

    //functions for rotation motor
    
    
}
