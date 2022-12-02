// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

/** Add your docs here. */
public class SwerveAbsoluteCANCoder extends SwerveAbsoluteSensor{
    CANCoder rotateAbsSensor;

    public SwerveAbsoluteCANCoder(int canCoderID){
        //the following sensor is angle of the module, as an absolute value
        rotateAbsSensor = new CANCoder(canCoderID);
        rotateAbsSensor.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        rotateAbsSensor.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20);//The default on this is 10, but 20 might be better given our code loop rate
        rotateAbsSensor.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255);
        System.out.println( "VbatFaults" + rotateAbsSensor.getStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults));
    }

    @Override
    protected void setRotateAbsSensor(double value) {
        rotateAbsSensor.configMagnetOffset(value, 0);
    }

    @Override
    public void zeroAbsPositionSensor() {
        setRotateAbsSensor(this.rotateAbsSensor.configGetMagnetOffset()-getAbsPosInDeg());
    }

    @Override
    public double getAbsPosInDeg() {
        return rotateAbsSensor.getAbsolutePosition();
    
    }
}
