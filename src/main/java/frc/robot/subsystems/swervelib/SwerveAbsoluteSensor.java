// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervelib;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * 
 */
public abstract class SwerveAbsoluteSensor {
    
    /**
     * The CANCoder has a mechanical zero point, this is hard 
     * to move, so this method is used to set the offset of the 
     * CANCoder so we can dictate the zero position. 
     * INPUTS MUST BE IN DEGREES. 
     * 
     * @param value a number between -180 and 180, where 0 is straight ahead
     */
    protected abstract void setRotateAbsSensor(double value);

   /**
    * This method is used to change the offset of 
    * the absolute sensor so we dictate the zero 
    * position as the current position of the module.
    */
   public abstract void zeroAbsPositionSensor();

   /**
    * The CANCoder reads the absolute rotational position
    * of the module. This method returns that positon in 
    * degrees.
    * note: NOT Inverted module safe (use getPosInRad())
    * 
    * @return the position of the module in degrees, should limit from -180 to 180
    */
   public abstract double getAbsPosInDeg();

    /**
     * This method gets the current position in radians and 
     * normally the zero is at the front of the robot.
     * 
     * @return the position of the module in radians, should limit from -PI to PI
     */
    public double getPosInRad() {
        //get the current position and convert it to radians.
        return Math.toRadians(getAbsPosInDeg());
    }

    /**
     * Returns the current angle of the swerve module, 
     * as read by the absolute rotational sensor, as a 
     * Rotation2d object. This is measured from the 
     * front of the robot, where counter-clockwise is 
     * positive.
     * 
     * @return A Rotation2d object, current position of the module
     */
    public Rotation2d getCurRot2d(){
        return new Rotation2d(getPosInRad());
    }

  }
