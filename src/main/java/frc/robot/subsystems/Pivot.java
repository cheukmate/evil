// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {
  /** Creates a new Pivot. */

  SparkMax pivot = new SparkMax(Constants.IDConstants.PIVOT, MotorType.kBrushless);
  private final SparkClosedLoopController pidController = pivot.getClosedLoopController();
  RelativeEncoder encoder = pivot.getEncoder(); // i dont even think we need this but ok

// Conversion Factors and gear ratio

  public static final double GEAR_RATIO = 144.0;
  public static final double POSITION_FACTOR = 360.0/GEAR_RATIO;
  private static final double VELOCITY_FACTOR = POSITION_FACTOR / 60.0;

  

  // Targets

  private static final double STOW_ANGLE = 0;
  private static final double INTAKE_ANGLE = 90; //TODO: MATCH REALITY

  // PID Control 4 maxmotion :P
  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  public Pivot() {
    configureMotor();
  }


  private void configureMotor(){

      //config da motor

SparkMaxConfig globalConfig = new SparkMaxConfig();

    // return units in human numbers,, wtf is a rotation Guys

    globalConfig.encoder
    .positionConversionFactor(POSITION_FACTOR)
    .velocityConversionFactor(VELOCITY_FACTOR);

    globalConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(kP, kI, kD)  // TODO: TUNE TS
    .outputRange(-1, 1)
    .maxMotion
            .cruiseVelocity(150)
            .maxAcceleration(300)
            .allowedProfileError(1.0)
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    globalConfig.softLimit
    .forwardSoftLimit(100.0)
    .reverseSoftLimit(-5.0)
    .forwardSoftLimitEnabled(true)
    .reverseSoftLimitEnabled(true);

    pivot.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivot.getEncoder().setPosition(0);
  }

  private void setTargetPosition(double targetDegrees){

    pidController.setSetpoint(targetDegrees, ControlType.kMAXMotionPositionControl);

  }

  public double getAngle(){
    return pivot.getEncoder().getPosition();

  }

  // yay commands!

  public Command pivotToAngle(double degrees){
    return this.run(() -> setTargetPosition(degrees))
    // .until(Math.abs(getAngle() - degrees < 2.0))
    .withName("PivotTo" + degrees);
  
    
}

public Command Stow(){
return pivotToAngle(STOW_ANGLE);

}

public void stupidCommand(double power){
  pivot.set(power);

}
//no position control ^
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot Angle", pivot.getEncoder().getPosition());
    //SmartDashboard.putNumber("Pivot Setpoint", pivot.getClosedLoopController().getMAXMotionSetpointPosition());
    
  }
}
