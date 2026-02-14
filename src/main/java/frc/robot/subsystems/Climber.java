// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  SparkMax climber = new SparkMax(Constants.IDConstants.CLIMBER_ID, MotorType.kBrushless);
 private SparkMaxConfig config = new SparkMaxConfig();
  public Climber() {
    configureClimber();
  }

 private void configureClimber(){
  config.smartCurrentLimit(40);
  config.idleMode(IdleMode.kBrake);


  climber.configure(config, ResetMode.kResetSafeParameters
  , PersistMode.kPersistParameters);

 }

  public void climbVoid(double speed){

    climber.set(speed);

  }

  public Command Climb(){
    return run(() -> {

    climbVoid(.4);
    
    });

  }

   public Command StopClimbing(){
    return run(() -> {

    climbVoid(0);
    
    });

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
