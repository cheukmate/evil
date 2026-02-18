// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeWheels extends SubsystemBase {
  /** Creates a new IntakeWheels. */
  SparkFlex wheels  = new SparkFlex(Constants.IDConstants.INTAKEWHEELS_FLEX_MAIN, MotorType.kBrushless);
  SparkFlex wheelsFollower = new SparkFlex(Constants.IDConstants.INTAKEWHEELS_FLEX_FOLLOWER, MotorType.kBrushless);

  SparkMax indexer = new SparkMax(Constants.IDConstants.INDEXER, MotorType.kBrushless);

  SparkFlexConfig config = new SparkFlexConfig();
  SparkMaxConfig config2 = new SparkMaxConfig();

  
  public IntakeWheels() {
    configureMotor();
    
    
  }

  public void configureMotor(){
    config.idleMode(IdleMode.kCoast);
    config.voltageCompensation(12.3);
    config.smartCurrentLimit(40);
    

    wheels.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    wheelsFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config2.idleMode(IdleMode.kCoast);
    config2.voltageCompensation(12.3);
    config2.smartCurrentLimit(25);

    indexer.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void Intake(double speed){
    wheels.set(-speed);
    wheelsFollower.set(speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
