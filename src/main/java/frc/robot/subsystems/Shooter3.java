// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter3 extends SubsystemBase {
  /** Creates a new Shooter3. */

  TalonFX shooterMaster = new TalonFX(Constants.IDConstants.FLYWHEEL_MOTOR_MAIN_KRAKEN);
  TalonFX shooterSlave = new TalonFX(Constants.IDConstants.FLYWHEEL_MOTOR_FOLLOWER_KRAKEN);

  // Requests

  VelocityVoltage velocityRequest = new VelocityVoltage(0);

  public Shooter3() {

    //sets the follower status
 shooterSlave.setControl(new Follower(shooterMaster.getDeviceID(), MotorAlignmentValue.Aligned));
 
  var talonFXConfigs = new TalonFXConfiguration();
  var slot0Configs = talonFXConfigs.Slot0;

  
  slot0Configs.kS = 0.00;
  slot0Configs.kV = 0.12;
  slot0Configs.kA = 0.0;
  slot0Configs.kP = 0.0;
  slot0Configs.kI = 0;
  slot0Configs.kD = 0.00;

  //set Motion Magic velocity 
  //settings
  

shooterMaster.getConfigurator().apply(talonFXConfigs);
shooterSlave.getConfigurator().apply(talonFXConfigs);

  }

  public Command setVelocity(double velocity){
    return this.run(() -> {
      velocityRequest.withVelocity(velocity);
      shooterMaster.setControl(velocityRequest);
    });
  }

   @Override
   public void periodic() {
     //This method will be called once per scheduler run
   }
 }
