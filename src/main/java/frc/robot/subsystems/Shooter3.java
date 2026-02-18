// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter3 extends SubsystemBase {
  /** Creates a new Shooter3. */

  TalonFX shooterMaster = new TalonFX(Constants.IDConstants.FLYWHEEL_MOTOR_MAIN_KRAKEN);
  TalonFX shooterSlave = new TalonFX(Constants.IDConstants.FLYWHEEL_MOTOR_FOLLOWER_KRAKEN);


  // Requests

  VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  NeutralOut neturalRequest = new NeutralOut();

  public Shooter3() {
    ConfigureMotors();
  }

  private void ConfigureMotors(){

    //sets the follower status
 
   TalonFXConfiguration configs = new TalonFXConfiguration();
   Slot0Configs slot0Configs = new Slot0Configs();


  slot0Configs.kS = 0.05;
  slot0Configs.kV = 0.12;
  slot0Configs.kA = 0.00;
  slot0Configs.kP = 0.0001;
  slot0Configs.kI = 0;
  slot0Configs.kD = 0.00;

  configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

  configs.Feedback.SensorToMechanismRatio = .645161; 

      shooterMaster.getConfigurator().apply(configs);
      shooterSlave.getConfigurator().apply(configs);
 
      shooterMaster.getConfigurator().apply(slot0Configs);
      shooterSlave.getConfigurator().apply(slot0Configs);

      shooterSlave.setControl(new Follower(Constants.IDConstants.FLYWHEEL_MOTOR_MAIN_KRAKEN, MotorAlignmentValue.Aligned));

  }

  public void setVelocityVoid(double rps){
    shooterMaster.setControl(velocityRequest.withVelocity(rps).withSlot(0));
  }

  public void StopVoid(){
    shooterMaster.setControl(neturalRequest);
  }

  public Command Stop(){
  return run(() -> {

    StopVoid();

  });
    
  }

  public Command setVelocity(){
    return run(() -> {
      setVelocityVoid(8);
    });

  }


   @Override

   //TODO: remove before competition, can usage and such
   public void periodic() {
     //This method will be called once per scheduler run
     SmartDashboard.putNumber("Flywheel/ActualRPS", shooterMaster.getVelocity().getValueAsDouble());
     SmartDashboard.putNumber("Flywheel/Voltage", shooterMaster.getMotorVoltage().getValueAsDouble());
    //SmartDashboard.putBoolean("Position", shooterMaster.getMotorKV());
     
   }
 }
