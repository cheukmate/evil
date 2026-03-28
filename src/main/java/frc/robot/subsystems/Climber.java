// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class Climber extends SubsystemBase {
//   /** Creates a new Climber. */
//   SparkMax climber = new SparkMax(Constants.IDConstants.CLIMBER_ID_MAIN, MotorType.kBrushless);
//   SparkMax climber2 = new SparkMax(Constants.IDConstants.CLIMBER_ID_SECONDARY, MotorType.kBrushless);

//  private SparkMaxConfig config = new SparkMaxConfig();
//   public Climber() {
//     configureClimber();
//   }

//  private void configureClimber(){
//   config.smartCurrentLimit(40);
//   config.idleMode(IdleMode.kBrake);

// // 
//   climber.configure(config, ResetMode.kResetSafeParameters
//   , PersistMode.kPersistParameters);
//   climber2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

//  }

//   public void climbLeft(double speed){

//     climber.set(speed);

//   }

//   public void climbRight(double speed){

//     climber2.set(speed);
//   }


//   public void unclimbLeft(double speed){

//     climber.set(-speed);

//   }

//   public void unclimbRight(double speed){

//     climber2.set(-speed);
//   }

//   public void stopClimbing(double speed){
//     climber.set(0);
//     climber2.set(0);
//   }

//   public Command ClimbLeft(){
//     return run(() -> {

//     climbLeft(.6);
    
//     });

//   }
  
//   public Command ClimbRight(){
//     return run(() -> {

//     climbRight(.6);
    
//     });

//   }

//   public Command unClimbLeft(){
//     return run(() -> {

//     unclimbLeft(.6);
    
//     });

//   }
  
//   public Command unClimbRight(){
//     return run(() -> {

//     unclimbRight(.6);
    
//     });

//   }

//    public Command StopClimbing(){
//     return run(() -> {

//     stopClimbing(0);
    
//     });

//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }
