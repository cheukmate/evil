package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;


import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;

import java.util.List;
import java.util.Optional;


import frc.robot.subsystems.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class ShootCommand extends Command {

    private record RecordedShot(Distance distance, AngularVelocity shooterSpeed, Time tof) {
        public Pair<Double, Double> getRPM() {
            return Pair.of(distance.in(Meters), shooterSpeed.in(RPM));
        }

        public Pair<Double, Double> getTOF() {
            return Pair.of(distance.in(Meters), tof.in(Second));
        }

    }

    private final Shooter shooter;
    private final Kicker kicker;
    private final Optional<SwerveSubsystem> swerve;
    private final AngularVelocity goalRPM;   // <-- parameter stored here
    private final double kickerSpeed = 1;
    private final double kickerStop = 0;
  

    private final Debouncer shootDebounce1 = new Debouncer(0.3, DebounceType.kFalling);

    private final List<RecordedShot> shots = List.of(
            // TUNE HERE
            new RecordedShot(Meters.of(1.25), RPM.of(2000), Second.of(1)),
            new RecordedShot(Meters.of(1.35), RPM.of(2100),Second.of(1)),
            new RecordedShot(Meters.of(1.45), RPM.of(2150),Second.of(1)),
            new RecordedShot(Meters.of(1.55), RPM.of(2200), Second.of(1)),
            new RecordedShot(Meters.of(1.65), RPM.of(2250),Second.of(1)),
            new RecordedShot(Meters.of(1.75), RPM.of(2300),Second.of(1)),
            new RecordedShot(Meters.of(1.85), RPM.of(2350), Second.of(1)),
            new RecordedShot(Meters.of(1.95), RPM.of(2400),Second.of(1)),
            new RecordedShot(Meters.of(2.05), RPM.of(2550),Second.of(1)),
            new RecordedShot(Meters.of(2.15), RPM.of(2600), Second.of(1)),
            new RecordedShot(Meters.of(2.30), RPM.of(2650),Second.of(1)),
            new RecordedShot(Meters.of(2.45), RPM.of(2700),Second.of(1)),
            new RecordedShot(Meters.of(2.60), RPM.of(2750), Second.of(1)),
            new RecordedShot(Meters.of(2.70), RPM.of(2800),Second.of(1)),
            new RecordedShot(Meters.of(2.90), RPM.of(2850),Second.of(1)),
            new RecordedShot(Meters.of(3.00), RPM.of(2900),Second.of(1)),
            new RecordedShot(Meters.of(3.10), RPM.of(2950),Second.of(1)),
            new RecordedShot(Meters.of(3.20), RPM.of(3000),Second.of(1)),
            new RecordedShot(Meters.of(3.30), RPM.of(3050),Second.of(1)),
            new RecordedShot(Meters.of(3.40), RPM.of(3100),Second.of(1)),
            new RecordedShot(Meters.of(3.50), RPM.of(3150),Second.of(1)),
            new RecordedShot(Meters.of(3.60), RPM.of(3200),Second.of(1)),
            new RecordedShot(Meters.of(3.70), RPM.of(3250),Second.of(1)),
            new RecordedShot(Meters.of(3.80), RPM.of(3300),Second.of(1)),
            new RecordedShot(Meters.of(3.90), RPM.of(3350),Second.of(1)),
            new RecordedShot(Meters.of(4.00), RPM.of(3400),Second.of(1)),
            new RecordedShot(Meters.of(4.5), RPM.of(3450), Second.of(1)),
            new RecordedShot(Meters.of(5.00), RPM.of(3500),Second.of(1))




    );
    private final InterpolatingDoubleTreeMap calculatedGoalRPM = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap calculatedTOF = new InterpolatingDoubleTreeMap();
   

    public ShootCommand(
            Shooter shooter,
            Kicker kicker,
           
            AngularVelocity goalRPM1,
            Angle goalDegree1   // <-- parameter passed in
    ) {
        this.shooter = shooter;
        this.kicker = kicker;
     
        this.swerve = Optional.empty();
        
        this.goalRPM = goalRPM1;   // <-- store parameter
        

        addRequirements(this.shooter, this.kicker);
    }

    public ShootCommand(
            Shooter shooter,
            Kicker kicker,
            
            SwerveSubsystem swerve) {
        this.shooter = shooter;
        this.kicker = kicker;
        
        this.swerve = Optional.of(swerve);
        goalRPM = RPM.zero();
        

        for (var shot : shots) {
            calculatedGoalRPM.put(shot.distance.in(Meters), shot.shooterSpeed.in(RPM));
            calculatedTOF.put(shot.distance.in(Meters), shot.tof.in(Second));
           
        }
        addRequirements(this.shooter, this.kicker);
    }

    

    @Override
    public void initialize() {
        // Spin up shooter to the passed RPM
        shooter.setVelocitySetpoint(goalRPM);
       // kicker.feedCommand();
        // if (swerve.isEmpty()) {
        //     hood.setAngleSetpoint(goalDegree);
        // }
        
        
    }

    @Override
    public void execute() {
        
        AngularVelocity goalRPM1 = goalRPM;
        
        if (swerve.isPresent()) {
            goalRPM1 = RPM.of(calculatedGoalRPM.get(swerve.get().distanceToHub()));
            //goalDegree1 = Degrees.of(calculatedHoodAngle.get(swerve.get().distanceToHub()));
        }

        shooter.setVelocitySetpoint(goalRPM1);
        
        //hood.setAngleSetpoint(goalDegree1);

      
         AngularVelocity shooterRPM = shooter.getRPM();

          boolean shooterReady =
        
         shootDebounce1.calculate(
                shooterRPM.isNear(
                        goalRPM1,
                       RPM.of(200)// tolerance
              )

     );
        // if (RobotBase.isSimulation()) {
            
        //     shooterReady = true;
            
        // }

         if(shooterReady){
          kicker.setDutyCycleSetpoint(kickerSpeed);
          
        } else {
         kicker.setDutyCycleSetpoint(kickerStop);
     }
            
       


    }

    @Override
    public void end(boolean interrupted) {
        shooter.setDutyCycleSetpoint(0);
        kicker.setDutyCycleSetpoint(kickerStop);
       // hood.setDutyCycleSetpoint(0);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}