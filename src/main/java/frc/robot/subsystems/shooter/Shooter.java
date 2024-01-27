package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
/**
 * two sets of rollers of the same size spin same direction,
 * index of seconday rollers make note go shooter, controller for flywheels,
 * difference of speed in flywheels determines angle and speed of note.
 * PID controller per motor to account for different dynamics (no follower)
 * periodic log velocity setpoints
 * pid only in subsystems
 * set net velocity, angle diff fywheel speed)
 *
**/
public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Shooter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
