package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drivetrain;
import frc.utils.Vector;

public class LeaveFromStationCommand extends Command {
  private final Drivetrain m_drive;
  private final double m_driveTime;
  private final Vector m_driveVec;
  private final boolean m_resetGyro;

  private final Timer m_timer;

  public LeaveFromStationCommand(Drivetrain drive, double driveTimeSeconds) {
    this(drive, driveTimeSeconds, 0.5, Math.PI / 2, false);
  }

  public LeaveFromStationCommand(
      Drivetrain drive,
      double driveTimeSeconds,
      double driveProp,
      double driveAng,
      boolean resetGyro) {
    m_driveTime = driveTimeSeconds;
    m_drive = drive;
    m_driveVec = new Vector(Math.cos(driveAng), Math.sin(driveAng)).mult(driveProp);
    m_timer = new Timer();
    m_resetGyro = resetGyro;
  }

  @Override
  public void initialize() {
    m_timer.start();
    if (m_resetGyro) m_drive.zeroHeading();
  }

  @Override
  public void execute() {
    m_drive.move(m_driveVec, 0);
  }

  @Override
  public boolean isFinished() {
    return m_timer.get() >= m_driveTime;
  }

  @Override
  public void end(boolean isInterupted) {
    m_drive.move(new Vector(), 0);
  }
}
