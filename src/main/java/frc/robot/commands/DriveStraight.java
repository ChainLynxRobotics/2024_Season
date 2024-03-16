package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drivetrain;
import frc.utils.Vector;

public class DriveStraight extends Command {
    private Drivetrain m_drivetrain;
    private double m_distMeters;
    private double initDist;

    
    public DriveStraight(Drivetrain drivetrain, double distMeters) {
        this.m_drivetrain = drivetrain;
        m_distMeters = distMeters;

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        initDist = m_drivetrain.getPose().getX();
    }

    @Override
    public void execute() {
        m_drivetrain.drive(new Vector(0.2, 0), new Vector(), false, false);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(new Vector(0, 0), new Vector(0, 0), false, false);
    }


    @Override
    public boolean isFinished() {
        System.out.println("displacement: " + Math.abs(m_drivetrain.getPose().getX()-initDist));
        return Math.abs(m_drivetrain.getPose().getX() - initDist) > m_distMeters;
    }
}
