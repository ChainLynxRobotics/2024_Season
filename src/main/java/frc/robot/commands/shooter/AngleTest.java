package frc.robot.commands.shooter;

import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotConfig.ShooterConfig;
import frc.robot.subsystems.shooter.Shooter;

public class AngleTest extends Command {
    private Shooter m_shooter;

    public AngleTest(Shooter shooter) {
        m_shooter = shooter;

        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        m_shooter.setAngle(MutableMeasure.ofBaseUnits(160, Units.Rotations));
    }

    @Override
    public void execute() {
        double ff = Math.cos(m_shooter.getCurrentAngle().in(Units.Radians))*ShooterConfig.kAngleControlFF;
        System.out.println("ff at: " + ff);
        m_shooter.setFF(ff);
    }

}
