package frc.utils;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.utils.Auto.Position;
import java.io.File;
import java.io.IOException;

public class PathReader {
  public static Pose2d getInitPose(String cmdName) {
    ObjectMapper mapper = new ObjectMapper();
    Rotation2d rot = new Rotation2d();
    Position pos = new Position();
    double xDist = 0;
    double yDist = 0;

    Auto auto;
    try {
      auto =
          mapper.readValue(
              new File("src/main/deploy/pathplanner/autos/" + cmdName + ".auto"), Auto.class);
      rot = new Rotation2d(auto.getStartingPose().getRotation());
      pos = auto.getStartingPose().getPosition();
      xDist = pos.getX();
      yDist = pos.getY();
    } catch (IOException e) {
      e.printStackTrace();
    }

    return new Pose2d(new Translation2d(xDist, yDist), rot);
  }
}
