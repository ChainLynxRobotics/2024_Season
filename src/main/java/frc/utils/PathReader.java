package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class PathReader {
  public static Pose2d getInitPose(String cmdName) {
    Rotation2d rot = new Rotation2d();
    double xDist = 0;
    double yDist = 0;

    try {
      String pattern = "\"x\": (\\d+\\.\\d+)|\"y\": (\\d+\\.\\d+)|\"rotation\": (\\d+\\.\\d+)";
      Pattern p = Pattern.compile(pattern);

      //TODO verify this is the right path
      BufferedReader r = new BufferedReader(new FileReader(new File("C:/Users/ChainLynx/2024_Season/src/main/deploy/pathplanner/autos/"+cmdName+".auto")));
      StringBuilder json = new StringBuilder();
      String line;
      while ((line = r.readLine()) != null) {
        json.append(line);
      }

      Matcher m = p.matcher(json.toString());
      while (m.find()) {
        if (m.group(1) != null) {
          xDist = Double.parseDouble(m.group(1));
        } else if (m.group(2) != null) {
          yDist = Double.parseDouble(m.group(2));
        } else if (m.group(3) != null) {
          rot = new Rotation2d(Double.parseDouble(m.group(3)));
        }
      }
      r.close();
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    } catch (IOException e) {
      e.printStackTrace();
    }

    return new Pose2d(new Translation2d(xDist, yDist), rot);
  }
}
