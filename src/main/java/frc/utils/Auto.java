package frc.utils;

import com.fasterxml.jackson.annotation.JsonProperty;
import java.util.List;

public class Auto {
  private double version;
  private StartingPose startingPose;
  private Command command;
  private String folder;
  private boolean choreoAuto;

  @JsonProperty("version")
  public double getVersion() {
    return version;
  }

  public void setVersion(double version) {
    this.version = version;
  }

  @JsonProperty("startingPose")
  public StartingPose getStartingPose() {
    return startingPose;
  }

  public void setStartingPose(StartingPose startingPose) {
    this.startingPose = startingPose;
  }

  @JsonProperty("command")
  public Command getCommand() {
    return command;
  }

  public void setCommand(Command command) {
    this.command = command;
  }

  @JsonProperty("folder")
  public String getFolder() {
    return folder;
  }

  public void setFolder(String folder) {
    this.folder = folder;
  }

  @JsonProperty("choreoAuto")
  public boolean isChoreoAuto() {
    return choreoAuto;
  }

  public void setChoreoAuto(boolean choreoAuto) {
    this.choreoAuto = choreoAuto;
  }

  public static class StartingPose {
    private Position position;
    private double rotation;

    @JsonProperty("position")
    public Position getPosition() {
      return position;
    }

    public void setPosition(Position position) {
      this.position = position;
    }

    @JsonProperty("rotation")
    public double getRotation() {
      return rotation;
    }

    public void setRotation(double rotation) {
      this.rotation = rotation;
    }
  }

  public static class Position {
    private double x;
    private double y;

    @JsonProperty("x")
    public double getX() {
      return x;
    }

    public void setX(double x) {
      this.x = x;
    }

    @JsonProperty("y")
    public double getY() {
      return y;
    }

    public void setY(double y) {
      this.y = y;
    }
  }

  public static class Command {
    private String type;
    private Data data;

    @JsonProperty("type")
    public String getType() {
      return type;
    }

    public void setType(String type) {
      this.type = type;
    }

    @JsonProperty("data")
    public Data getData() {
      return data;
    }

    public void setData(Data data) {
      this.data = data;
    }
  }

  public static class Data {
    private List<Command> commands;
    private String pathName;
    private String name;

    @JsonProperty("commands")
    public List<Command> getCommands() {
      return commands;
    }

    public void setCommands(List<Command> commands) {
      this.commands = commands;
    }

    @JsonProperty("pathName")
    public String getPathName() {
      return pathName;
    }

    public void setPathName(String pathName) {
      this.pathName = pathName;
    }

    @JsonProperty("name")
    public String getName() {
      return name;
    }

    public void setName(String name) {
      this.name = name;
    }
  }
}
