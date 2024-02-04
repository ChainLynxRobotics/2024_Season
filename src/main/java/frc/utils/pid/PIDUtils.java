package frc.utils.pid;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.revrobotics.SparkPIDController;

/**
 * Contains helper methods for simplifying and abstracting code related to PID controllers.
 * @see frc.utils.pid.PIDConfig
 * @see frc.utils.pid.PIDParam
 */
public final class PIDUtils {

  private PIDUtils() { }

  /**
   * Maps a {@link frc.utils.pid.PIDParam} to the corresponding setter method of a {@link com.revrobotics.SparkPIDController}.
   * <p> Example: {@code getCorrespondingSetMethod(controller, PIDParam.P)} will return a  {@link java.util.function.Consumer} {@code (value) -> controller.setP(value)}
   * @param controller The PID controller that the mapping will target
   * @param pidParam The {@link frc.utils.pid.PIDParam} that the mapping will reference
   */
  public static Consumer<Double> getCorrespondingSetMethod(SparkPIDController controller, PIDParam pidParam) {
    
    Consumer<Double> returnValue = null;

    switch (pidParam) {
      case P:
        returnValue = x -> controller.setP(x);
        break;
      case I:
        returnValue = x -> controller.setI(x);
        break;
      case D:
        returnValue = x -> controller.setD(x);
        break;
      case IZone:
        returnValue = x -> controller.setIZone(x);
        break;
      case FF:
        returnValue = x -> controller.setFF(x);
        break;
      case MaxOutput:
        returnValue = x -> controller.setOutputRange(controller.getOutputMin(), x);
        break;
      case MinOutput:
        returnValue = x -> controller.setOutputRange(x, controller.getOutputMax());
        break;
    }

    return returnValue;

  }

  /**
   * Maps a {@link frc.utils.pid.PIDParam} to the corresponding getter method of a {@link com.revrobotics.SparkPIDController}.
   * <p> Example: {@code getCorrespondingGetMethod(controller, PIDParam.P)} will return a  {@link java.util.function.Supplier} {@code () -> controller.getP()}
   * @param controller The PID controller that the mapping will target
   * @param pidParam The {@link frc.utils.pid.PIDParam} that the mapping will reference
   */
  public static Supplier<Double> getCorrespondingGetMethod(SparkPIDController controller, PIDParam pidParam) {
    
    Supplier<Double> returnValue = null;

    switch (pidParam) {
      case P:
        returnValue = () -> controller.getP();
        break;
      case I:
        returnValue = () -> controller.getI();
        break;
      case D:
        returnValue = () -> controller.getD();
        break;
      case IZone:
        returnValue = () -> controller.getIZone();
        break;
      case FF:
        returnValue = () -> controller.getFF();
        break;
      case MaxOutput:
        returnValue = () -> controller.getOutputMax();
        break;
      case MinOutput:
        returnValue = () -> controller.getOutputMin();
        break;
    }

    return returnValue;

  }
  
}