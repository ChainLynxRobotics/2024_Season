package frc.utils.pid;

import edu.wpi.first.math.Pair;
import java.util.HashMap;
import java.util.function.BiConsumer;

/**
 * Stores config values for a {@link com.revrobotics.SparkPIDController}. Contains some helper
 * methods for abstraction.
 *
 * @see frc.utils.pid.PIDParam
 */
public class PIDConfig {

  // Stores mapping of PIDParam to the parameter key and value.
  private HashMap<PIDParam, Pair<String, Double>> configValues;

  /**
   * Constructs a new PIDConfig.
   *
   * @param PGainKey
   * @param PGainValue
   * @param IGainKey
   * @param IGainValue
   * @param DGainKey
   * @param DGainValue
   * @param IZoneKey
   * @param IZoneValue
   * @param FFGainKey
   * @param FFGainValue
   * @param MaxOutputKey
   * @param MaxOutputValue
   * @param MinOutputKey
   * @param MinOutputValue
   */
  public PIDConfig(
      String PGainKey,
      double PGainValue,
      String IGainKey,
      double IGainValue,
      String DGainKey,
      double DGainValue,
      String IZoneKey,
      double IZoneValue,
      String FFGainKey,
      double FFGainValue,
      String MaxOutputKey,
      double MaxOutputValue,
      String MinOutputKey,
      double MinOutputValue) {
    this.configValues.put(PIDParam.P, Pair.of(PGainKey, PGainValue));
    this.configValues.put(PIDParam.I, Pair.of(IGainKey, IGainValue));
    this.configValues.put(PIDParam.D, Pair.of(DGainKey, DGainValue));
    this.configValues.put(PIDParam.I_ZONE, Pair.of(IZoneKey, IZoneValue));
    this.configValues.put(PIDParam.FF, Pair.of(FFGainKey, FFGainValue));
    this.configValues.put(PIDParam.MAX_OUTPUT, Pair.of(MaxOutputKey, MaxOutputValue));
    this.configValues.put(PIDParam.MIN_OUTPUT, Pair.of(MinOutputKey, MinOutputValue));
  }

  /** Gets the key and value for the corresponding {@link frc.utils.pid.PIDParam}. */
  public Pair<String, Double> get(PIDParam param) {
    return this.configValues.get(param);
  }

  /** Gets the key for the corresponding {@link frc.utils.pid.PIDParam}. */
  public String getKey(PIDParam param) {
    return this.configValues.get(param).getFirst();
  }

  /** Gets the value for the corresponding {@link frc.utils.pid.PIDParam}. */
  public double getValue(PIDParam param) {
    return this.configValues.get(param).getSecond();
  }

  /** Iterates through the key and value for every {@link frc.utils.pid.PIDParam}. */
  public void forEachPIDParam(BiConsumer<PIDParam, Pair<String, Double>> consumer) {
    for (PIDParam param : PIDParam.values()) {
      consumer.accept(param, this.get(param));
    }
  }
}
