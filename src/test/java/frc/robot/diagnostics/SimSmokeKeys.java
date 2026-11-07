package frc.robot.diagnostics;

/**
 * Log keys that only sim smoke writes (see frc.robot.util.SimSmoke). A check that needs the
 * simulator lists one of these in requiredKeys(), so it is skipped on match logs.
 */
public final class SimSmokeKeys {

  /** The name of the auto sim smoke ran, such as "Elevator Demo". */
  public static final String AUTO = "/RealOutputs/SimSmoke/Auto";

  private SimSmokeKeys() {}
}
