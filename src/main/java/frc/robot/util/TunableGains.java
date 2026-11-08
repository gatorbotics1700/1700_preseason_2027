package frc.robot.util;

import java.util.Arrays;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Closed loop gains you can change on the dashboard while tuning, under /Tuning/<name>/. Send them
 * to the motor only when changed() says so, because sending settings is slow.
 */
public class TunableGains {

  // Public so tests can type on the pretend dashboard.
  public final LoggedNetworkNumber kP;
  public final LoggedNetworkNumber kI;
  public final LoggedNetworkNumber kD;
  public final LoggedNetworkNumber kS;
  public final LoggedNetworkNumber kG;
  public final LoggedNetworkNumber kV;
  public final LoggedNetworkNumber kA;

  // The gains the motor has now.
  private final double[] sent;

  /** Gains for one mechanism, starting at these defaults (the ones the motor is set up with). */
  public TunableGains(
      String mechanismName,
      double kP,
      double kI,
      double kD,
      double kS,
      double kG,
      double kV,
      double kA) {
    String folder = "/Tuning/" + mechanismName + "/";
    this.kP = new LoggedNetworkNumber(folder + "kP", kP);
    this.kI = new LoggedNetworkNumber(folder + "kI", kI);
    this.kD = new LoggedNetworkNumber(folder + "kD", kD);
    this.kS = new LoggedNetworkNumber(folder + "kS", kS);
    this.kG = new LoggedNetworkNumber(folder + "kG", kG);
    this.kV = new LoggedNetworkNumber(folder + "kV", kV);
    this.kA = new LoggedNetworkNumber(folder + "kA", kA);
    sent = new double[] {kP, kI, kD, kS, kG, kV, kA};
  }

  /** True once each time a gain on the dashboard changes, so the caller sends the new gains. */
  public boolean changed() {
    double[] now = {kP.get(), kI.get(), kD.get(), kS.get(), kG.get(), kV.get(), kA.get()};
    if (Arrays.equals(now, sent)) {
      return false;
    }
    System.arraycopy(now, 0, sent, 0, now.length);
    return true;
  }
}
