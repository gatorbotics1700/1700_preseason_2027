package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj.DriverStation;
import org.junit.jupiter.api.Test;

public class RobotContainerTest {
  @Test
  public void createRobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    try {

      new RobotContainer();

    } catch (Exception error) {
      error.printStackTrace();
      fail("Failed to instantiate robot container!!!!");
    }
  }
}
