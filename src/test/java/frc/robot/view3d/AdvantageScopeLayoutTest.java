package frc.robot.view3d;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.fasterxml.jackson.databind.JsonNode;
import java.io.IOException;
import java.util.List;
import org.junit.jupiter.api.Test;

/** The elevator's AdvantageScope layout draws the elevator robot correctly in its 3D tab. */
public class AdvantageScopeLayoutTest {

  /**
   * The 3D tab draws the robot, its parts, the goal ghost, then the ghost's parts. Order matters
   * because a part attaches to the robot or ghost listed just before it.
   */
  @Test
  public void field3dTabDrawsRobotThenPartsThenGhost() throws IOException {
    List<JsonNode> sources = View3dChecks.field3dSources(ElevatorModel.LAYOUT);

    assertEquals(4, sources.size());
    assertSource(sources.get(0), "robot", "/RealOutputs/Drive/Odometry/Robot");
    assertSource(sources.get(1), "component", "/RealOutputs/Mech/Elevator/ComponentPoses");
    assertSource(sources.get(2), "ghost", "/RealOutputs/Drive/Odometry/Robot");
    assertSource(sources.get(3), "component", "/RealOutputs/Mech/Elevator/ComponentPosesGoal");
  }

  /** The robot and the ghost use the model ElevatorModel builds, by its name. */
  @Test
  public void robotAndGhostUseTheElevatorModel() throws IOException {
    List<JsonNode> sources = View3dChecks.field3dSources(ElevatorModel.LAYOUT);

    assertEquals(ElevatorModel.ROBOT_NAME, sources.get(0).get("options").get("model").asText());
    assertEquals(ElevatorModel.ROBOT_NAME, sources.get(2).get("options").get("model").asText());
  }

  /** Checks one thing the 3D tab draws: what kind it is and which log key it reads. */
  private static void assertSource(JsonNode source, String type, String logKey) {
    assertEquals(type, source.get("type").asText());
    assertEquals(logKey, source.get("logKey").asText());
  }
}
