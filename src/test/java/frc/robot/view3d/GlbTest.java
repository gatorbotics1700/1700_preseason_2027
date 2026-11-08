package frc.robot.view3d;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.view3d.Glb.Box;
import frc.robot.view3d.Glb.Part;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/** A model written with Glb reads back with the size it was given, in the robot frame. */
public class GlbTest {

  private static final double TOLERANCE_METERS = 1e-6;
  private static final double[] RED = {1, 0, 0, 1};

  /** Two boxes read back as one box around both, in robot coordinates (z up). */
  @Test
  public void boundsReadBackInTheRobotFrame(@TempDir Path folder) throws IOException {
    Box low = new Box(-0.5, -0.25, 0.0, 0.5, 0.25, 0.1);
    Box tall = Box.centered(0.2, 0.1, 0.6, 0.1, 0.1, 1.0);
    Path file = folder.resolve("model.glb");
    Files.write(file, Glb.write(List.of(new Part(RED, List.of(low, tall)))));

    Box bounds = Glb.readBounds(file);

    assertEquals(-0.5, bounds.minX, TOLERANCE_METERS);
    assertEquals(-0.25, bounds.minY, TOLERANCE_METERS);
    assertEquals(0.0, bounds.minZ, TOLERANCE_METERS);
    assertEquals(0.5, bounds.maxX, TOLERANCE_METERS);
    assertEquals(0.25, bounds.maxY, TOLERANCE_METERS);
    assertEquals(1.1, bounds.maxZ, TOLERANCE_METERS);
  }

  /** The same parts always give the same bytes, so model files only change when sizes do. */
  @Test
  public void sameInputGivesSameBytes() throws IOException {
    List<Part> parts = List.of(new Part(RED, List.of(Box.centered(0, 0, 0.5, 1, 1, 1))));
    assertArrayEquals(Glb.write(parts), Glb.write(parts));
  }
}
