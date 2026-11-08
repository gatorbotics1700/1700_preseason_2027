package frc.robot.view3d;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/** Model files are rewritten only when they change, and extra files are reported. */
public class ModelFilesTest {

  /** Missing files are written, and a second run finds nothing to rewrite. */
  @Test
  public void writesMissingFilesThenNothing(@TempDir Path folder) throws IOException {
    Map<String, byte[]> files = Map.of("model.glb", new byte[] {1, 2, 3});

    assertEquals(List.of("model.glb"), ModelFiles.rewriteStale(folder, files));
    assertEquals(List.of(), ModelFiles.rewriteStale(folder, files));
  }

  /** A changed file is rewritten. */
  @Test
  public void rewritesChangedFile(@TempDir Path folder) throws IOException {
    Files.write(folder.resolve("model.glb"), new byte[] {9});
    Map<String, byte[]> files = Map.of("model.glb", new byte[] {1, 2, 3});

    assertEquals(List.of("model.glb"), ModelFiles.rewriteStale(folder, files));
  }

  /** A file the model does not build is reported as extra. */
  @Test
  public void reportsExtraFiles(@TempDir Path folder) throws IOException {
    Files.write(folder.resolve("old_part.glb"), new byte[] {1});
    Map<String, byte[]> files = Map.of("model.glb", new byte[] {1});
    ModelFiles.rewriteStale(folder, files);

    assertEquals(List.of("old_part.glb"), ModelFiles.extraFiles(folder, files));
  }
}
