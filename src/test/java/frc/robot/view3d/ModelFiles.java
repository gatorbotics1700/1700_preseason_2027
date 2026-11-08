package frc.robot.view3d;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

/**
 * Keeps a mechanism's generated 3D model files in sync with the code. A model test calls these, so
 * changing a constant rebuilds the files and fails once, until the new files are committed.
 */
public final class ModelFiles {

  private ModelFiles() {}

  /** Writes every file that differs from what was just built, and returns their names. */
  public static List<String> rewriteStale(Path folder, Map<String, byte[]> files)
      throws IOException {
    List<String> rewritten = new ArrayList<>();
    Files.createDirectories(folder);
    for (Map.Entry<String, byte[]> file : files.entrySet()) {
      Path path = folder.resolve(file.getKey());
      boolean same = Files.exists(path) && Arrays.equals(Files.readAllBytes(path), file.getValue());
      if (!same) {
        Files.write(path, file.getValue());
        rewritten.add(file.getKey());
      }
    }
    return rewritten;
  }

  /** The files in the folder that were not built, so nothing stale lingers. */
  public static List<String> extraFiles(Path folder, Map<String, byte[]> files) {
    List<String> extra = new ArrayList<>();
    for (File file : folder.toFile().listFiles()) {
      if (!files.containsKey(file.getName())) {
        extra.add(file.getName());
      }
    }
    return extra;
  }
}
