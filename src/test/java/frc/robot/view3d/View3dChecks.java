package frc.robot.view3d;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import frc.robot.diagnostics.LogCheck;
import frc.robot.diagnostics.LogFile;
import frc.robot.diagnostics.Report;
import frc.robot.diagnostics.SimSmokeKeys;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/** Log checks that keep the 3D view honest: what the robot logs is what AdvantageScope draws. */
public final class View3dChecks {

  /** The team's AdvantageScope layout. */
  public static final Path TEAM_LAYOUT =
      Path.of("src", "main", "deploy", "AdvantageScope-Layout.json");

  // AdvantageScope's number for a 3D field tab in a layout file.
  static final int FIELD_3D_TAB = 3;

  private View3dChecks() {}

  /** Every 3D view check. */
  public static List<LogCheck> all() {
    List<LogCheck> checks = new ArrayList<>();
    checks.add(new LayoutKeysAreLogged(TEAM_LAYOUT));
    return checks;
  }

  /**
   * Simulator only: every key a layout's 3D tab reads is in the log. AdvantageScope draws nothing
   * for a key that is never logged.
   */
  public static class LayoutKeysAreLogged implements LogCheck {
    private final Path layout;

    /** Checks the 3D tab of this layout file. */
    public LayoutKeysAreLogged(Path layout) {
      this.layout = layout;
    }

    /** The check's name in the report. */
    public String name() {
      return "3D view: " + layout.getFileName() + " keys are logged";
    }

    /** The log keys this check needs. */
    public String[] requiredKeys() {
      return new String[] {SimSmokeKeys.AUTO};
    }

    /** Looks through the log and reports what it finds. */
    public void check(LogFile log, Report report) {
      try {
        for (JsonNode source : field3dSources(layout)) {
          String key = source.get("logKey").asText();
          if (!log.has(key)) {
            report.error("the layout reads " + key + ", which the robot never logs");
          }
        }
      } catch (IOException e) {
        report.error("can't read " + layout + ": " + e.getMessage());
      }
    }
  }

  /** What a layout's first 3D tab draws, in order. */
  public static List<JsonNode> field3dSources(Path layout) throws IOException {
    JsonNode root = new ObjectMapper().readTree(layout.toFile());
    for (JsonNode hub : root.get("hubs")) {
      JsonNode state = hub.get("state");
      JsonNode tabGroup = state.get("tabs");
      for (JsonNode tab : tabGroup.get("tabs")) {
        if (tab.get("type").asInt() == FIELD_3D_TAB) {
          List<JsonNode> sources = new ArrayList<>();
          JsonNode controller = tab.get("controller");
          for (JsonNode source : controller.get("sources")) {
            sources.add(source);
          }
          return sources;
        }
      }
    }
    throw new IOException("no 3D field tab in " + layout);
  }
}
