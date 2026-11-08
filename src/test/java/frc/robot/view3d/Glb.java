package frc.robot.view3d;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

/**
 * Reads and writes .glb files (the 3D model format AdvantageScope loads) made of colored boxes.
 * Positions are in the robot frame in meters (+x forward, +y left, +z up). glTF is y-up, so robot
 * (x, y, z) is written as (x, z, -y).
 */
public final class Glb {

  /** A box, by its lowest and highest corner, in the robot frame. */
  public static class Box {
    public final double minX;
    public final double minY;
    public final double minZ;
    public final double maxX;
    public final double maxY;
    public final double maxZ;

    /** A box from its two corners. */
    public Box(double minX, double minY, double minZ, double maxX, double maxY, double maxZ) {
      this.minX = minX;
      this.minY = minY;
      this.minZ = minZ;
      this.maxX = maxX;
      this.maxY = maxY;
      this.maxZ = maxZ;
    }

    /** A box from its center and its size along x, y and z. */
    public static Box centered(
        double centerX, double centerY, double centerZ, double sizeX, double sizeY, double sizeZ) {
      return new Box(
          centerX - sizeX / 2,
          centerY - sizeY / 2,
          centerZ - sizeZ / 2,
          centerX + sizeX / 2,
          centerY + sizeY / 2,
          centerZ + sizeZ / 2);
    }
  }

  /** Boxes that share one color. Colors are red, green, blue, opacity, each from 0 to 1. */
  public static class Part {
    final double[] color;
    final List<Box> boxes;

    /** A part made of these boxes, in this color. */
    public Part(double[] color, List<Box> boxes) {
      this.color = color;
      this.boxes = boxes;
    }
  }

  // Corners go counterclockwise seen from outside, so the renderer knows the front. 0=min, 1=max.
  private static final int[][] FACE_NORMALS = {
    {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}
  };
  private static final int[][][] FACE_CORNERS = {
    {{1, 0, 0}, {1, 1, 0}, {1, 1, 1}, {1, 0, 1}},
    {{0, 1, 0}, {0, 0, 0}, {0, 0, 1}, {0, 1, 1}},
    {{1, 1, 0}, {0, 1, 0}, {0, 1, 1}, {1, 1, 1}},
    {{0, 0, 0}, {1, 0, 0}, {1, 0, 1}, {0, 0, 1}},
    {{0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}},
    {{0, 1, 0}, {1, 1, 0}, {1, 0, 0}, {0, 0, 0}}
  };
  private static final int CORNERS_PER_BOX = 24; // 6 faces x 4 corners
  private static final int INDICES_PER_BOX = 36; // 6 faces x 2 triangles x 3 corners

  // Numbers the glTF format uses to name data types.
  private static final int FLOAT_TYPE = 5126;
  private static final int UNSIGNED_SHORT_TYPE = 5123;
  private static final int VERTEX_DATA = 34962;
  private static final int INDEX_DATA = 34963;
  private static final int GLB_MAGIC = 0x46546C67; // "glTF"
  private static final int JSON_CHUNK = 0x4E4F534A; // "JSON"
  private static final int BINARY_CHUNK = 0x004E4942; // "BIN"

  private static final ObjectMapper JSON = new ObjectMapper();

  private Glb() {}

  /** Builds a .glb file from parts. The same parts always give the same bytes. */
  public static byte[] write(List<Part> parts) throws IOException {
    ObjectNode gltf = JSON.createObjectNode();
    gltf.putObject("asset").put("version", "2.0").put("generator", "frc.robot.view3d.Glb");
    gltf.put("scene", 0);
    gltf.putArray("scenes").addObject().putArray("nodes").add(0);
    gltf.putArray("nodes").addObject().put("mesh", 0);
    ArrayNode primitives = gltf.putArray("meshes").addObject().putArray("primitives");
    ByteArrayOutputStream binary = new ByteArrayOutputStream();
    for (Part part : parts) {
      addPart(gltf, primitives, binary, part);
    }
    gltf.putArray("buffers").addObject().put("byteLength", binary.size());
    return packGlb(JSON.writeValueAsBytes(gltf), binary.toByteArray());
  }

  /** Adds one part: its material, then its corners, normals and triangles as binary data. */
  private static void addPart(
      ObjectNode gltf, ArrayNode primitives, ByteArrayOutputStream binary, Part part) {
    int material = arrayOf(gltf, "materials").size();
    ObjectNode pbr = arrayOf(gltf, "materials").addObject().putObject("pbrMetallicRoughness");
    ArrayNode color = pbr.putArray("baseColorFactor");
    for (double channel : part.color) {
      color.add(channel);
    }
    pbr.put("metallicFactor", 0.1).put("roughnessFactor", 0.7);

    int cornerCount = part.boxes.size() * CORNERS_PER_BOX;
    ByteBuffer positions = littleEndian(cornerCount * 3 * Float.BYTES);
    ByteBuffer normals = littleEndian(cornerCount * 3 * Float.BYTES);
    ByteBuffer indices = littleEndian(part.boxes.size() * INDICES_PER_BOX * Short.BYTES);
    float[] min = {Float.MAX_VALUE, Float.MAX_VALUE, Float.MAX_VALUE};
    float[] max = {-Float.MAX_VALUE, -Float.MAX_VALUE, -Float.MAX_VALUE};
    int corner = 0;
    for (Box box : part.boxes) {
      corner = addBox(box, corner, positions, normals, indices, min, max);
    }

    int accessor = arrayOf(gltf, "accessors").size();
    int positionView = addView(gltf, binary, positions, VERTEX_DATA);
    ObjectNode positionAccessor = addAccessor(gltf, positionView, FLOAT_TYPE, cornerCount, "VEC3");
    // AdvantageScope and the tests read the model's size from these.
    positionAccessor.set("min", floats(min));
    positionAccessor.set("max", floats(max));
    addAccessor(gltf, addView(gltf, binary, normals, VERTEX_DATA), FLOAT_TYPE, cornerCount, "VEC3");
    int indexCount = part.boxes.size() * INDICES_PER_BOX;
    addAccessor(
        gltf,
        addView(gltf, binary, indices, INDEX_DATA),
        UNSIGNED_SHORT_TYPE,
        indexCount,
        "SCALAR");

    ObjectNode primitive = primitives.addObject();
    primitive.putObject("attributes").put("POSITION", accessor).put("NORMAL", accessor + 1);
    primitive.put("indices", accessor + 2).put("material", material);
  }

  /** Writes one box's 24 corners, normals and 12 triangles. Returns the next corner number. */
  private static int addBox(
      Box box,
      int firstCorner,
      ByteBuffer positions,
      ByteBuffer normals,
      ByteBuffer indices,
      float[] min,
      float[] max) {
    double[][] corners = {{box.minX, box.minY, box.minZ}, {box.maxX, box.maxY, box.maxZ}};
    int corner = firstCorner;
    for (int face = 0; face < FACE_NORMALS.length; face++) {
      for (int[] pick : FACE_CORNERS[face]) {
        float[] glPosition = toGltf(corners[pick[0]][0], corners[pick[1]][1], corners[pick[2]][2]);
        float[] glNormal =
            toGltf(FACE_NORMALS[face][0], FACE_NORMALS[face][1], FACE_NORMALS[face][2]);
        for (int axis = 0; axis < 3; axis++) {
          positions.putFloat(glPosition[axis]);
          normals.putFloat(glNormal[axis]);
          min[axis] = Math.min(min[axis], glPosition[axis]);
          max[axis] = Math.max(max[axis], glPosition[axis]);
        }
      }
      int[] triangles = {0, 1, 2, 0, 2, 3};
      for (int offset : triangles) {
        indices.putShort((short) (corner + offset));
      }
      corner += 4;
    }
    return corner;
  }

  /** Robot frame (z up) to glTF (y up). Uses 0 - y so 0 stays 0 instead of -0. */
  private static float[] toGltf(double x, double y, double z) {
    return new float[] {(float) x, (float) z, (float) (0.0 - y)};
  }

  /** Adds binary data (padded to a multiple of 4 bytes, as glTF requires) and its "view". */
  private static int addView(
      ObjectNode gltf, ByteArrayOutputStream binary, ByteBuffer data, int target) {
    while (binary.size() % 4 != 0) {
      binary.write(0);
    }
    ArrayNode views = arrayOf(gltf, "bufferViews");
    ObjectNode view = views.addObject();
    view.put("buffer", 0).put("byteOffset", binary.size()).put("byteLength", data.capacity());
    view.put("target", target);
    binary.write(data.array(), 0, data.capacity());
    return views.size() - 1;
  }

  /** Adds an "accessor": how to read one view (number type, how many, single or x/y/z). */
  private static ObjectNode addAccessor(
      ObjectNode gltf, int view, int numberType, int count, String shape) {
    ObjectNode accessor = arrayOf(gltf, "accessors").addObject();
    accessor.put("bufferView", view).put("componentType", numberType);
    accessor.put("count", count).put("type", shape);
    return accessor;
  }

  /** Puts the JSON and the binary data into one .glb file, with the headers the format needs. */
  private static byte[] packGlb(byte[] json, byte[] binary) {
    int jsonLength = (json.length + 3) / 4 * 4; // padded with spaces to a multiple of 4
    int binaryLength = (binary.length + 3) / 4 * 4;
    int total = 12 + 8 + jsonLength + 8 + binaryLength;
    ByteBuffer glb = littleEndian(total);
    glb.putInt(GLB_MAGIC).putInt(2).putInt(total);
    glb.putInt(jsonLength).putInt(JSON_CHUNK).put(json);
    for (int i = json.length; i < jsonLength; i++) {
      glb.put((byte) ' ');
    }
    glb.putInt(binaryLength).putInt(BINARY_CHUNK).put(binary);
    return glb.array();
  }

  /** The box a whole .glb model fits in, in the robot frame. */
  public static Box readBounds(Path file) throws IOException {
    ByteBuffer glb = ByteBuffer.wrap(Files.readAllBytes(file)).order(ByteOrder.LITTLE_ENDIAN);
    if (glb.getInt(0) != GLB_MAGIC) {
      throw new IOException(file + " is not a .glb file");
    }
    int jsonLength = glb.getInt(12);
    String json = new String(glb.array(), 20, jsonLength, StandardCharsets.UTF_8);
    double[] min = {Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE};
    double[] max = {-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
    for (JsonNode accessor : JSON.readTree(json.trim()).get("accessors")) {
      if (accessor.has("min")) {
        for (int axis = 0; axis < 3; axis++) {
          min[axis] = Math.min(min[axis], accessor.get("min").get(axis).asDouble());
          max[axis] = Math.max(max[axis], accessor.get("max").get(axis).asDouble());
        }
      }
    }
    // glTF (x, y, z) is robot (x, -z, y).
    return new Box(min[0], -max[2], min[1], max[0], -min[2], max[1]);
  }

  private static ArrayNode arrayOf(ObjectNode gltf, String name) {
    return gltf.has(name) ? (ArrayNode) gltf.get(name) : gltf.putArray(name);
  }

  private static ArrayNode floats(float[] values) {
    ArrayNode array = JSON.createArrayNode();
    for (float value : values) {
      array.add(value);
    }
    return array;
  }

  private static ByteBuffer littleEndian(int size) {
    return ByteBuffer.allocate(size).order(ByteOrder.LITTLE_ENDIAN);
  }
}
