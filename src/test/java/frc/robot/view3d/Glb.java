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
    public final double minXMeters;
    public final double minYMeters;
    public final double minZMeters;
    public final double maxXMeters;
    public final double maxYMeters;
    public final double maxZMeters;

    /** A box from its two corners. */
    public Box(
        double minXMeters,
        double minYMeters,
        double minZMeters,
        double maxXMeters,
        double maxYMeters,
        double maxZMeters) {
      this.minXMeters = minXMeters;
      this.minYMeters = minYMeters;
      this.minZMeters = minZMeters;
      this.maxXMeters = maxXMeters;
      this.maxYMeters = maxYMeters;
      this.maxZMeters = maxZMeters;
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

  /** One part's binary data: corner positions, normals, and the corners of each triangle. */
  private static class Mesh {
    final ByteBuffer positions;
    final ByteBuffer normals;
    final ByteBuffer indices;
    final float[] min = {Float.MAX_VALUE, Float.MAX_VALUE, Float.MAX_VALUE};
    final float[] max = {-Float.MAX_VALUE, -Float.MAX_VALUE, -Float.MAX_VALUE};
    final int cornerCount;
    final int indexCount;
    int nextCorner = 0;

    /** Room for this many boxes. */
    Mesh(int boxCount) {
      cornerCount = boxCount * CORNERS_PER_BOX;
      indexCount = boxCount * INDICES_PER_BOX;
      positions = littleEndian(cornerCount * 3 * Float.BYTES);
      normals = littleEndian(cornerCount * 3 * Float.BYTES);
      indices = littleEndian(indexCount * Short.BYTES);
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
  // Two triangles make a square face: its corners 0-1-2 and 0-2-3.
  private static final int[] FACE_TRIANGLES = {0, 1, 2, 0, 2, 3};
  private static final int CORNERS_PER_FACE = 4;
  private static final int CORNERS_PER_BOX = 24; // 6 faces x 4 corners
  private static final int INDICES_PER_BOX = 36; // 6 faces x 2 triangles x 3 corners

  // How shiny and how rough every part looks.
  private static final double METALLIC = 0.1;
  private static final double ROUGHNESS = 0.7;

  // Numbers the glTF format uses to name data types.
  private static final int FLOAT_TYPE = 5126;
  private static final int UNSIGNED_SHORT_TYPE = 5123;
  private static final int VERTEX_DATA = 34962;
  private static final int INDEX_DATA = 34963;

  // The .glb file layout: a 12 byte header, then chunks that each start with an 8 byte header.
  private static final int GLB_MAGIC = 0x46546C67; // "glTF"
  private static final int GLB_VERSION = 2;
  private static final int GLB_HEADER_BYTES = 12;
  private static final int CHUNK_HEADER_BYTES = 8;
  private static final int JSON_CHUNK = 0x4E4F534A; // "JSON"
  private static final int BINARY_CHUNK = 0x004E4942; // "BIN"
  private static final int CHUNK_ALIGNMENT_BYTES = 4;

  private static final ObjectMapper JSON = new ObjectMapper();

  private Glb() {}

  /** Builds a .glb file from parts. The same parts always give the same bytes. */
  public static byte[] write(List<Part> parts) throws IOException {
    ObjectNode gltf = JSON.createObjectNode();
    addScene(gltf);
    ArrayNode meshes = gltf.putArray("meshes");
    ObjectNode mesh = meshes.addObject();
    ArrayNode primitives = mesh.putArray("primitives");
    ByteArrayOutputStream binary = new ByteArrayOutputStream();
    for (Part part : parts) {
      addPart(gltf, primitives, binary, part);
    }
    ArrayNode buffers = gltf.putArray("buffers");
    ObjectNode buffer = buffers.addObject();
    buffer.put("byteLength", binary.size());
    return packGlb(JSON.writeValueAsBytes(gltf), binary.toByteArray());
  }

  /** Adds the file's header: one scene with one node that shows the one mesh. */
  private static void addScene(ObjectNode gltf) {
    ObjectNode asset = gltf.putObject("asset");
    asset.put("version", "2.0");
    asset.put("generator", "frc.robot.view3d.Glb");
    gltf.put("scene", 0);
    ArrayNode scenes = gltf.putArray("scenes");
    ObjectNode scene = scenes.addObject();
    ArrayNode sceneNodes = scene.putArray("nodes");
    sceneNodes.add(0);
    ArrayNode nodes = gltf.putArray("nodes");
    ObjectNode node = nodes.addObject();
    node.put("mesh", 0);
  }

  /** Adds one part: its material, then its corners, normals and triangles as binary data. */
  private static void addPart(
      ObjectNode gltf, ArrayNode primitives, ByteArrayOutputStream binary, Part part) {
    int material = addMaterial(gltf, part.color);
    Mesh mesh = new Mesh(part.boxes.size());
    for (Box box : part.boxes) {
      addBox(box, mesh);
    }
    int firstAccessor = arrayOf(gltf, "accessors").size();
    addMeshData(gltf, binary, mesh);

    ObjectNode primitive = primitives.addObject();
    ObjectNode attributes = primitive.putObject("attributes");
    attributes.put("POSITION", firstAccessor);
    attributes.put("NORMAL", firstAccessor + 1);
    primitive.put("indices", firstAccessor + 2);
    primitive.put("material", material);
  }

  /** Adds a material in this color and returns its number. */
  private static int addMaterial(ObjectNode gltf, double[] color) {
    ArrayNode materials = arrayOf(gltf, "materials");
    ObjectNode material = materials.addObject();
    ObjectNode pbr = material.putObject("pbrMetallicRoughness");
    ArrayNode baseColor = pbr.putArray("baseColorFactor");
    for (double channel : color) {
      baseColor.add(channel);
    }
    pbr.put("metallicFactor", METALLIC);
    pbr.put("roughnessFactor", ROUGHNESS);
    return materials.size() - 1;
  }

  /** Writes one box's 6 faces into the mesh. */
  private static void addBox(Box box, Mesh mesh) {
    double[][] corners = {
      {box.minXMeters, box.minYMeters, box.minZMeters},
      {box.maxXMeters, box.maxYMeters, box.maxZMeters}
    };
    for (int face = 0; face < FACE_NORMALS.length; face++) {
      addFace(corners, face, mesh);
    }
  }

  /** Writes one face: 4 corners with their normal, and the 2 triangles between them. */
  private static void addFace(double[][] corners, int face, Mesh mesh) {
    int[] normal = FACE_NORMALS[face];
    float[] glNormal = toGltf(normal[0], normal[1], normal[2]);
    for (int[] pick : FACE_CORNERS[face]) {
      float[] glPosition = toGltf(corners[pick[0]][0], corners[pick[1]][1], corners[pick[2]][2]);
      for (int axis = 0; axis < 3; axis++) {
        mesh.positions.putFloat(glPosition[axis]);
        mesh.normals.putFloat(glNormal[axis]);
        mesh.min[axis] = Math.min(mesh.min[axis], glPosition[axis]);
        mesh.max[axis] = Math.max(mesh.max[axis], glPosition[axis]);
      }
    }
    for (int offset : FACE_TRIANGLES) {
      mesh.indices.putShort((short) (mesh.nextCorner + offset));
    }
    mesh.nextCorner += CORNERS_PER_FACE;
  }

  /** Adds the mesh's positions, normals and triangles, each with an accessor to read it. */
  private static void addMeshData(ObjectNode gltf, ByteArrayOutputStream binary, Mesh mesh) {
    int positionView = addView(gltf, binary, mesh.positions, VERTEX_DATA);
    ObjectNode positions = addAccessor(gltf, positionView, FLOAT_TYPE, mesh.cornerCount, "VEC3");
    // AdvantageScope and the tests read the model's size from these.
    positions.set("min", floats(mesh.min));
    positions.set("max", floats(mesh.max));
    int normalView = addView(gltf, binary, mesh.normals, VERTEX_DATA);
    addAccessor(gltf, normalView, FLOAT_TYPE, mesh.cornerCount, "VEC3");
    int indexView = addView(gltf, binary, mesh.indices, INDEX_DATA);
    addAccessor(gltf, indexView, UNSIGNED_SHORT_TYPE, mesh.indexCount, "SCALAR");
  }

  /** Robot frame (z up) to glTF (y up). Uses 0 - y so 0 stays 0 instead of -0. */
  private static float[] toGltf(double x, double y, double z) {
    return new float[] {(float) x, (float) z, (float) (0.0 - y)};
  }

  /** Adds binary data (padded to a multiple of 4 bytes, as glTF requires) and its "view". */
  private static int addView(
      ObjectNode gltf, ByteArrayOutputStream binary, ByteBuffer data, int target) {
    while (binary.size() % CHUNK_ALIGNMENT_BYTES != 0) {
      binary.write(0);
    }
    ArrayNode views = arrayOf(gltf, "bufferViews");
    ObjectNode view = views.addObject();
    view.put("buffer", 0);
    view.put("byteOffset", binary.size());
    view.put("byteLength", data.capacity());
    view.put("target", target);
    binary.write(data.array(), 0, data.capacity());
    return views.size() - 1;
  }

  /** Adds an "accessor": how to read one view (number type, how many, single or x/y/z). */
  private static ObjectNode addAccessor(
      ObjectNode gltf, int view, int numberType, int count, String shape) {
    ArrayNode accessors = arrayOf(gltf, "accessors");
    ObjectNode accessor = accessors.addObject();
    accessor.put("bufferView", view);
    accessor.put("componentType", numberType);
    accessor.put("count", count);
    accessor.put("type", shape);
    return accessor;
  }

  /** Puts the JSON and the binary data into one .glb file, with the headers the format needs. */
  private static byte[] packGlb(byte[] json, byte[] binary) {
    int jsonLength = padded(json.length);
    int binaryLength = padded(binary.length);
    int total =
        GLB_HEADER_BYTES + CHUNK_HEADER_BYTES + jsonLength + CHUNK_HEADER_BYTES + binaryLength;
    ByteBuffer glb = littleEndian(total);
    glb.putInt(GLB_MAGIC);
    glb.putInt(GLB_VERSION);
    glb.putInt(total);
    glb.putInt(jsonLength);
    glb.putInt(JSON_CHUNK);
    glb.put(json);
    // The JSON chunk is padded with spaces.
    for (int i = json.length; i < jsonLength; i++) {
      glb.put((byte) ' ');
    }
    glb.putInt(binaryLength);
    glb.putInt(BINARY_CHUNK);
    glb.put(binary);
    return glb.array();
  }

  /** The box a whole .glb model fits in, in the robot frame. */
  public static Box readBounds(Path file) throws IOException {
    ByteBuffer glb = ByteBuffer.wrap(Files.readAllBytes(file)).order(ByteOrder.LITTLE_ENDIAN);
    if (glb.getInt(0) != GLB_MAGIC) {
      throw new IOException(file + " is not a .glb file");
    }
    int jsonLength = glb.getInt(GLB_HEADER_BYTES);
    int jsonStart = GLB_HEADER_BYTES + CHUNK_HEADER_BYTES;
    String json = new String(glb.array(), jsonStart, jsonLength, StandardCharsets.UTF_8);
    double[] min = {Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE};
    double[] max = {-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
    JsonNode accessors = JSON.readTree(json.trim()).get("accessors");
    for (JsonNode accessor : accessors) {
      if (accessor.has("min")) {
        JsonNode accessorMin = accessor.get("min");
        JsonNode accessorMax = accessor.get("max");
        for (int axis = 0; axis < 3; axis++) {
          min[axis] = Math.min(min[axis], accessorMin.get(axis).asDouble());
          max[axis] = Math.max(max[axis], accessorMax.get(axis).asDouble());
        }
      }
    }
    // glTF (x, y, z) is robot (x, -z, y).
    return new Box(min[0], -max[2], min[1], max[0], -min[2], max[1]);
  }

  /** The named list in the file, made empty the first time. */
  private static ArrayNode arrayOf(ObjectNode gltf, String name) {
    if (gltf.has(name)) {
      return (ArrayNode) gltf.get(name);
    }
    return gltf.putArray(name);
  }

  /** A JSON list of numbers. */
  private static ArrayNode floats(float[] values) {
    ArrayNode array = JSON.createArrayNode();
    for (float value : values) {
      array.add(value);
    }
    return array;
  }

  /** A size rounded up to a multiple of 4, as glTF chunks require. */
  private static int padded(int size) {
    return (size + CHUNK_ALIGNMENT_BYTES - 1) / CHUNK_ALIGNMENT_BYTES * CHUNK_ALIGNMENT_BYTES;
  }

  /** An empty buffer that writes numbers in glTF's byte order. */
  private static ByteBuffer littleEndian(int size) {
    return ByteBuffer.allocate(size).order(ByteOrder.LITTLE_ENDIAN);
  }
}
