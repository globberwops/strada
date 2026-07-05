# ADR 0007: Tessellator Layer and 2D Visualizer Architecture

## Status
Accepted

## Context
Strada needs an example application to preview, inspect, and verify loaded road maps. This requires a lightweight, performant 2D visualizer built with Qt6 and OpenGL.

To support rendering, we also need to implement the planned (but currently missing) **Tessellator** library layer, which discretizes the map's continuous mathematical curves (reference lines, lane boundaries, elevation profiles, and widths) into renderable lines and triangles.

The system must handle viewport manipulation (pan, zoom, rotate), display contextual HUD data (such as road/lane identification under the cursor), and provide clear spatial orientation indicators (a rotating orientation gizmo and a segmented geographic scale bar).

## Decisions

### 1. Reusable `strada_tess` Library Layer
Rather than hardcoding tessellation inside the visualizer, we will build a reusable library target `strada_tess` with the following structure:
* **Headers**: [include/strada/tess/tessellator.hpp](file:///workspaces/strada/include/strada/tess/tessellator.hpp)
* **API Factory**:
  `auto BuildTessellator(const ast::AbstractSyntaxTree& map, double chord_error) -> Tessellator;`
* **Output Structures**:
  * `tess::Vertex`: 32-bit floats `(x, y, z)` for GPU buffer friendliness.
  * `tess::Mesh`: Static index and vertex buffers representing lane/road surfaces, carrying `road_id`, `lane_id`, and `lane_type` (e.g. driving, shoulder, sidewalk) for styling and picking.
  * `tess::Polyline`: Vertex coordinates representing line markers, carrying `road_id`, `is_reference_line`, and `marking_type` (e.g. solid, broken) for styled boundary rendering.

### 2. Discretization via Temporary CPM Compilation
To avoid replicating complex clothoid mathematics (Fresnel integrals), lane offsets, and superelevation calculations inside the tessellator, the `BuildTessellator` factory will:
1. Compile a temporary, local `CompiledPhysicsModel` (CPM) from the input AST.
2. Evaluate stations and sample coordinates $(X, Y, Z)$ using the CPM's query interfaces.
3. Discard the CPM once the tessellated mesh lists are finalized.
This maintains dry codebase principles while ensuring the `Tessellator` layer remains completely decoupled and owns its static data.

### 3. Batched Graphics Pipeline (2 Draw Calls)
To maximize rendering performance, the visualizer widget (based on `QOpenGLWidget`):
* Combines all static lane and road surface meshes into a single large Vertex Buffer Object (VBO) and Index Buffer Object (IBO). These are rendered in a single `GL_TRIANGLES` draw call.
* Batches all static lane boundaries, markings, and reference lines into a second VBO rendered via a single `GL_LINES` or `GL_LINE_STRIP` draw call.
This minimizes CPU-GPU state changes and draw call overhead.

### 4. CPU-Side Interactive Picking via CPM
Instead of complex GPU picking or holding separate geometry nodes for every lane, the visualizer leverages Strada's high-speed spatial index:
* On cursor movement, the visualizer maps screen coordinates to world $(X, Y)$ coords.
* It queries the CPM CPU-side using `InertialToLane` (which is $O(\log N)$ and operates in sub-milliseconds).
* The picked `RoadId` and `LaneId` are displayed in the HUD, and the matching lane's tessellated geometry is rendered dynamically as a highlight overlay.

### 5. Google Maps Navigation Style
The visualizer will implement a single standard navigation profile:
* **Pan**: Left-click and drag.
* **Zoom**: Scroll wheel (centered on the mouse cursor position).
* **Rotate**: Right-click and drag.
* **Reset**: Pressing the `R` key resets the camera center, zoom, and rotation.

### 6. Cartographic Scale Bar and Rotating Gizmo (QPainter HUD)
Text rendering and UI overlays will be handled by Qt's `QPainter` drawing directly on top of the `QOpenGLWidget` during paint calls:
* **Orientation Gizmo**: A static $60\times60$ px corner widget indicating East ($X$) and North ($Y$) directions, rotated dynamically with the map's view.
* **Geographical Scale Bar**: A segmented bar divided into 4-5 alternating filled (dark) and non-filled (light) blocks. Its physical length $L$ adjusts dynamically (e.g. 10m, 50m, 100m) to keep the screen width between 80px and 150px based on the zoom factor.

## Consequences

### Positive
* **Optimal Rendering Performance**: Grouping static map geometry into two draw calls avoids CPU-GPU bottlenecks.
* **Math Utility Reuse**: Leveraging CPM prevents duplicating Fresnel integrals and polynomial evaluation.
* **Low Coupling**: The tessellator and visualizer are consumers of the AST/CPM layers, maintaining clean software boundaries.
* **Cartographic Fidelity**: The alternating segmented scale bar and rotating gizmo provide high-quality map inspection tools.

### Negative
* **Temporary Allocation**: Building the Tessellator incurs the temporary memory and CPU cost of compiling the CPM on startup (typically under a few hundred milliseconds).
