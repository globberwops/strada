# ADR 0010: Interactive Route Creation in the 2D Visualizer

## Status
Accepted

## Context
Strada provides a high-performance C++20 library for OpenDRIVE maps. While the visualizer app currently inspects individual lanes and reference lines, it lacks the ability to plan, visualize, and query continuous paths across the road network. To support testing vehicle dynamics along longer paths, we need a way to interactively construct, visualize, and represent a **Route** directly in the 2D visualizer app.

## Decisions

### 1. Route-Level and Waypoint-Based Domain Model
We define the following domain concepts:
* **Waypoint**: A point along the route designated by a clicked location on a drivable lane.
* **Route**: A continuous sequence of roads connecting two or more waypoints, calculated by concatenating shortest paths between consecutive waypoints.
* **Route Segment**: A component of a Route representing travel along a single road in a specific direction (Forward or Backward).

### 2. Directionless Road Routing with Snapped Waypoints
To keep pathfinding simple and robust, we will leverage the existing [routing::Graph](file:///workspaces/strada/include/strada/routing/graph.hpp#L20).
* When the user clicks a lane, we resolve the click to a road ID.
* We perform Dijkstra pathfinding between the start and end road IDs using `Graph::FindRoute`.
* We do not enforce the start/end lane direction at the pathfinding stage (Option A). The routing graph finds the shortest path, and we resolve travel directions (`Forward` or `Backward`) post-search.

### 3. Graph API Extensions (`FindRoute`)
We will extend [routing::Graph](file:///workspaces/strada/include/strada/routing/graph.hpp#L20) to return a structured `Route` containing directional segments:
```cpp
namespace strada::routing {

struct RouteSegment {
  std::string road_id;
  bool forward{true};
  double length{0.0};
};

struct Route {
  std::vector<RouteSegment> segments;
};

// ... inside class Graph ...
auto FindRoute(std::string_view start_road_id, std::string_view end_road_id) const
    -> std::optional<Route>;
}
```

### 4. Continuous $s/t$ Route Coordinate System
A Route establishes its own longitudinal ($s_{route}$) and lateral ($t_{route}$) coordinate system:
* $s_{route} \in [0, L_{route}]$ starting at $0$ at the beginning of the first segment and accumulating lengths of successive segments.
* For a given $s_{route}$ mapping to a segment of length $L$:
  * If the segment is `Forward` (along the road's natural direction):
    * $s_{local} = s_{route} - start\_s$
    * $t_{route} = t_{local}$
  * If the segment is `Backward` (opposite to the road's natural direction):
    * $s_{local} = L - (s_{route} - start\_s)$
    * $t_{route} = -t_{local}$ (to ensure looking in the direction of travel, positive $t_{route}$ is always to the left, and negative $t_{route}$ is always to the right).

### 5. Interaction Flow in ViewportWidget
We will add a modal **Route Creation Mode** to the visualizer:
* **Toggling**: Pressing `P` enters/exits Route Creation Mode.
* **Waypoints**: Left-clicking a drivable lane adds a snapped waypoint.
* **Pan vs. Click**: Disambiguated via a 5-pixel motion threshold between mouse press and release. If the drag distance exceeds 5 pixels, it is handled as a viewport pan; otherwise, it registers as a waypoint click.
* **Undo & Clear**: Pressing `Backspace` or `Delete` undos the last waypoint. Pressing `C` clears the route. `Escape` exits the mode.

### 6. HUD Card Layout and Rendering
* **HUD**: The **Route Planner** HUD card is placed in the top-right corner, offset below the compass gizmo (starting at $y=100$). It displays the list of waypoints, cumulative route length, and keyboard reminders.
* **Path Rendering**: We highlight the drivable lane meshes of the roads in the route using a neon cyan overlay (`ColorA{0.0F, 229.0F/255.0F, 1.0F, 0.4F}`).
* **Waypoint Rendering**: We project each waypoint's world position back to the screen via the camera's `WorldToScreen` function and draw a numbered circular marker overlay via `QPainter`.

## Consequences

### Positive
* **Logical Consistency**: The continuous $s_{route}/t_{route}$ mapping guarantees right-handed coordinates along the direction of travel.
* **User-friendly Controls**: The click vs. drag threshold prevents accidental waypoint placement when moving the map.
* **Low Complexity**: Using road-level routing keeps the Dijkstra search space small while still highlighting detailed drivable lanes in the visualizer.

### Negative
* **Multi-stage Path Concatenation**: Extending routes with multiple waypoints requires concatenating shortest path segments, which could result in backtrack loops if waypoints are placed out of logical network order.
