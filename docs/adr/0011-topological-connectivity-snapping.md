# ADR 0011: Topological Connectivity Snapping in the Compiled Physics Model

## Status
Accepted

## Context
Strada's Compiled Physics Model (CPM) exposes hot-path coordinate snapping and transformations (`InertialToRoad` and `InertialToLane`). These snapping queries utilize a thread-local `QueryContext` to exploit temporal coherence on the `last_road`.

However, when the vehicle exits the boundary of the `last_road`, the temporal cache check fails, and the snapping algorithm falls back directly to a global search over the Bounding Volume Hierarchy (BVH) spatial index. On complex road networks (such as a 5-road junction), a global BVH search takes up to $2.3\text{ ms}$, exceeding the $1\text{ ms}$ budget of a $1000\text{ Hz}$ simulator step.

To prevent these transition latency spikes and protect the real-time simulation budget, we must utilize the topological connectivity of roads (predecessors, successors, and junction connections) to resolve the next road locally before resorting to a global BVH search.

## Decisions

### 1. Flat Adjacent Neighbor Tables in CPM (CSR Format)
To keep the CPM self-contained and decoupled from the logical routing graph layer (which contains higher-level Dijkstra logic and travel directions), the CPM compiles and stores its own flat adjacency tables at build time:
*   `std::vector<std::uint32_t> road_neighbor_first_idx;`
*   `std::vector<std::uint32_t> road_neighbor_count;`
*   `std::vector<RoadId> flat_neighbors;`

### 2. Boundary-Specific Adjacency Lists
Connections are divided into two distinct lists per road to reflect physical boundary transitions:
*   `start_neighbors`: Roads connected at $s = 0$ (predecessors), sorted with the priority path first.
*   `end_neighbors`: Roads connected at $s = \text{Length}$ (successors), sorted with the priority path first.

### 3. Dual-Mode Snapping: Route-Guided and Route-Free
We implement two modes of snapping to support both routed and unrouted actors:
1.  **Route-Guided (Primary)**: The consumer can pass an `active_route` list of `RoadId`s and an `active_route_index` inside `QueryContext`. The query restricts snapping search to the current and next route roads, automatically advancing the index in the context when the transition is successful.
2.  **Route-Free (Fallback)**: When no route is specified, the query selects the neighbor list based on vehicle $s$-closeness on the `last_road`:
    *   If $s_{\text{last}} > \text{Length} / 2$, check the `end_neighbors` list first.
    *   If $s_{\text{last}} \le \text{Length} / 2$, check the `start_neighbors` list first.

### 4. Junction Sibling Fallback
To resolve overlapping paths at junction entrances (where multiple connecting roads share the same starting point), if the cache check on a junction connecting road fails, the algorithm checks sibling roads of that junction entrance before falling back to the global BVH.

### 5. Geometric Alignment Priority Heuristic
The CPM compiler determines the priority fallback path at junctions by computing the heading change ($\Delta\text{heading}$) between the incoming road's exit tangent and each candidate connecting road's entrance tangent. The candidate with the smallest heading change (closest to $0$ radians, representing "going straight") is placed first in the neighbor list.

### 6. Graceful Local-to-Global Fallback
If the vehicle deviates from the route or the local neighbor check fails, the query gracefully performs the global BVH search to re-localize the vehicle. If a route-guided query goes off-route, route-guided tracking is suspended.

## Consequences

### Positive
*   **No Transition Latency Spikes**: Spikes at road transitions are eliminated, capping transitions to cheap local projection checks ($< 10\text{ }\mu\text{s}$) rather than full global searches ($2.3\text{ ms}$).
*   **API Compatibility**: The transformation signatures are unchanged because route and caching details are self-contained inside `QueryContext`.
*   **Decoupled Architecture**: CPM remains self-contained and does not link against the logical routing graph.

### Negative
*   **Compiled Size Overhead**: Storing adjacency indices for every road introduces minor memory overhead, though negligible ($\approx 24$ bytes per road).
*   **Static Adjacency Compilation**: Dynamic changes to road connections at runtime are not supported (which matches OpenDRIVE's static map design).
