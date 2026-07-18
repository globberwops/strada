# Strada

You use the Strada library to compile ASAM OpenDRIVE 1.9 (`.xodr`) files into structured in-memory representations for vehicle dynamics simulation, pathfinding, and visualization.

## Architecture

You access and query map layers through the unified `strada::Strada` facade class. The library exposes four main components:

* **AST Parser**: You parse OpenDRIVE XML schemas into typed C++ objects.
* **Compiled Physics Model (CPM)**: You query coordinate transformations and poses at 1 kHz in thread-safe loops over a flat, cache-friendly Struct-of-Arrays (SoA) layout.
* **Routing Graph**: You run A* and Dijkstra pathfinding over the topological lane network.
* **Tessellator**: You generate 3D meshes and boundary polylines from the map's continuous mathematical curves.

## Getting Started

### Build from Source

You need a compiler with C++20 support, CMake 3.22+, and Qt6 for the visualizer.

Configure the build directory:
```bash
cmake --preset default-debug
```

Build the library and the visualizer application:
```bash
cmake --build --preset default-debug -j$(nproc)
```

Run the unit tests:
```bash
ctest --test-dir build/default --output-on-failure
```

### Initialize the Facade

Instantiate the unified class to load and compile a map:

```cpp
#include <strada/strada.hpp>

// Set chord_error to std::nullopt to disable Tessellator generation.
strada::Strada::Options options{.chord_error = 0.5};
strada::Strada map("path/to/map.xodr", options);

// Query individual layers
const auto& ast = map.AbstractSyntaxTree();
const auto& cpm = map.CompiledPhysicsModel();
const auto& graph = map.Graph();

if (map.Tessellator().has_value()) {
  const auto& tessellator = map.Tessellator()->get();
}
```

### Visualizer Example

You can inspect roads, view coordinate transformations, and plan routes with the interactive visualizer application. The visualizer runs on OpenGL and features a Rosé Pine theme.

Run the visualizer with a map file:
```bash
./build/default/examples/visualizer/visualizer tests/data/roads.xodr
```

## Documentation

* [CONTRIBUTING.md](CONTRIBUTING.md): You can find commit conventions, C++ style rules, and pre-commit hook setups.
* [CONTEXT.md](CONTEXT.md): You can find details on reference frames, poses, coordinate transformations, and the domain glossary.

## License

The project uses the Boost Software License 1.0.
