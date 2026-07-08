# Strada Idea Backlog

This file tracks raw, un-fleshed-out feature ideas and architectural improvements. When an idea is ready to be implemented, run `/grill-with-docs` or `/wayfinder` to flesh it out into a spec and publish implementation tickets.

## Active Ideas

### 1. Map Facade
- **Problem**: Downstream consumers must manually instantiate and chain individual `Build*` factories (CPM, Routing Graph, Tessellator) from a parsed AST.
- **Rough Solution**: Build a single entry point (the `MapFacade`) that coordinates these factory calls in a single convenience interface. The facade should simply convenience the calls without imposing extra policy or locking individual layers together.
- **Next Step**: Run a [/grill-with-docs](.agents/skills/grill-with-docs/SKILL.md) session to design the facade's API signature and verify composition boundaries.

### 2. SIMD Batch Queries for CPM
- **Problem**: Evaluators (like vehicle dynamics engines) querying contact points for multiple tires or sensors suffer overhead from iterative single-pose coordinate lookup.
- **Rough Solution**: Implement a batch coordinate transformation API in `strada::cpm` taking a `std::span` of inputs and outputs to leverage CPU vector registers (SoA alignment is already in place).
- **Next Step**: Run a [/prototype](.agents/skills/prototype/SKILL.md) session to benchmark vectorization gains on a batch coordinate lookup versus sequential lookups.

### 3. German SVG Road Signs Integration
- **Problem**: OpenDRIVE maps define German road signals and signs, but the Tessellator and visualization systems lack high-quality SVG assets to render them accurately.
- **Rough Solution**: Integrate SVG road signs from the [Wikimedia Germany Road Signs Category](https://commons.wikimedia.org/wiki/Category:SVG_road_signs_in_Germany) into the visualization pipeline.
- **Next Step**: Run a [/research](.agents/skills/research/SKILL.md) session to identify which sign types map to the OpenDRIVE 1.9 standard and inventory available SVG assets.

### 4. Switch Between Lane Layers
- **Problem**: Downstream consumers (e.g. driver models vs rendering engines) may need to query or visualize different lane representations (e.g., topological lane graph vs physical SoA CPM lanes vs tessellated meshes) dynamically.
- **Rough Solution**: Add API support to swap or toggle active/visible lane layers in the rendering and querying systems.
- **Next Step**: Run a [/grill-with-docs](.agents/skills/grill-with-docs/SKILL.md) session to define the requirements and target layers.


---

## Graduated / Completed Ideas

*No graduated ideas yet. When an idea is turned into a spec and tickets, move it here and link to the spec/tickets.*
