# Rod Studio

Interactive simulator of elastic rods (Kirchhoff rods) implementing **Discrete Elastic Rods** (Bergou et al., SIGGRAPH 2008). Pure C# / .NET 8, OpenGL 4.5 core through hand-written WGL bindings. **Zero external dependencies.** One VS2022 solution, builds out of the box.

This is the physics of cables, hair, DNA supercoiling, drill strings and telephone cords: thin structures where bending and twisting couple through geometry.

## Scenes

| # | Scene | What it demonstrates |
|---|-------|----------------------|
| 1 | Cantilever | Clamped beam under gravity, validated against Euler-Bernoulli theory |
| 2 | Euler buckling | A clamp driven inward past the critical load; the arch pops out of plane |
| 3 | Plectonemes | Twist injected into a slack rod converts to writhe: DNA-style supercoiling with self-contact |
| 4 | Cable drape | Catenary with bending stiffness, swingable pin |
| 5 | Coil drop | A coiled rod with straight rest shape dropped on the floor: unwinding, writhing, ground and self-contact |
| 6 | Whirl | Top clamp cranked in a circle: travelling helical waves |

The stripes on the tube are anchored to the material frame, so you see twist propagate through the rod directly.

## The method

Isotropic Kirchhoff rod (circular cross-section), discretized per Bergou et al.:

- Centerline nodes plus one scalar twist angle per edge. Twist-free reference frames maintained by **time-parallel transport**; the material frame twist is measured against the frame holonomy (reference twist).
- **Quasistatic material frame**: for an isotropic rod the bending energy does not depend on the twist angles, so the twist stationarity conditions are linear. Solved every substep as a tridiagonal system (Thomas algorithm).
- **Exact analytic gradients** of the discrete bending energy (including edge-length variation) and of the reference twist (frame holonomy). Both are verified against central finite differences in the self-test; the bending gradient matches to 4e-9 relative, the twist gradient to 5e-8.
- **Inextensibility by fast projection** (Goldenthal et al. 2007). Edge-length constraints couple only neighbours, so each Newton iteration is one tridiagonal solve. Converges to machine precision in a handful of iterations.
- **Contacts**: ground plane with friction, rod self-contact through a spatial hash over segment AABBs with exact segment-segment closest-point tests (Ericson).

Integration is symplectic Euler with fixed substeps and a hard per-frame budget (the sim goes slow-mo under load instead of spiraling).

## Engineering notes

- The solver runs in `double` (custom `Vec3`, aggressively inlined); conversion to `float` happens only at the render boundary.
- Steady-state the simulation loop allocates nothing: scratch buffers for the tridiagonal solves are reused, the contact hash reuses its cell lists, the Thomas solver uses `stackalloc` for typical rod sizes.
- The tube mesh is rebuilt on the CPU each frame from the centerline and material frames and streamed with buffer orphaning. A rod is a few thousand vertices; this is not the bottleneck and keeps the renderer trivial.
- The OpenGL binding is hand-rolled: every entry point resolved through `wglGetProcAddress` at startup. Window and 4.5 core context created the canonical WGL way (dummy context first, then `wglChoosePixelFormatARB` with MSAA and `wglCreateContextAttribsARB`).
- Planar stencil shadows: overlapping shadow triangles darken the ground exactly once.

## Validation

The physics is testable without a GPU:

```
dotnet run --project src -- --selftest
```

runs on any OS and checks:

1. bending force against finite differences of the discrete energy
2. twist force against finite differences under the frame-transport convention used by the dynamics
3. stationarity of the quasistatic twist solve
4. fast projection convergence to machine precision
5. cantilever tip deflection against the analytic Euler-Bernoulli solution with the exact discrete load distribution (the discrete clamp is first-order accurate in h: the error halves when the resolution doubles)
6. a 20k-substep torture run of the twisted self-contacting rod
7. a smoke run of every shipped scene

## Build and run

Requirements: Windows, .NET 8 SDK. Open `RodStudio.sln` in Visual Studio 2022 and run, or:

```
dotnet run --project src -c Release
```

## Controls

```
1..6        switch scene            LMB drag    orbit
Space       pause                   RMB drag    pan
R           reset scene             wheel       zoom
Left/Right  scene parameter         F           refocus camera
Up/Down     bending stiffness       W           wireframe
T/G         twist stiffness         V           vsync
H           toggle help             Esc         quit
```

## References

- M. Bergou, M. Wardetzky, S. Robinson, B. Audoly, E. Grinspun. *Discrete Elastic Rods.* ACM SIGGRAPH 2008.
- M. Bergou, B. Audoly, E. Vouga, M. Wardetzky, E. Grinspun. *Discrete Viscous Threads.* ACM SIGGRAPH 2010.
- R. Goldenthal, D. Harmon, R. Fattal, M. Bercovier, E. Grinspun. *Efficient Simulation of Inextensible Cloth.* ACM SIGGRAPH 2007.
- C. Ericson. *Real-Time Collision Detection.* Morgan Kaufmann, 2005.

The embedded 8x8 HUD font is public domain (Daniel Hepper / Marcel Sondaar, IBM VGA fonts).

## Roadmap

- Anisotropic cross-sections and natural curvature (full material-frame curvatures; makes the quasistatic twist solve a Newton iteration instead of a single linear solve)
- Multiple interacting rods, knot tying
- Implicit integration of the bending forces for stiffer rods at larger steps

## License

MIT
