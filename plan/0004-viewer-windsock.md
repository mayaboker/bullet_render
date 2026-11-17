# Viewer-only windsock update

Goal: update windsock in packaged Godot viewer from simulation wind-state messages.
Steps: save renderer companion plan; add regression; refresh wind visualization before viewer-only control early return; rebuild packaged viewer; run X=1 m/s flight and inspect screenshot.
Interfaces: unchanged topics and ownership; keep command UI disabled in viewer-only mode.
Verification: Godot viewer-only regression and existing wind scene checks, export, live message and screenshot confirmation.
Scope: sibling renderer authorized; no physics changes.
Status: Implemented and verified; uncommitted
Implementation commit: Pending

Outcome: viewer-only regression failed before fix and passed after; Godot wind scene and wind unit checks passed; packaged viewer rebuilt; 24 Python checks passed. Live X=1.0 m/s wind screenshot /tmp/wind-x1-render-fixed.png shows Wind 1.0 m/s. Full flight completed with cleanup. Full workspace suite not rerun.
