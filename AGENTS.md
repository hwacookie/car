# Project Rules

## General rule: never do physically impossible things

The simulation must never produce motion that no real car could perform:
no teleportation, no instant heading changes, no turning radius below a
real car's mechanical minimum, no position/heading updates that fall out
of sync. This applies to game logic, physics, fallbacks, and edge cases
(junctions, dead ends, segment hand-offs) alike.

When a desired behavior would require physically impossible motion, fix
the underlying logic (e.g. brake earlier, blend offsets, smooth the
heading) instead of weakening or bypassing the checks that catch it.

This rule may ONLY be violated if the user explicitly asks for it.
