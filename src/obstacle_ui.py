# Obstacle palette UI (docs/OBSTACLES.md)
# Fixed panel in the top-right corner, immediately left of the minimap.
# Interactions use the LEFT mouse button only (middle-mouse pan and scroll
# zoom are untouched; left click was previously unused in the game window):
#   1. Place:  press a palette slot -> ghost car follows the cursor with its
#              final heading live-updating; release over the paved area ->
#              placed, off-road or back over the palette -> cancelled.
#   2. Move:   press an obstacle in the world -> picked up (same ghost);
#              release over the paved area -> re-placed + re-aligned there.
#   3. Delete: drag a world obstacle onto the trashcan (it highlights while
#              one hovers over it). Dropping a fresh palette item into the
#              trashcan is simply a cancel.
# While dragging, the ghost is drawn at full size with reduced opacity; an
# invalid drop target (off-road) tints it red so the rejection on release is
# never surprising.

from __future__ import annotations

import pygame

from . import config
from .obstacles import (OBSTACLE_COLORS, GHOST_INVALID_RGB, PlacementError,
                        ObstacleManager, obstacle_footprint, point_in_box,
                        tinted_car_sprite)


class ObstaclePalette:
    PANEL_W = 150
    SLOT_W = 42
    SLOT_H = 56
    SLOT_LABELS = {"blue": "blu", "yellow": "yel", "white": "wht"}
    LOAD_ROW_H = 18
    MAX_LOAD_ROWS = 8

    def __init__(self, manager: ObstacleManager, network, camera, renderer):
        self.manager = manager
        self.network = network
        self.camera = camera
        self.renderer = renderer

        # Drag state: None | ("new", color) | ("move", ob_id)
        self.drag = None
        self.cursor = (0, 0)          # last known screen position
        self.save_mode = False        # layout-name input field has focus
        self.save_name = ""
        self.load_mode = False        # show the saved-layouts list
        self._load_entries: list[str] = []
        self._msg = None              # flash text
        self._msg_until = 0           # pygame ticks

    # --- Layout (recomputed per draw: the window is resizable) --------------

    def _minimap_rect(self) -> pygame.Rect:
        return pygame.Rect(
            self.camera.width - config.MINIMAP_SIZE - config.MINIMAP_MARGIN,
            config.MINIMAP_MARGIN,
            config.MINIMAP_SIZE, config.MINIMAP_SIZE)

    def _layout(self) -> dict:
        w = self.camera.width
        mm_x = w - config.MINIMAP_SIZE - config.MINIMAP_MARGIN
        px = mm_x - 6 - self.PANEL_W
        py = config.MINIMAP_MARGIN
        lay: dict = {}

        slot_y = py + 30
        total = 3 * self.SLOT_W + 2 * 4
        x0 = px + (self.PANEL_W - total) // 2
        lay["slots"] = {
            color: pygame.Rect(x0 + i * (self.SLOT_W + 4), slot_y,
                               self.SLOT_W, self.SLOT_H)
            for i, color in enumerate(("blue", "yellow", "white"))
        }

        y = slot_y + self.SLOT_H + 8
        bw = (self.PANEL_W - 16 - 6) // 2
        lay["save_btn"] = pygame.Rect(px + 8, y, bw, 22)
        lay["load_btn"] = pygame.Rect(px + 8 + bw + 6, y, bw, 22)
        y += 22 + 8
        tw = 36
        lay["trash"] = pygame.Rect(px + (self.PANEL_W - tw) // 2, y, tw, 26)
        y += 26 + 6

        if self.save_mode:
            lay["input"] = pygame.Rect(px + 8, y, self.PANEL_W - 16, 20)
            y += 20 + 14
        if self.load_mode:
            n = min(len(self._load_entries), self.MAX_LOAD_ROWS)
            lay["load_rows"] = [
                pygame.Rect(px + 8, y + i * self.LOAD_ROW_H,
                            self.PANEL_W - 16, self.LOAD_ROW_H - 2)
                for i in range(n)
            ]
            y += max(n, 1) * self.LOAD_ROW_H + (10 if n == 0 else 2)

        lay["panel"] = pygame.Rect(px, py, self.PANEL_W, y - py + 6)
        return lay

    # --- Input ---------------------------------------------------------------

    def text_input_active(self) -> bool:
        """True while the layout-name field has focus (the game's keyboard
        shortcuts must stay quiet then - typing 'b' must not toggle the
        breadcrumb trail)."""
        return self.save_mode

    def handle_event(self, event) -> bool:
        """Returns True if the event was consumed (NOT passed to the camera).
        Only the LEFT mouse button is ever consumed; middle-button panning
        and scroll zoom go through untouched."""
        if event.type == pygame.MOUSEMOTION:
            self.cursor = event.pos
            return False
        if not hasattr(event, "button") or event.button != 1:
            return False

        lay = self._layout()
        pos = event.pos

        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.save_mode:
                return True                     # field has focus: swallow clicks
            if self.load_mode:
                for i, rect in enumerate(lay.get("load_rows", [])):
                    if rect.collidepoint(pos):
                        self._do_load(self._load_entries[i])
                        return True
                # LOAD button toggles the list; any other click closes it.
                self.load_mode = False
                return True

            for color, rect in lay["slots"].items():
                if rect.collidepoint(pos):
                    self.drag = ("new", color)
                    self.cursor = pos
                    return True
            if lay["save_btn"].collidepoint(pos):
                self.save_mode = True
                self.save_name = ""
                return True
            if lay["load_btn"].collidepoint(pos):
                self._load_entries = self.manager.list_layouts()
                self.load_mode = not self.load_mode
                return True

            # World: pick up an existing obstacle (topmost first).
            wx, wy = self.camera.screen_to_world(*pos)
            for ob in reversed(self.manager.snapshot()):
                if point_in_box(wx, wy, obstacle_footprint(ob)):
                    self.drag = ("move", ob.id)
                    return True
            return False

        if event.type == pygame.MOUSEBUTTONUP:
            if self.drag is None:
                return False
            kind, payload = self.drag
            self.drag = None

            over_trash = lay["trash"].collidepoint(pos)
            over_ui = (lay["panel"].collidepoint(pos)
                       or self._minimap_rect().collidepoint(pos))
            if kind == "move" and over_trash:
                self.manager.remove(payload)
                self._flash(f"Obstacle {payload} deleted")
                return True
            if over_ui:
                return True                     # cancelled - nothing placed

            wx, wy = self.camera.screen_to_world(*pos)
            try:
                if kind == "new":
                    ob = self.manager.place(self.network, "car", payload, wx, wy)
                    self._flash(f"Obstacle {ob.id} placed ({payload})")
                else:
                    self.manager.move(self.network, payload, wx, wy)
                    self._flash(f"Obstacle {payload} moved")
            except (PlacementError, KeyError):
                pass                            # ghost was red - rejection expected
            return True

        return False

    def handle_keydown(self, event) -> bool:
        """KEYDOWN events; True if consumed (only while the name field is up)."""
        if not self.save_mode:
            return False
        k = event.key
        if k in (pygame.K_RETURN, pygame.K_KP_ENTER):
            name = self.save_name.strip()
            if name:
                try:
                    self.manager.save(name)
                    self._flash(f"Saved layout '{name}'")
                except PlacementError as e:
                    self._flash(str(e))
            self.save_mode = False
            return True
        if k == pygame.K_BACKSPACE:
            self.save_name = self.save_name[:-1]
            return True
        if k == pygame.K_ESCAPE:
            self.save_mode = False
            return True
        return False

    def handle_textinput(self, event) -> bool:
        """TEXTINPUT events; True if consumed."""
        if not self.save_mode:
            return False
        text = event.text
        if all(ch.isprintable() for ch in text):
            self.save_name = (self.save_name + text)[:60]
        return True

    def _do_load(self, name: str):
        try:
            loaded, skipped = self.manager.load(name, self.network)
            msg = f"Loaded '{name}': {loaded} obstacle(s)"
            if skipped:
                msg += f", {skipped} skipped (off-road)"
            self._flash(msg)
        except (FileNotFoundError, PlacementError, OSError, ValueError) as e:
            self._flash(f"Load failed: {e}")
        finally:
            self.load_mode = False

    def _flash(self, text: str):
        self._msg = text
        self._msg_until = pygame.time.get_ticks() + 3000

    # --- Drawing ---------------------------------------------------------------

    def draw_world(self, surface: pygame.Surface):
        """Obstacles in world space: above road markings, below car/HUD/
        minimap/palette. They do not appear on the minimap (for now)."""
        cam = self.camera
        skip_id = self.drag[1] if (self.drag and self.drag[0] == "move") else None
        for ob in self.manager.snapshot():
            if ob.id == skip_id:
                continue                        # drawn as ghost at the cursor
            self._draw_car(surface, cam, ob.x, ob.y, ob.heading,
                           OBSTACLE_COLORS[ob.color], 255)

        # Ghost while dragging - only over the map (not over the palette or
        # minimap), with its final heading live-updating under the cursor.
        if self.drag is not None:
            kind, payload = self.drag
            if kind == "new":
                ghost_color = OBSTACLE_COLORS[payload]
            else:
                ob = self.manager.get(payload)
                ghost_color = OBSTACLE_COLORS[ob.color] if ob is not None else None
            if ghost_color is None:
                return                          # moved obstacle vanished mid-drag
            lay = self._layout()
            if not (lay["panel"].collidepoint(self.cursor)
                    or self._minimap_rect().collidepoint(self.cursor)):
                wx, wy = cam.screen_to_world(*self.cursor)
                valid = self.network.is_on_road(wx, wy)
                try:
                    heading = self.manager.align_heading(wx, wy, self.network)
                except PlacementError:
                    return                      # no road at all - nothing to show
                rgb = ghost_color if valid else GHOST_INVALID_RGB
                self._draw_car(surface, cam, wx, wy, heading, rgb, 140)

    def _draw_car(self, surface: pygame.Surface, cam, wx: float, wy: float,
                  heading_deg: float, rgb: tuple[int, int, int], alpha: int):
        """Same sprite pipeline as the player car (scale with zoom, rotate
        to heading), centered on (wx, wy)."""
        scale = cam.zoom
        w_px = int(config.CAR_WIDTH * config.PIXELS_PER_METER * scale)
        l_px = int(config.CAR_LENGTH * config.PIXELS_PER_METER * scale)
        if w_px < 2 or l_px < 2:
            return
        base = tinted_car_sprite(rgb, alpha)
        scaled = pygame.transform.scale(base, (w_px, l_px))
        rotated = pygame.transform.rotate(scaled, -heading_deg)
        sx, sy = cam.world_to_screen(wx, wy)
        surface.blit(rotated, rotated.get_rect(center=(int(sx), int(sy))))

    def draw_panel(self, surface: pygame.Surface):
        """The palette box itself (topmost layer)."""
        lay = self._layout()
        panel = lay["panel"]
        pygame.draw.rect(surface, (20, 20, 20), panel)
        pygame.draw.rect(surface, (100, 100, 100), panel, 2)

        surface.blit(self._text("obst_title", "OBSTACLES", 14, (230, 230, 230)),
                     (panel.x + 8, panel.y + 7))

        # Slots: one static car per color.
        for color, rect in lay["slots"].items():
            hovered = (self.cursor is not None and self.drag is None
                       and not self.save_mode and not self.load_mode
                       and rect.collidepoint(self.cursor))
            pygame.draw.rect(surface, (48, 48, 52) if hovered else (32, 32, 36), rect)
            pygame.draw.rect(surface, OBSTACLE_COLORS[color], rect, 1)
            icon = self._icon(color)
            surface.blit(icon, icon.get_rect(center=(rect.centerx, rect.y + 20)))
            label = self._text(f"obst_slot_{color}", self.SLOT_LABELS[color],
                               10, (200, 200, 200))
            surface.blit(label, (rect.centerx - label.get_width() // 2, rect.y + 41))

        # SAVE / LOAD buttons.
        for btn, label in ((lay["save_btn"], "SAVE"), (lay["load_btn"], "LOAD")):
            pygame.draw.rect(surface, (50, 50, 55), btn)
            pygame.draw.rect(surface, (120, 120, 125), btn, 1)
            t = self._text(f"obst_btn_{label}", label, 12, (230, 230, 230))
            surface.blit(t, t.get_rect(center=btn.center))

        # Trashcan (highlights while a dragged obstacle hovers over it).
        tr = lay["trash"]
        hovering = (self.drag is not None and self.drag[0] == "move"
                    and self.cursor is not None and tr.collidepoint(self.cursor))
        self._draw_trash(surface, tr, (255, 200, 60) if hovering else (150, 150, 155))

        # Save: name input field.
        if self.save_mode:
            inp = lay["input"]
            pygame.draw.rect(surface, (10, 10, 12), inp)
            pygame.draw.rect(surface, (200, 200, 80), inp, 1)
            caret = "▌" if (pygame.time.get_ticks() // 400) % 2 == 0 else ""
            shown = self.save_name + caret
            t = self._text("obst_input", shown if shown else "_", 12, (240, 240, 240))
            surface.blit(t, (inp.x + 4, inp.y + 3))
            hint = self._text("obst_hint", "ENTER save · ESC cancel", 9, (150, 150, 150))
            surface.blit(hint, (panel.centerx - hint.get_width() // 2, inp.bottom + 2))

        # Load: clickable list of this map's saved layouts.
        if self.load_mode:
            rows = lay.get("load_rows", [])
            if not self._load_entries:
                t = self._text("obst_nolayouts", "(no saved layouts)", 10, (160, 160, 160))
                surface.blit(t, (panel.x + 10, panel.bottom - 24))
            for i, rect in enumerate(rows):
                hovered = self.cursor is not None and rect.collidepoint(self.cursor)
                if hovered:
                    pygame.draw.rect(surface, (60, 60, 70), rect)
                t = self._text(f"obst_layout_{i}", self._load_entries[i][:22],
                               11, (235, 235, 235))
                surface.blit(t, (rect.x + 4, rect.y + 1))
            if len(self._load_entries) > len(rows):
                t = self._text("obst_more",
                               f"+{len(self._load_entries) - len(rows)} more",
                               9, (150, 150, 150))
                surface.blit(t, (panel.x + 10, rows[-1].bottom + 2))

        # Flash message (top-center banner, above everything).
        if self._msg is not None:
            if pygame.time.get_ticks() < self._msg_until:
                t = self._text("obst_msg", self._msg, 12, (255, 230, 120))
                bx = (surface.get_width() - t.get_width()) // 2
                pygame.draw.rect(surface, (20, 20, 20),
                                 (bx - 8, 44, t.get_width() + 16, t.get_height() + 10))
                pygame.draw.rect(surface, (120, 120, 125),
                                 (bx - 8, 44, t.get_width() + 16, t.get_height() + 10), 1)
                surface.blit(t, (bx, 49))
            else:
                self._msg = None

    def _icon(self, color: str) -> pygame.Surface:
        """Small car icon for a palette slot (cached)."""
        key = f"icon_{color}"
        if not hasattr(self, "_icons"):
            self._icons = {}
        if key not in self._icons:
            self._icons[key] = pygame.transform.scale(
                tinted_car_sprite(OBSTACLE_COLORS[color]), (16, 32))
        return self._icons[key]

    def _draw_trash(self, surface: pygame.Surface, rect: pygame.Rect, color):
        # Lid + handle
        pygame.draw.rect(surface, color, (rect.x + 4, rect.y, rect.w - 8, 4))
        pygame.draw.rect(surface, color, (rect.centerx - 5, rect.y - 3, 10, 3))
        # Body (trapezoid) with ribs
        body = [(rect.x + 6, rect.y + 6), (rect.right - 6, rect.y + 6),
                (rect.right - 9, rect.bottom), (rect.x + 9, rect.bottom)]
        pygame.draw.polygon(surface, color, body)
        for fx in (0.35, 0.5, 0.65):
            x_top = rect.x + 6 + (rect.w - 12) * fx
            x_bot = rect.x + 9 + (rect.w - 18) * fx
            pygame.draw.line(surface, (30, 30, 34),
                             (x_top, rect.y + 7), (x_bot, rect.bottom - 1), 2)

    def _text(self, key: str, text: str, size: int, color):
        """Panel text via the renderer's cached PIL text helper."""
        return self.renderer._hud_text(f"obst_{key}", text, size, color)
