# Fahrmanöver – Spezifikation

Alle Manöver beschreiben, wie das Auto sich in der Simulation verhält.
Die Referenzlinie (BicycleNav) steuert Spurposition und Geschwindigkeit;
der Driver setzt Blinker und Intent (Gas / Bremse).

---

## 1. Parken (Pull-Over)

**Auslöser:** Rechtsblinker + Bremse (automatisch bei `PARK_DISTANCE_M ≤ 50 m` zum Ziel, oder manuell per Tastatur/API).

| Phase | Beschreibung |
|-------|-------------|
| **Einleitung** | Blinker rechts an. Gas weg, Bremse an. Geschwindigkeit sinkt auf ~5–10 km/h. |
| **Verschwenken nach rechts** | Referenzlinie blendet von Normalposition zum rechten Straßenrand (`PARK_BLEND_START_M` → `PARK_BLEND_END_M`). Das Auto fährt eine sanfte Rechtskurve zum Rand. |
| **Parallel ausrichten** | Im letzten Abschnitt (`PARK_BLEND_END_M ≈ 12 m`) ist die Referenzlinie konstant am Rand – das Auto fährt geradeaus parallel zum Straßenrand. Räder werden parallel ausgerichtet (lateral error < `PARK_ALIGN_LATERAL_M`). |
| **Anhalten** | Geschwindigkeit geht auf 0. Auto steht mit den Rädern parallel und möglichst nah am rechten Rand. |
| **Blinker aus** | Sobald das Auto gestoppt ist (`speed < 0.1 m/s` und `dist_to_dest < 1.0 m`), wird der Blinker automatisch ausgeschaltet. |

### Parameter

- `PARK_DISTANCE_M = 50.0` – Ab hier wird Parken eingeleitet
- `PARK_BLEND_START_M = 40.0` – Verschwenken beginnt (m vom Ende)
- `PARK_BLEND_END_M = 12.0` – Verschwenken abgeschlossen, gerade Linie zum Rand
- `PARK_TRACK_LOOKAHEAD_M` – Kurzer Lookahead für enges Tracking der Referenzlinie
- `PARK_ALIGN_LATERAL_M` – Laterale Toleranz, ab der Räder parallel ausgerichtet werden

---

## 2. Anfahren (Pull-Out)

**Auslöser:** Erster Frame nach Spawn/Teleport (`_pull_out_done == False`). Der Driver setzt automatisch **Linker Blinker** + Gas.

| Phase | Beschreibung |
|-------|-------------|
| **Einleitung** | Linker Blinker an. Gas an, Bremse weg. Geschwindigkeit auf ~18 km/h begrenzt. Lookahead auf 1.5m reduziert (maximaler Lenkeinschlag). |
| **Links in die Spur** | Referenzlinie blendet vom rechten Rand (`edge_offset`) zur Normalposition (`base_offset`). Das Auto lenkt links und verschwenkt in die rechte Fahrspur. |
| **Normalposition erreicht** | Nach einem Frame ist `_pull_out_done = True`. Blinker aus, Speed-Limit weg, Lookahead normal. Auto beschleunigt auf Cruising-Speed. |

### Parameter

- `PULL_OUT_START_M = 20.0` – Blend-Zone-Ende (m vom Route-Start)
- `PULL_OUT_END_M = 5.0` – Blend-Zone-Anfang (m vom Route-Start)
- Pull-out ist nur **ein Frame** aktiv – danach fährt das Auto normal

---

## 3. Geradeausfahren

**Auslöser:** Kein Blinker aktiv, kein Parken, kein Anfahren.

| Phase | Beschreibung |
|-------|-------------|
| **Normalposition** | Referenzlinie in `base_offset` (rechte Fahrspur, mittig im Lane). Auto beschleunigt auf Cruise-Speed (`A_CRUISE`) oder bremst bei Kurven (`A_BRAKE`). |
| **Kurven** | Speed-Profile drosselt Geschwindigkeit vor Kurven. Pure Pursuit-Steuerung mit speed-abhängigem Lookahead. |

---

## 4. Abbiegen (Links / Rechts)

**Auslöser:** Blinker links/rechts (manuell per Taste oder API). `_intended_turn()` liest `pending_turn` aus dem Driver.

| Phase | Beschreibung |
|-------|-------------|
| **Einleitung** | Blinker in Abbiegerichtung an. |
| **Spurwechsel zum Außenrand** | Vor der Kreuzung (`TURN_OFFSET_FAR_M = 25 m`) blendet die Referenzlinie zur außenliegenden Position: Linksabbieger → weit außen, Rechtsabbieger → Mittelwert zwischen `base_offset` und `max_offset`. Blend läuft symmetrisch vor **und nach** der Kurve. |
| **Kurve** | Speed-Profile drosselt stark (bei enger Krümmung > 0.15: `A_LAT_MAX` auf 60% reduziert). Tight Lookahead (`2.5 + 0.2*v`). Auto durchfährt die Kreuzung auf der vorgeblendeten Referenzlinie. |
| **Zurück in Normalposition** | Nach der Kreuzung (`TURN_OFFSET_FAR_M` hinter dem Punkt) blendet die Linie zurück auf `base_offset`. |
| **Blinker aus** | `clear_blinker_if_turned()` prüft den Winkel des Kurvenausgangs und schaltet den Blinker ab. |

### Parameter

- `TURN_OFFSET_FAR_M = 25.0` – Blend beginnt (m vor/nach Kreuzung)
- `TURN_OFFSET_NEAR_M = 5.0` – Blend abgeschlossen
- Linksabbieger: `turn_offset = max_offset` (außenrand)
- Rechtsabbieger: `turn_offset = (base_offset + max_offset) / 2` (Mittelwert)
- Lookahead: `2.5 + 0.2 * speed` (basis), bei enger Kurve (`k > 0.15`): `A_LAT_MAX *= 0.6`

---

## 5. Wenden (U-Turn On-Site)

**Auslöser:** Kommando "wenden" (API/Tastatur). Blinker links an.

Das System wählt die Strategie automatisch basierend auf der Straßenbreite:
- **Breite Straße** (`Straßenbreite ≥ Wendekreisradius ≈ 11 m`): Einmaliges Wenden (5a)
- **Schmale Straße** (`Straßenbreite < Wendekreisradius`): Mehrpunkt-Wende (5b)

### 5a. Einmaliges Wenden (breite Straße)

| Phase | Beschreibung |
|-------|-------------|
| **Rechts ranfahren** | Auto verschwenkt zum rechten Straßenrand (analog Parken, aber ohne zu stoppen). Geschwindigkeit wird reduziert auf ~5–10 km/h. |
| **Links ausschwenken** | Lenkrad voll links. Das Auto fährt einen großen Bogen von rechts nach links – der Vorderrad-Spur beschreibt eine Halbkurve über die gesamte Straßenbreite. Der Heckbereich folgt mit Slipwinkel (Bicycle-Model). |
| **Mittendurch** | Heading hat sich um ~180° geändert. Auto kreuzt die Mittellinie zwangsläufig – dies ist beim Wenden erlaubt und erwartet. |
| **Rechts ausrichten** | Lenkrad wird gerade gezogen bzw. leicht rechts nachgesteuert, damit das Auto auf der jetzt rechten Spur (ursprüngliche Gegenfahrbahn) parallel zum Straßenrand endet. |
| **Weiterfahren** | Blinker aus. Normalposition in neuer Fahrtrichtung. Gas geben. |

### 5b. Dreipunkt-Wende (schmale Straße)

Wenn die Straße zu schmal für einen einzigen Bogen ist, führt das Auto ein klassisches Dreipunkt-Wenden durch:

| Schritt | Gang | Lenkrad | Beschreibung |
|---------|------|---------|-------------|
| **1. Rechts ranfahren** | Vorwärts | leicht rechts | Langsam zum rechten Rand fahren und stoppen. Blinker links an. |
| **2. Links vorwärts** | Vorwärts | voll links | Langsam vorwärts, bis die Vorderräder fast am linken Straßenrand sind (Heading ~45–60° nach links). Stoppen. |
| **3. Rechts rückwärts** | Rückwärtsgang | voll rechts | Rückwärts bis das Auto am rechten Rand steht und Heading weiter gedreht ist (~120–150°). Stoppen. |
| **4. Links vorwärts** | Vorwärts | voll links | Vorwärtsgang, in die Gegenrichtung einfahren und parallel ausrichten. Blinker aus. |

Bei extrem schmalen Straßen (z.B. ~3 m) kann der **Vierpunkt-Wende** nötig sein: Schritte 2–3 werden einfach ein zweites Mal wiederholt – das Auto pendelt hin und her, bis es um 180° gedreht ist.

### Parameter

- `TURN_AROUND_MIN_WIDTH` – Minimale Straßenbreite für einmaliges Wenden (≈ Wendekreisradius)
- Geschwindigkeit während des Manövers: ~5–10 km/h (niedrig genug für engen Radius, hoch genug um nicht stehen zu bleiben)
- Blinker links während des gesamten Manövers aktiv
- LaneGuard wird während des Wendevorgangs temporär unterdrückt (Mittellinie-Überquerung ist beabsichtigt)
- Rückwärtsfahrt: Bicycle-Model mit negativem `speed` und invertierter Lenkwirkung

### TODO (nicht implementiert)

Dreipunkt-Wende erfordert folgende Erweiterungen:
- `speed` als signed Wert (negativ = rückwärts) – aktuell `max(0, ...)` in `car.py`
- Wende-Referenzlinie generieren: U-Kurve zurück die Straße runter (aktuelle Route baut nur vorwärts)
- Manöver-Flag für LaneGuard-Unterdrückung (Mittellinie-Überquerung ist beabsichtigt)

---

## 6. Ausweichen (Obstacle Avoidance)

**Auslöser:** Hindernis in der eigenen Spur erkannt (z.B. stehendes Fahrzeug, Baustelle, Gegenstand).

| Phase | Beschreibung |
|-------|-------------|
| **Hindernis erkennen** | Sensorik/API meldet Objekt in der Fahrspur innerhalb von `DETECTION_DISTANCE_M`. |
| **Versuch: Auf eigener Spur bleiben** | Auto versucht, das Hindernis so dicht wie möglich an der rechten Seite zu passieren, ohne die Mittellinie zu überqueren. Dazu blendet die Referenzlinie kurz nach rechts (zum Rand) und zurück. |
| **Falls nötig: Mittellinie überqueren** | Wenn das Hindernis zu weit in die eigene Spur ragt und ein Passieren auf der eigenen Seite unmöglich ist, schwenkt das Auto nach links über die Mittellinie – analog an einem langsamen Fahrzeug vorbeifahren. Blinker links an. |
| **Hindernis passiert** | Sobald das Hindernis hinter dem Auto ist, blendet die Referenzlinie zurück in die normale Spurposition. |
| **Blinker aus (falls aktiv)** | Wenn der Blinker wegen Mittellinie-Überquerung aktiv war, wird er automatisch ausgeschaltet, sobald das Auto wieder komplett auf der eigenen Spur ist. |

### Parameter

- `DETECTION_DISTANCE_M` – Erkennungsbereich für Hindernisse
- `MIN_CLEARANCE_M` – Minimaler Sicherheitsabstand zum Hindernis
- LaneGuard wird unterdrückt, solange das Ausweichmanöver aktiv ist
- Geschwindigkeit wird während des Manövers reduziert
