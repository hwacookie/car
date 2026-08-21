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
| **Einleitung** | Blinker in Abbiegerichtung an. Die Route wird neu gebaut (`_maybe_rebuild`), diesmal über den gewählten Ast der Kreuzung. |
| **Linienwahl** | Es gibt **keinen festen Abbiege-Offset mehr**. Die Fahrlinie ist die Lösung einer Optimierung (`src/raceline.py`): minimale Krümmung innerhalb eines Korridors, dessen Grenzen die beiden harten Regeln sind – nie von der Fahrbahn, nie auf die Gegenfahrbahn. Die klassische Linie „außen rein, Scheitel innen, außen raus“ ist **nirgends codiert**, sie fällt aus der Geometrie heraus. |
| **Kurve** | Das Speed-Profile leitet die Kurvengeschwindigkeit direkt aus der Krümmung der *optimierten* Linie ab: `v = sqrt(a_lat / kappa)`. Keine Sonderfälle, keine Deckelung. |
| **Zurück in Normalposition** | Ergibt sich von selbst – hinter der Kreuzung ist die krümmungsärmste Linie wieder die Spurmitte. |
| **Blinker aus** | `clear_blinker_if_turned()` prüft den Winkel des Kurvenausgangs und schaltet den Blinker ab. |

### Parameter

- `A_LAT_MAX = 4.5 m/s²` – Querbeschleunigungs-Limit (Untersteuern)
- `A_LAT_PLAN_FRACTION = 0.7` – das Profil plant nur mit 70 % davon. Plant
  man mit dem vollen Wert, ist die Gierrate im Scheitel bereits gesättigt
  und dem Regler bleibt **keine Reserve zum Nachkorrigieren**: gemessen lief
  Pure Pursuit dann 1 m innerhalb der eigenen Linie – genug, um mit der
  Flanke über die Mittellinie zu geraten.
- `LANE_CENTRE_MARGIN_M = 0.5` – Abstand der Korridor-Untergrenze zur
  Mittellinie. Enthält bewusst Reserve für den Schleppfehler des Reglers:
  der Korridor beschränkt die *Linie*, die harte Regel gilt aber dem *Auto*.
- `CURVATURE_WINDOW_M = 1.0` – feste physikalische Fensterbreite der
  Krümmungsmessung.

### Kreuzungsmitte (der weiße Punkt)

Geradeaus und beim **Rechtsabbiegen** muss der Knotenpunkt (der weiße Punkt,
den der Renderer an jedem Knoten mit Grad ≥ 3 zeichnet) **links** liegen –
das ist nur „rechts fahren“ an der Stelle, an der die Mittellinie aufhört zu
existieren.

Beim **Linksabbiegen** gilt das ausdrücklich **nicht**. StVO § 9 Abs. 4:
*„Linksabbieger müssen einander voreinander abbiegen, sofern nicht die
Verkehrslage oder die Gestaltung der Kreuzung ein Umeinanderfahren
erfordert.“* Der Regelfall ist **voreinander** – entgegenkommende
Linksabbieger fahren Fahrerseite an Fahrerseite aneinander vorbei, jeder
biegt vor der Mitte ab, und der Punkt liegt damit **rechts**.

Das ist keine Feinheit: zwingt man Linksabbieger, den Punkt links zu lassen,
ist die einzige verbleibende Linie enger als der Wendekreis des Autos
(gemessen 1,5 m gegen 3,46 m Minimum) – die Beschränkung war schlicht falsch
für dieses Manöver.

### Erreichbarkeit

Ein Blinker bedeutet „ich will an der nächsten Stelle abbiegen, an der es
*noch physikalisch geht*“ (siehe `TURN_REWORK_PLAN.md` § 2.5). Ist das Auto
bereits schneller, als das Profil an seiner Position erlaubt, hilft kein
Bremsen mehr – dann wird die Kreuzung **durchfahren** (`_rebuild_straight_past`)
und der Blinker bleibt an.

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
