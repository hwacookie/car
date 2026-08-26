# Fahrmanöver – Spezifikation

Alle Manöver beschreiben, wie das Auto sich in der Simulation verhält.
Die Referenzlinie (BicycleNav) steuert Spurposition und Geschwindigkeit;
der Driver setzt Blinker und Intent (Gas / Bremse).

---

## 1. Parken (Pull-Over)

**Auslöser:** Rechtsblinker + Bremse (automatisch, wenn der Abstand zum Ziel die **geschwindigkeitsabhängige** komfortable Bremsstrecke erreicht — `v²/(2·A_PARK)` plus kleine Reaktionsreserve; bei 65 km/h ≈ 50 m, bei 20 km/h nur ~7 m. Kein fester Wert. Oder manuell per Tastatur/API).

| Phase | Beschreibung |
|-------|-------------|
| **Einleitung** | Blinker rechts an. Gas weg, Bremse an. Geschwindigkeit sinkt auf ~5–10 km/h. |
| **Verschwenken nach rechts** | Referenzlinie blendet von Normalposition zum rechten Straßenrand (`PARK_BLEND_START_M` → `PARK_BLEND_END_M`). Das Auto fährt eine sanfte Rechtskurve zum Rand. Der Zielabstand zum Rand ist das Ergebnis einer Suche: so nah wie möglich, ohne dass eine Fahrzeugecke während der Schrägfahrt über den Bordstein reicht. |
| **Parallel ausrichten** | Im letzten Abschnitt (`PARK_ALIGN_M`) ist die Referenzlinie konstant am Rand – das Auto fährt geradeaus parallel zum Straßenrand. Geregelt wird hier nicht mehr per Pure Pursuit (das kommt an der Linie *drehend* an), sondern mit einem Stanley-Gesetz: Lenkwinkel = Kurswinkelfehler + Querabstandsterm. Beide Terme gehen gemeinsam gegen null – genau „bündig am Rand und parallel dazu“. |
| **Anhalten** | Geschwindigkeit geht auf 0, dabei läuft die Verzögerung aus (progressive Bremsen) – kein Rucken beim Stillstand. Auto steht mit den Rädern parallel und möglichst nah am rechten Rand; bei einer Ziel-Flagge steht der vordere Stoßfänger an der Flagge. |
| **Blinker aus** | Sobald das Auto gestoppt ist (`speed < 0.1 m/s` und `dist_to_dest < 1.0 m`), wird der Blinker automatisch ausgeschaltet. |

Reproduzierbar headless: `.venv/bin/python scripts/sim_park.py [startpunkt] [--dest]`
logt den kompletten Anhaltevorgang (Phase, Verzögerung, Restweg, Querablage,
Kurswinkelfehler) und misst am Ende Bordsteinabstand und Stoßfängerposition.

### Parameter

- **Auslösedistanz** – geschwindigkeitsabhängig: `v²/(2·A_PARK)` + Reaktionsreserve (explizite Entscheidung: kein fester Wert)
- `A_PARK = 3.5 m/s²` – komfortables Bremsen beim Parken (~0,35 g), kein Volllastbremsen
- `PARK_CREEP_SPEED_M = 2.0 m/s` (7 km/h) – Creep-Geschwindigkeit in der Verschwenkzone (Band 5–10 km/h)
- `PARK_STOP_TAU = V_C / A_PARK` (≈ 0,57 s) – Zeitkonstante des Ausrollens.
  Im Schlussabschnitt ist die Zielgeschwindigkeit **proportional zur
  Restdistanz** (`v = d / τ`), die Verzögerung `a = v / τ` läuft also mit
  der Geschwindigkeit aus und ist am Stillstand null. τ ist so gewählt,
  dass die Verzögerung zu Beginn des Ausrollens genau `A_PARK` beträgt.
  Unter `PARK_ROLL_END_M_S = 0.3 m/s` übernimmt eine kleine konstante
  Verzögerung (`PARK_ROLL_END_A = 0.6 m/s²`), sonst kröche das Auto den
  Exponentialschwanz noch sekundenlang aus.
- **Zonengeometrie – abgeleitet, nicht frei gewählt** (die früher hier
  genannten 40 m / 12 m stammen aus einer Zeit vor dem Brems-&-Park-Plan;
  bei 2 m/s Creep wären 40 m *20 Sekunden Schrittgeschwindigkeit*):
  - `PARK_ALIGN_M = 4.0` – gerades Stück am Rand zum Ausrichten. Kürzer
    ging nicht: der Regler schleppt der Driftlinie ~0,35 m hinterher, ein
    Auto das beim Halten noch verschwenkt, steht schräg (gemessen 5–12°).
  - `PARK_BLEND_END_M = max(PARK_ALIGN_M, V_C·τ)` – Verschwenken
    abgeschlossen, konstanter Offset bis zum Stillstand.
  - `PARK_BLEND_START_M = V_C·PARK_SWERVE_S + PARK_BLEND_END_M` (≈ 12 m)
    – mit `PARK_SWERVE_S = 4.0 s`, also 8 m Driftweg. Kürzer erzwingt die
    Eckenprüfung einen Parkplatz weiter draußen (bei 6 m Drift 0,94 m vom
    Bordstein statt 0,54 m), länger bringt nichts mehr.
- **Alle Distanzen zählen ab dem HALTEPUNKT**, nicht ab dem Linienende:
  am Flaggenziel steht der vordere Stoßfänger an der Flagge (Referenzpunkt
  = Hinterachse, also `FRONT_OVERHANG_M` davor), an einer Sackgasse eine
  ganze Fahrzeuglänge vor dem Asphaltende.
- `PARK_TRACK_LOOKAHEAD_M` – Kurzer Lookahead für enges Tracking der Referenzlinie
- `PARK_ALIGN_GAIN = 1.0` – Querabstands-Verstärkung des Stanley-Reglers im Schlussabschnitt
- `PARK_ALIGN_LATERAL_M` – Laterale Toleranz für „auf der Linie“

### Variante: Sackgassenende / Route-Ende

Gleiches Manöver wie Parken, ohne Ziel-Flagge: Die Route endet an einer echten
Sackgasse (Knoten mit Grad ≤ 1 – keine Straße führt weiter). Das Auto darf dort
**nicht** zur Mittellinie ausgleiten (altes Verhalten), sondern zieht an den
**rechten Straßenrand** – so weit rechts wie möglich, ohne die befestigte Fläche
zu verlassen – und hält.

- **Zielposition**: rechteste befahrbare Position = rechter Fahrbahnrand − halbe
  Fahrzeugbreite − `ROAD_EDGE_TOLERANCE_M` (0,5 m); alle vier Räder bleiben
  vollständig auf der befestigten Fläche. Begrenzt durch die echte
  Straßengeometrie (Shapely-Polygon), nie durch einen festen Anteil der
  Straßenbreite – ein schmaler Wirtschaftsweg und eine breite Hauptstraße enden
  jeweils an ihrem eigenen Rand.
- **Warum nicht Mittellinie?** An einer normalen Kreuzung wird der Offset zur
  Mittellinie geblendet, weil beide benachbarten Segmente den gemeinsamen
  Knotenpunkt teilen und die Übergabe keinen lateralen Sprung haben soll. Am
  Sackgassenende gibt es keine Übergabe an ein weiterführendes Segment – das
  Auto stoppt einfach. Die natürliche Position ist der rechte Bordstein.
- **Ausführung (physikalisch plausibel)**: sanftes, distanzbasiertes Ausweichen
  über die letzten ~20 m (gleiches Fenster wie der Mittellinien-Blend), kein
  lateraler Snap oder Teleport; die Lateralführung bleibt innerhalb dessen, was
  das Auto bei seiner aktuellen Geschwindigkeit leisten kann. Der Rand-Blend
  ersetzt den Mittellinien-Blend nur bei einer *echten* Sackgasse; an jeder
  Kreuzung mit weiterführendem Segment (ausgeführter Bogen, Slide-past oder
  geradewürdige Fortsetzung) bleibt der Mittellinien-Blend unverändert.
- **Bremsen** bis zum Stillstand unverändert (physikbasierte Bremsstrecke +
  5 m Sicherheitsreserve); nur das laterale Ziel ändert sich von Mittellinie
  auf rechten Rand.
- **Optional: Wenden am Ende.** Soll die Route wiederholt werden, greift nach dem
  Anhalten am Rand die Sackgassen-Wende (180°-Heading-Flip, kleiner fester Nudge
  zurück auf die Fahrbahn, Blinker aus) – der Nudge behält die erreichte
  Randseite bei.

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

Gilt für den **BICYCLE-Modus** (AI-Fahrt): das Auto weicht selbst aus. Im
**FREE-Modus** ist das Ausweichen Sache des Spielers; das Auto hält sich dort
an Stop-on-Contact (`docs/OBSTACLES.md`).

Hindernisse werden über die Palette neben der Minimap oder per REST API auf
die Straße gesetzt und lassen sich als Layout speichern/laden —
`docs/OBSTACLES.md`.

### Zwei Hindernisklassen

- **Statisch** (dieses Kapitel): z.B. parkende Autos. Stehen still, Position
  bekannt → reine Geometrie: Lücken, Korridor, Bremsstrecke.
- **Beweglich** (später, noch nicht spezifiziert): andere Fahrzeuge,
  Fußgänger, Kinder. Sie können sich vor Erreichen des Kollisionspunkts
  entfernen und sind oft erst in großer Entfernung sichtbar → erfordert die
  **Vorhersage** möglicher Kollisionen (Zukunftsbelegung statt fester Boxen).
  Die Erkennungs-/Entscheidungslogik wird so gehalten, dass statische und
  bewegliche Hindernisse später dieselbe Schnittstelle speisen: „Belegung
  entlang der Route“.

### Erkennung (statische Hindernisse)

Ein Hindernis ist **relevant**, wenn sein Footprint den Korridor um die
Referenzlinie schneidet (Korridorbreite = Fahrzeugbreite + Reserve) und es in
Fahrtrichtung vor dem Auto liegt. Spätestens relevant ist es, wenn der Abstand
die geschwindigkeitsabhängige komfortable Bremsstrecke bis zum Halten hinter
dem Hindernis erreicht hat — `v²/(2·A_AVOID)` plus Reaktionsreserve, wie beim
Parken (explizite Entscheidung: kein fester Wert).

### Optionen (Priorität: rechts → links → halten)

| Option | Bedingung | Verhalten |
|--------|-----------|-----------|
| **1. Rechts vorbei** (eigene Spur) | Lücke zwischen Hindernis und rechtem Fahrbahnrand ≥ Fahrzeugbreite + `AVOID_CLEARANCE_M` (Hindernisseite) + `ROAD_EDGE_TOLERANCE_M` (Randseite) | Referenzlinie läuft durch die rechte Lücke, so nah am Rand wie erlaubt; **kein Blinker** |
| **2. Links vorbei** (Mittellinie überqueren) | Option 1 nicht möglich, **und** die Gegenfahrbahn ist innerhalb von `AVOID_ONCOMING_AHEAD_M` (vorne) bzw. `AVOID_ONCOMING_BEHIND_M` (hinten) frei | Referenzlinie überquert die Mittellinie; LaneGuard wird im Ausweichbereich unterdrückt; **Linker Blinker** an |
| **3. Halten** | Keine Passage-Option ist an der aktuellen Position noch sicher ausführbar (beide Seiten blockiert, zu nah, zu schnell) | Komfortables Bremsen mit `A_AVOID` zum Stillstand hinter dem Hindernis, Standabstand ≥ `STOP_GAP_M` |

**Erreichbarkeit wie beim Abbiegen (§4):** Ist die gewählte Option nicht mehr
physikalisch ausführbar, wird **nicht** unsicher ausgewichen — das Auto bremst
(Option 3). Ein Ausweichmanöver ist ein Manöver wie jedes andere: nichts
Unmögliches.

Mehrere Hindernisse (z.B. Slalom): die Entscheidung gilt über den gesamten
relevanten Abschnitt — maßgebend ist die **engste** freie Breite.

### Ausführung

- Das Hindernis wird als **zusätzliche harte Grenze** in die Raceline-
  Optimierung eingegeben: der Korridor wird um den dilatierten Footprint des
  Hindernisses (+ `AVOID_CLEARANCE_M`) verkleinert. Die Ausweichlinie ist dann
einfach die krümmungsärmste Linie durch die Lücke — dieselbe Maschine wie
Spurhalten und Abbiegen, keine Sonderlogik.
- Für Option 2 wird die Korridor-Untergrenze (Mittellinie) im Ausweichbereich
  aufgehoben (analog zur Einbahnregelung in §4).
- Die Geschwindigkeit ergibt sich aus dem bestehenden Speed-Profile aus der
  Krümmung der Ausweichlinie — enge Lücke bedeutet automatisch langsamer.
- **Blinker:** Der für Option 2 gesetzte Linker Blinker wird für die Dauer des
  Manövers gehalten (wie beim Wenden) und erst ausgeschaltet, wenn das Auto
  wieder vollständig auf der eigenen Spur ist — die mechanische Cam-Logik ist
  während des Ausweichens unterdrückt.
- **Rückkehr in die Normalposition:** Sobald das Heck des Autos die Vorderkante
des Hindernisses um `RECOVERY_MARGIN_M` passiert hat, greift die
  Hindernis-Grenze nicht mehr; die krümmungsärmste Linie ist wieder die
  Spurmitte — die Rückblendung ergibt sich von selbst (wie hinter einer
  Kreuzung).

### Parameter (Vorschläge — noch nicht implementiert)

- `A_AVOID = 3.5 m/s²` – komfortable Verzögerung für Erkennung und Halten
  (wie `A_PARK`, kein Grund zum Notbremsen)
- `AVOID_CLEARANCE_M = 0.5` – lateraler Mindestabstand zum Hindernis beim Passieren
- `AVOID_ONCOMING_AHEAD_M = 60` / `AVOID_ONCOMING_BEHIND_M = 30` – die
  Gegenfahrbahn muss in diesem Bereich frei sein (in Teil 1: frei von
  statischen Hindernissen; später auch von vorhergesagtem Verkehr)
- `STOP_GAP_M = 5.0` – Standabstand hinter dem Hindernis
- `RECOVERY_MARGIN_M` – Heck-Vorsprung, ab dem das Hindernis „passiert“ gilt

### Interaktion mit anderen Manövern

- **Aktives Abbiegen** (Blinker + Turn-Blend-Zone): keine Mittellinie-
  Überquerung zum Ausweichen — nur Option 1 oder 3.
- **Parken / Wenden am Ziel:** explizite Kommandos haben Vorrang; liegt das
  Ziel hinter einem Hindernis, gilt Option 3 (halten) statt drüberzufahren.
