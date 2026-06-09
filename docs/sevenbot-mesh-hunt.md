# 7Bot Mesh Hunt — Bevindingen

**Datum:** 2026-05-06
**Doel:** STL/CAD-meshes vinden voor de 7Bot desktop robotarm (PineconeAI Kickstarter 2015) om de URDF visueel hoogwaardig te maken.

---

## Samenvatting (TL;DR)

**Geen STL/STEP/CAD-bronnen gevonden** voor de 7Bot — niet op GitHub, Thingiverse, Printables, GrabCAD, Cults3D, of in officiële PineconeAI-repositories. Het bedrijf is na 2018 stilgevallen en heeft nooit CAD-bronbestanden vrijgegeven.

**Wel gevonden — twee waardevolle alternatieven:**
1. **Hoogwaardige technische lijntekening** met genummerde gewrichten (front + side view, vermoedelijk uit de officiële handleiding) — bruikbaar als visuele referentie en om af te meten.
2. **Gedimensioneerde reach-diagrammen** met de werkelijke armlengtes (434 mm totaal, deelradii 200 / 318.25 / 433.96 mm).
3. **Bestaande URDF (asukiaaa/ros_sevenbot)** — alleen primitieven (box/cylinder), géén meshes, maar de joint-axes en transforms zijn correct en kunnen als kruis-check dienen.

**Aanbeveling:** zie [Aanbeveling](#aanbeveling) onderaan — meten van de fysieke arm + tracen van blueprints in OpenSCAD is sneller en accurater dan blijven zoeken.

---

## Wat doorzocht is

### GitHub
| Query | Resultaat |
|---|---|
| `gh search repos "7bot"` | 30+ repos, vooral Discord/Twitch bots. Relevant: `7Bot/*` (officieel), `asukiaaa/ros_sevenbot`, `codyhex/sevenbot`, `dehavenm/7bot-Python`, `rmcolbert/7bot-Archive`, `TuxLeon/RPi_Console`, `woshialex/py7bot`, `Lumnca/7Bot_yolov5`, `robdobsn/RobotProgramSCAMP`, `OnionIoT/obot` |
| `gh search repos "sevenbot"` | Alleen `asukiaaa/ros_sevenbot` (zie boven) en `codyhex/sevenbot` (blijkt een wheeled robot te zijn — verkeerde naam) |
| `gh search code "7Bot URDF"` | Geen hits |
| `gh search code "7bot" --extension stl/step/iges/SLDPRT/f3d/obj/dae` | Geen relevante hits (alleen toevallige string-matches in ongerelateerde projecten) |

### Officiële PineconeAI/7Bot GitHub-org (`github.com/7Bot`)
- `7Bot-Arduino-lib` — Arduino bibliotheek (geen CAD)
- `7Bot-Processing-Examples` — Processing voorbeelden (geen CAD)
- `7Bot-V2` — alleen Chinese PDF's en firmware `.bin` (al bevestigd)
- `AI` — Python tutorials + voorbeelden, géén CAD, **maar wel hoogwaardige joint-foto's en blueprints**

### 3D-modelrepositories
| Site | Resultaat |
|---|---|
| Thingiverse (via 3d-search CLI + web) | Niets — alleen ongerelateerde "bot" speelgoed |
| Printables (via 3d-search CLI) | Niets — alleen ongerelateerde modellen |
| GrabCAD | Niets — wel veel andere 6-DOF arms, geen 7Bot |
| Cults3D | "3.9k results for 7bot" maar allemaal ongerelateerd |
| MakerWorld / 3DWhere / CrealityCloud | Niets relevants |

### Wayback Machine
- `web.archive.org` blokkeert Claude's WebFetch (403). Niet kunnen verifiëren of pinecone-ai.cn ooit CAD aanbood. Aanbeveling voor handmatige check: `https://web.archive.org/web/2017*/pinecone-ai.cn` en `http://www.7bot.cc`.

### Kickstarter-pagina
- KS blokkeert WebFetch (403). Op basis van campagne-tekst en news-coverage uit 2015 (Atmel blog, Roboticgizmos, Kicktraq) is er **geen belofte** geweest om CAD vrij te geven — wel software (Arduino lib, Processing). Het frame is "aluminium tubes + machined parts" wat closed-source is gebleven.

---

## Wel binnengehaald — gebruik in `docs/photos/sevenbot-blueprints/`

| Bestand | Bron | Wat het is | Bruikbaarheid |
|---|---|---|---|
| `joints.jpg` | `github.com/7Bot/AI/tutorials/img/joints.jpg` | Engineering lijntekening (front + side view) met alle 6 joints genummerd 0–5 | **Goud waard** — direct te tracen in Inkscape/OpenSCAD voor accurate silhouettes |
| `joint0.jpg` … `joint6.jpg` | idem | Close-ups per joint met as-aanduidingen | Ideaal voor joint origin/axis verificatie |
| `tech-spec.jpg` | `github.com/7Bot/AI/docs/images/tech-spec.jpg` | Specsheet (Chinees) met gedimensioneerde werkruimte: armlengte 434 mm, gewicht 1.2 kg, payload 0.2/0.3 kg, plus reach-diagrammen met radii 200 / 318.25 / 433.96 mm | Bevestigt link-lengtes en bereik |
| `hardware-peripherals.jpg` | idem | Foto van de complete kit met grippers en vacuum head | Visuele referentie voor end-effectors |
| `PineconeAI-logo.jpg` | idem | Logo (mocht je het ergens willen tonen) | Cosmetisch |
| `asukiaaa-sevenbot-reference.urdf` | `github.com/asukiaaa/ros_sevenbot/urdf/sevenbot.urdf` | Primitieve URDF (boxes/cylinders) met de juiste joint-axes en kinematische ketting | Cross-check tegen onze eigen URDF — link-lengtes en axes komen grotendeels overeen |

**Geen** STL/STEP/IGES/SLDPRT/DAE/OBJ-meshes gevonden om in `ros2_ws/src/sevenbot_description/meshes/` te plaatsen. De meshes-directory blijft leeg.

---

## Wat NIET vindbaar / paywalled / dead

- **PineconeAI website** (`pinecone-ai.cn`, `7bot.cc`) — bedrijf is defunct, sites offline. Wayback Machine ontoegankelijk via huidige tools.
- **Originele Kickstarter rewards** — bevatten waarschijnlijk geen CAD; campagne benadrukte aluminium-machined parts, geen open hardware-belofte.
- **CN forums (52pojie, makerlab, eepw)** — niet doorzocht omdat zoekmachines geen Chinese 7Bot-CAD-resultaten teruggeven.

---

## Aanbeveling

**Niet verder zoeken naar bestaande meshes.** De bron is dood, en wat er was is nooit gepubliceerd. Twee veel productievere routes:

### Route A — Meten + parametrische OpenSCAD (geschat 2–4 uur)
1. Erik meet de fysieke arm met een schuifmaat: link-lengtes, profielmaten, servo-housing dimensies, joint offsets.
2. Combineer met de bekende specs uit `tech-spec.jpg` (434 mm totaal, 200/318/434 reach).
3. Bouw per link een `.scad`-bestand met simpele extrusies (aluminium U-profielen + servo-blocks + scharnierkappen). Export → STL → in URDF.
4. Resultaat: parametrisch, makkelijk aan te passen, en accurater dan welke gevonden mesh ook zou zijn geweest.

### Route B — Trace de blueprint in Inkscape → OpenSCAD (1–2 uur per link)
1. Open `joints.jpg` in Inkscape, trace silhouet per link in twee orthogonale views.
2. Importeer SVG in OpenSCAD, `linear_extrude` met geschatte breedte (uit fysieke meting of pix-to-mm scaling op basis van bekende totaallengte 434 mm).
3. Snel maar minder netjes dan Route A — geen aparte features (gaten, kapjes, servo-mounts).

**Voorkeur:** Route A. De skill `openscad` is beschikbaar voor parametrische modellering en STL-export. Erik hoeft alleen ~10 maatvoeringen te leveren (bv. base 80×60×40 mm, link2-tube 120 mm, etc.) — dan kan ik de meshes binnen één sessie genereren.

### Route C (laatste redmiddel) — Photogrammetry
Als meten te omslachtig is: 30–50 foto's rondom de arm met smartphone → Meshroom of Polycam → ruwe mesh → handmatig opschonen. Veel werk, lagere fidelity dan Routes A/B.

---

## Bronnen geraadpleegd

- [7Bot Kickstarter campaign](https://www.kickstarter.com/projects/1128055363/7bot-a-powerful-desktop-robot-arm-for-future-inven)
- [7Bot officiële GitHub-org](https://github.com/7bot)
- [asukiaaa/ros_sevenbot](https://github.com/asukiaaa/ros_sevenbot) — primitive-only URDF
- [codyhex/sevenbot](https://github.com/codyhex/sevenbot) — niet de 7Bot maar een wheeled robot
- [rmcolbert/7bot-Archive](https://github.com/rmcolbert/7bot-Archive) — archief van docs, GUI binaries, geen CAD
- [robdobsn/RobotProgramSCAMP](https://github.com/robdobsn/RobotProgramSCAMP) — bevat alleen een `.dxf` voor een mounting-baseplate (niet de arm zelf)
- [Atmel blog 2015 announcement](https://atmelcorporation.wordpress.com/2015/09/15/7bot-is-a-desktop-robot-arm-that-can-see-think-and-learn/)
- [Robots and Physical Computing — 7bot tag](https://robotsandphysicalcomputing.blogspot.com/search/label/7bot)
