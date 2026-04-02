# Standards and Annotations for Chamfer/Fillet in Engineering Drawings

## Overview

This research covers the engineering drawing standards, best practices, and annotation conventions for chamfers and fillets. Understanding these standards is important for SolveSpace's chamfer/fillet implementation because:
1. SolveSpace exports to DXF/SVG (2D drawings) — these drawings need to express chamfer/fillet dimensions correctly
2. SolveSpace exports to STEP (3D) — the geometry must represent chamfer/fillet surfaces correctly
3. Users will need to dimension/annotate chamfer/fillet features in SolveSpace drawings

## Key Sources
- Wikipedia: Chamfer (https://en.wikipedia.org/wiki/Chamfer)
- Wikipedia: Fillet (mechanics) (https://en.wikipedia.org/wiki/Fillet_(mechanics))
- Wikipedia: Engineering drawing (https://en.wikipedia.org/wiki/Engineering_drawing)
- ISO Standards catalog (accessed iso.org)
- Standard textbook knowledge (Madsen et al., "Engineering Drawing and Design" page 179)

---

## 1. ISO 13715 — Edge Condition Indication

### Standard Overview
**ISO 13715** is the international standard titled "Technical product documentation — Indication of edge condition". It is a key standard for communicating the condition of edges in engineering drawings.

#### Version History:
- ISO 13715:1994 — original (TC213/SC3)
- ISO 13715:2017 — current revision

#### Purpose:
ISO 13715 defines symbols and indications to specify:
1. **Undetermined edges** — where edge condition is not specified (manufacturing discretion)
2. **Removed material** — chamfer or rounding/fillet
3. **Unbroken edges** — sharp edges (no chamfer or fillet required)
4. **Required edge condition** — both specified and unspecified

### ISO 13715 Key Concepts

#### Edge Condition Symbol
ISO 13715 uses a modified version of the surface texture symbol to indicate edge condition. The symbol appears at the junction line (edge) in the drawing view.

#### Indication Values
The standard specifies:
- `t` — tolerance of the edge condition
- `R` — radius value for rounded edges (fillets)
- `C` or chamfer notation — size of chamfer

For a **chamfer**: notation uses the format `C<value>` or `<angle>×<distance>` (e.g., `C2` means 45°×2mm chamfer)

For a **fillet/round**: notation uses `R<value>` (e.g., `R3` means 3mm radius fillet)

### ISO 13715 vs. ASME

**ISO Region:**
- ISO 13715 uses specialized edge symbols attached to the drawing
- Placed on the edge line or via leader/extension line
- Format: edge symbol + value

**US/ASME Region:**
- ASME Y14.5-2018 handles chamfer as a general note or dimensional callout
- Chamfer: typically `<angle>×<distance>` notation (e.g., `45°×3`) or "3×45°" or "C3"
- Fillet: "R3" with a leader pointing to the feature

---

## 2. Drawing Callout Conventions for Chamfers

### Standard Chamfer Notation

In engineering drawing practice, chamfers are called out in several ways:

#### Method 1: Angle × Distance
```
45 × 3   (45 degree chamfer, 3mm leg)
```
This is the ISO preferred format (angle first, then distance).

#### Method 2: Distance × Angle
```
3 × 45°  (3mm leg, 45 degree)
```
ASME/US practice often puts dimension before angle.

#### Method 3: "C" Abbreviation
```
C3       (45° chamfer with 3mm leg)
C2.5     (45° chamfer with 2.5mm leg)
```
"C" notation assumes 45° angle; if not 45°, angle must be specified separately.

#### Method 4: General Note
On drawings with many chamfers of the same size:
```
BREAK ALL EDGES 0.5 × 45°
ALL UNMACHINED EDGES TO HAVE 2×45° CHAMFER
```

#### Method 5: Dual-Parameter Notation (non-45° chamfers)
```
1.5 × 30°
```
For non-standard angle chamfers.

### Chamfer Annotation in Technical Drawing

Per Madsen et al., "Engineering Drawing and Design" (Delmar 2004, ISBN 0-7668-1634-6):
- A chamfer is "a bevel on an edge"
- Standard angles: 30°, 45°, 60°
- 45° is most common (for "C" notation shorthand)
- The dimension value is the length of the leg (offset on each adjacent face)
- For unequal-leg chamfers: both distances specified separately

---

## 3. Drawing Callout Conventions for Fillets

### Standard Fillet Notation

#### 3D (Solid Model) Fillet:
```
R3       (3mm radius fillet)
R.5      (0.5mm radius)
```

#### 2D Sketch Fillet:
- Same `R<value>` notation
- If all interior corners have same radius: general note "ALL FILLETS R2"
- Individual specification: leader + "R2"

### Terminology Differences Between CAD Systems
Per Wikipedia (Fillet - mechanics article):

| Software | Rounded/convex edge | Angled cut |
|----------|--------------------|-----------| 
| Autodesk Inventor, AutoCAD, Rhino3D, CATIA, FreeCAD, SolidWorks | Fillet | Chamfer |
| CADKEY, Unigraphics | Blend | (N/A) |
| PTC Creo/Pro-Engineer | Round | Chamfer |

SolveSpace should follow the "Fillet" terminology (convex rounded = fillet, angled cut = chamfer) consistent with most CAD tools.

---

## 4. Fillet vs. Round vs. Chamfer Terminology

From Wikipedia's Chamfer article (citing Madsen et al. 2004, p. 179):
- **Chamfer** = flat transitional edge (often 45°) between two faces
- **Fillet** = rounding of an **interior** corner (concave, stress concentration reduction)
- **Round** or **radius** = rounding of an **exterior** corner (convex)

**Engineering note:** Some sources conflate fillet and round. SolidWorks/FreeCAD use "fillet" for both interior and exterior. The mechanical distinction:
- Interior corner → fillet (adds material in the corner)
- Exterior corner → round/radius (removes material from the corner)

For SolveSpace purposes:
- **CHAMFER** group = creates a flat cut at an edge (always removes material)
- **FILLET** group = creates a curved surface at an edge (may be interior or exterior edge)

---

## 5. Engineering Applications and Why Standards Matter

### Chamfer Applications (from Wikipedia)
1. **Assembly guidance** — leads bolts/pins into holes
2. **Edge protection** — prevents sharp edge injury
3. **Interference fit clearance** — clears internal radius from cutting tool
4. **Rust prevention** — eliminates exposed thin metal at corners
5. **Appearance** — decorative or aesthetic
6. **Manufacturing** — easier to verify than radius tolerances

### Fillet Applications (from Wikipedia/Fillet mechanics)
1. **Stress concentration reduction** — distributes load over broader area
2. **Aerodynamics** — reduces interference drag at junctions
3. **Manufacturing** — allows round-tip end mills to cut concave areas
4. **Safety** — eliminates sharp edges that cause injury

---

## 6. Drawing Annotation in SolveSpace Context

### SolveSpace's Current 2D Drawing Export

SolveSpace exports drawings (DXF/SVG) using its constraint-based dimensioning. In 2D sketch mode, it already handles dimension annotations.

For chamfer/fillet annotations in SolveSpace drawings:

#### 2D Sketch Chamfer Annotation
When a 2D chamfer exists in a sketch (LINE_SEGMENT replacing corner), the user can add:
- Two dimensions to the chamfer line (distance from each vertex)
- Or an angular constraint + linear dimension

Currently NO automatic chamfer annotation in SolveSpace.

#### 2D Sketch Fillet Annotation  
When a 2D fillet arc exists (ARC_OF_CIRCLE at a corner), the user can add:
- Radius dimension using `R` annotation (already supported by SolveSpace)
- `Sketch > Dimension > Radius` applies to arc entities

#### 3D Group Chamfer/Fillet Annotation
For 3D chamfer/fillet groups, annotation would typically appear in section views or orthographic views. SolveSpace currently does not have a mechanism to automatically annotate 3D features in exported 2D views.

---

## 7. ASME Y14.5 vs. ISO GPS Standards

### ASME Y14.5 (US Standard)
- ASME Y14.5-2018 (most recent)
- Used primarily in US, Canada
- Chamfer annotation: dimension + angle notation

### ISO GPS System (International Standard)
- ISO 8015 — GPS fundamentals
- ISO 128 — Technical drawing presentation
- ISO 13715 — Edge condition indication (specific to chamfer/fillet)
- ISO 129-1 — Dimensioning of linear features

**Key difference:** ISO 13715 provides a dedicated edge condition symbol system. ASME uses simpler text-based callouts.

---

## 8. ISO 13715 Symbol Details (Standard Content)

Based on published descriptions of ISO 13715:

### Symbol Structure
The edge condition symbol in ISO 13715 is derived from the surface roughness symbol (per ISO 1302). It has:
- A V-shape (like a check mark)
- Extended horizontal leg for written conditions
- Subscript area for edge condition value

### Condition Types (per ISO 13715:2017)
1. **Removed material** (e.g., chamfer or radius):
   - Indicated by a diagonal stroke through the symbol base
   - Value indicates max allowed removal: `R0.1...R0.5` = radius range
   - Or `C0.1...C0.5` = chamfer size range

2. **Unbroken edge** (sharp/unspecified):
   - No diagonal stroke
   - Value: `0` to indicate sharp edge required

3. **Unspecified**:
   - Symbol with question mark or "±" indicating tolerance without specifying method

### Practical Interpretation
In practice:
- Most manufacturers use informal notation: "C2", "R3", "2×45°"
- ISO 13715 symbol system is used in precise manufacturing documentation
- CAD software rarely generates ISO 13715 symbols automatically

---

## 9. Implications for SolveSpace Implementation

### Do We Need ISO 13715 Support?
For MVP: **No**. ISO 13715 is a drawing annotation standard for technical documentation. SolveSpace currently doesn't auto-generate ISO 13715 symbols for any features.

For future DXF export enhancement: **Possibly**. SolveSpace could export ISO 13715 edge symbols to DXF when chamfer/fillet groups are present.

### What SolveSpace Should Do (Practical)
1. **2D sketch fillet/chamfer**: Use existing constraint-based dimensioning. The user adds `Radius` dimension to arc (for fillet) or linear+angular dimensions to chamfer line.
   - No changes needed to drawing export for 2D sketch features.

2. **3D chamfer/fillet groups**: The Group::valA and Group::valB parameters contain the offset/radius. 
   - In the TextWindow (`ShowGroupInfo`), display the offset value clearly
   - In DXF/2D drawing export: no automatic annotation for 3D features (same as current extrude/revolve behavior — SolveSpace doesn't auto-dimension 3D features in 2D views)

3. **STEP export**: Chamfer/fillet surfaces should export as B-rep surfaces (PLANE for chamfer, CYLINDRICAL_SURFACE for fillet). This is sufficient; the downstream CAM/viewer can identify them.

### SolveSpace Group UI Display

In `ShowGroupInfo()` (textscreens.cpp:380), for CHAMFER group type, the display should show:
```
Chamfer: offset = 2.00 mm
```

For FILLET group type:
```
Fillet: radius = 3.00 mm
```

Using `SS.MmToString(g->valA)` to respect unit preferences.

---

## 10. Standard Chamfer/Fillet Parameters for User Expectation

### Common Chamfer Sizes (from industry practice)
- Fine: 0.3–0.5mm (edge break, corrosion prevention)
- Small: 1–2mm (part assembly)
- Medium: 2–5mm (functional clearance)
- Large: 5–10mm (cosmetic, heavy parts)
- Standard 45° is most common, but 30° and 60° also used

### Common Fillet Sizes (from industry practice)
- Tiny: 0.5–1mm (stress relief on small features)
- Small: 1–3mm (typical machined part)
- Medium: 3–10mm (structural application)
- Large: >10mm (heavy loading, fluid flow)

### Implication for SolveSpace UI
- Default value for chamfer: 1mm (reasonable starting point)
- Default value for fillet: 1mm
- The existing unit system (mm/inches) applies via `SS.MmToString` and `SS.StringToMm`

---

## 11. Drawing Standards Relevant to SolveSpace Export

### DXF Export (2D)
SolveSpace exports 2D drawings as DXF (AutoCAD format). Chamfer/fillet surfaces in 3D will appear as their 2D projections (lines/arcs) in DXF. No special annotation needed unless user explicitly adds dimensions.

### SVG Export (2D)  
Same as DXF — 2D projection only.

### STEP Export (3D)
Per ISO 10303 (STEP standard), B-rep surfaces are exported as:
- Chamfer surface: `PLANE` entity (bounded by edges)
- Fillet surface: `CYLINDRICAL_SURFACE` entity (bounded by edges)
Both are standard STEP entities fully supported by SolveSpace's existing STEP exporter (src/exportstep.cpp).

### PDF Export (2D)
SolveSpace exports PDF from 2D views. Chamfer/fillet projections appear as lines/arcs automatically from the geometry.

---

## 12. Summary of Standards Relevance to SolveSpace

| Standard | Relevance | Action Required |
|----------|-----------|-----------------|
| ISO 13715 | Edge condition indication | None for MVP; future enhancement for DXF annotation |
| ASME Y14.5 | GD&T for US drawings | No action; user adds dimensions manually |
| ISO 129-1 | Linear dimension notation | Handled by existing dimension tools |
| ISO 10303 (STEP) | 3D B-rep export | PLANE/CYLINDRICAL_SURFACE already supported |
| ISO 128 | Drawing presentation | No chamfer-specific requirements |
| DIN 406-12 | Tolerancing (German standard) | No action required |

### Key Conclusion
**ISO 13715 and ASME Y14.5 are annotation/documentation standards, NOT geometry standards.** They describe how to communicate chamfer/fillet requirements on drawings. SolveSpace's job is to:
1. Create the correct 3D geometry (which it will via `SSurface::FromPlane` and cylindrical NURBS)
2. Export that geometry correctly to STEP (via existing STEP exporter, no changes for flat/cylindrical surfaces)
3. Show the feature parameters in the TextWindow UI

The actual drawing annotation is the user's responsibility (add `R3` dimension, `C2` note, etc.) using SolveSpace's existing constraint/dimension tools.

---

## 13. Stress Concentration Factor Relevance (Engineering Context)

Fillets are commonly used to reduce stress concentration factors (Kt). This is a common engineering design requirement that motivates users to use fillets in SolveSpace designs. The formula for stress concentration at a fillet is:

```
Kt = 1 + (nominal stress × factor based on r/d ratio)
```

Where `r` is the fillet radius and `d` is the notch depth. Larger fillet radius → lower Kt → less fatigue failure risk.

**Implication for SolveSpace:** Users will want to change the fillet radius parametrically (Group::valA as a solver param) to optimize Kt. This is exactly what our parametric Group approach supports — change valA, re-generate, observe change in model.

---

## Conclusion

For the SolveSpace chamfer/fillet MVP:
1. **No new annotation features needed** — existing dimension/constraint tools cover user needs
2. **STEP export works as-is** — PLANE and CYLINDRICAL_SURFACE are standard STEP entities
3. **ISO 13715 is informative but not required** for MVP implementation
4. **TextWindow should show Group::valA** clearly formatted with units (using `SS.MmToString`)
5. **Default values**: chamfer = 1mm, fillet = 1mm
6. **Terminology follows majority convention**: CHAMFER = flat cut, FILLET = rounded blend
7. **ASME/ISO notation**: C-notation for 45° chamfer, R-notation for fillet — these can appear in group name/description in the TextWindow
