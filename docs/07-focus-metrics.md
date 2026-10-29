# Focus (sharpness) metrics

**Summary.** Autofocus and refocus rate each image with a scalar sharpness score computed by one of two ALOX sub-VIs.
`alox_vision_measure_focus_fft.vi` extracts an optional rectangular ROI, decimates it by nearest-neighbour resampling with a
scale factor, and returns the **sum of the magnitudes of the complete 2-D DFT, DC term included**.
`alox_vision_measure_focus_sobel.vi` runs the same front end, applies an NI non-linear edge filter (Differentiation, Gradient,
Prewitt, Roberts, Sigma or Sobel), and returns the **mean grey level of the edge image**. In `FULL` mode it returns the
**product of the six means**. The refocus adds two compositions: "Sigma" runs a 20×20 conditional low-pass (`IMAQ LowPass`)
in place on the camera image first, and "FULL" multiplies the six-filter edge product (ROI) by the FFT score (whole image).
The ALOX wiring is fully visible in the printouts. The internals of the NI library functions are not: resampling phase, border
handling, FFT scaling, and the exact Sigma and LowPass formulas. This chapter gives the exact ALOX logic and the best-known NI
definitions, and it marks every NI-internal point. The Python reference turns each unknown into an explicit parameter, and a
numerical validation plan settles them against NI Vision 2018.

**Sources:** `alox_vision_measure_focus_fft.vi` [Doc6 p.946–950]; `alox_vision_measure_focus_sobel.vi` [Doc6 p.955–960];
NI Vision 2018 VIs called by them: IMAQ FFT [Doc6 p.938–941], IMAQ ComplexPlaneToArray [Doc6 p.942–945], IMAQ Extract 2
[Doc6 p.951–954], IMAQ EdgeDetection [Doc6 p.986–989], IMAQ Histogram [Doc6 p.990–993], IMAQ Resample [Doc6 p.994–997],
Image Type [Doc6 p.998–1001], IMAQ Create [Doc6 p.1002–1005], IMAQ GetImageSize [Doc6 p.1006–1009], IMAQ Dispose
[Doc6 p.1014–1017], IMAQ Image.ctl [Doc6 p.1018–1021]; IMAQ LowPass [Doc6 p.961–964] (called by the refocus). Call sites:
`alox_camera_server_autofocus.vi`, state `Measure` [Doc6 p.605–606]; `alox_camera_server_refocus.vi`, state `Mesure Position`
[Doc6 p.1035–1036]. ROI Descriptor [Doc6 p.1010–1013] and IMAQ Point [Doc6 p.1022–1025] belong to the dead "Oxyd" branch of
the refocus and are not used by the metrics.

Related chapters: [01-architecture.md](01-architecture.md) (camera loop, threading),
[05-camera-acquisition.md](05-camera-acquisition.md) (image source, pixel type, flip, flat field),
[06-autofocus-refocus.md](06-autofocus-refocus.md) (when and with which ROI and scale the metrics are called, how the scores are
stored and returned), [09-configuration-logging-errors.md](09-configuration-logging-errors.md) (error clusters),
[10-migration-plan.md](10-migration-plan.md). The questionnaire items mentioned below are in [open-questions.md](open-questions.md).

**Evidence tags.** Untagged statements are read from the printouts. **(inferred)** marks a deduction. For NI internals it
also marks behaviour taken from NI's published documentation (NI Vision Concepts Manual, function help), which is not part of
the printouts. **(unknown)** marks a point the printouts cannot settle: an experiment (§10) or a questionnaire answer must.

---

## 1. Overview

### 1.1 Where the metrics are used

| Caller | `Autofocus Method` value | Computation | ROI | Scale factor |
|---|---|---|---|---|
| Autofocus [Doc6 p.605–606] | `FFT` | FFT metric | client `ROI` | client `Subsampling` |
| | `Prewitt` | edge metric, `Method = Prewitt`, `FULL` **not wired** (sub-VI default, presumably False **(inferred)**) | client `ROI` | client `Subsampling` |
| | `Sobel` and every other value (`"Sobel", Default` case) | edge metric, `Method = Sobel`, `FULL = False` | client `ROI` | client `Subsampling` |
| Refocus [Doc6 p.1035–1036] | `Sobel` / `Prewitt` / `Diff` / `Gradient` | edge metric, `Method` = Sobel / Prewitt / Differentiation / Gradient, `FULL = False` | computed "Fin" ROI | constant 0.5 |
| | `FFT` | FFT metric | Fin ROI | 0.5 |
| | `Sigma` | `IMAQ LowPass {20, 20, 40 %}` in place on the whole camera image, then edge metric `Method = Sigma`, `FULL = False` | Fin ROI | 0.5 |
| | `FULL` | edge metric `FULL = True` (`Method` not wired) **×** FFT metric with `Optional Rectangle` not wired (whole image); both VIs run in parallel | Fin ROI for the edge part only | 0.5 |

- The `Timer (ms)` outputs of both VIs are not wired by either caller [Doc6 p.605–606, p.1035–1036].
- The input image is the session's source image after flip and optional flat-field correction (see
  [05-camera-acquisition.md](05-camera-acquisition.md)). This buffer is created as `Grayscale (U8)` [Doc6 p.1003], but
  `IMAQdx Get Image2` "changes the image type to a suitable format" when it does not match the camera's video format
  [Doc6 p.895]. A Mono8 camera therefore gives U8 input (the expected case **(inferred)**) and a Mono16 camera would give U16.
  The actual camera pixel format is **(unknown)** (questionnaire Q7).
- The ROI computation (Fin ROI), the argmax, and the storage of `[Z, score]` rows are specified in
  [06-autofocus-refocus.md](06-autofocus-refocus.md).
- The dead refocus "Oxyd" branch calls other metric sub-VIs (labelled "FFT OXYD" and "SOBEL OXYD") with a circular ROI
  [Doc6 p.1036]. They are not in the printouts, so this chapter cannot specify them. They matter only if questionnaire Q19
  revives the Oxyd method.

### 1.2 Processing chain

```
Image In (U8) ──► [ROI: IMAQ Extract 2, only if Optional Rectangle is non-empty]
              ──► [IMAQ Resample, Zero Order, to rint(W·s) × rint(H·s)]
              ├─► FFT metric : IMAQ FFT (complex SGL) ─► |F| (SGL array) ─► Σ (SGL) ─► Focus Score (DBL)
              └─► Edge metric: IMAQ EdgeDetection(Method, Threshold 0) (U8, saturated) ─► IMAQ Histogram ─► Mean Value (SGL)
                               FULL: the six methods, product of the six means (SGL) ─► Focus Score (DBL)
Image Out = Image In (same reference, untouched)
```

Neither metric VI applies a low-pass filter, a window, a frequency mask or a mean subtraction [Doc6 p.948, p.957].
`IMAQ LowPass` appears only in the refocus "Sigma" branch.

### 1.3 What is certain and what is not

| Element | Status | Settled by |
|---|---|---|
| ROI rule, size rounding, call order, parameters, aggregation (sum, mean, product), FULL wiring, error paths | observed | — |
| `[L, T, R, B]` rectangle with exclusive right/bottom edge | **(inferred)** | A2 |
| Behaviour for out-of-image, `[0,0,0,0]` or non-4-element rectangles | **(unknown)** | A2 |
| Zero-order source-pixel mapping (phase) | **(unknown)**, 3 candidates | A1 |
| FFT scaling, padding, accepted input types | **(unknown)**, no scaling and no padding expected | A7 |
| Formulas of the six edge filters | **(inferred)** from NI documentation; Sigma is the least certain | A3–A5 |
| Border fill used by the edge filters | **(unknown)**, 3 candidates | A4 |
| `IMAQ LowPass` formula, even kernel size | **(unknown)** | A9 |
| Execution order of the six parallel FULL branches | **(unknown)** (race) | B2 |
| Default of the `FULL` input | False **(inferred)** from the front-panel rendering | Q30 |

---

## 2. LabVIEW and NI Vision conventions used in this chapter

| Convention | What it means | Python consequence |
|---|---|---|
| Error cluster `{status, code, source}` | NI VIs and both metric VIs skip their work when `error in.status` is TRUE. The metric VIs then return `Focus Score = 0` [Doc6 p.949, p.959]. A failure inside the chain also ends with score 0, because the NI nodes downstream do not execute and return defaults (an empty array sums to 0, an empty histogram report has mean 0) **(inferred)**. | Raise an exception internally and convert it to `(score 0.0, error)` at the caller boundary (§9). |
| Image wire = reference | An NI image is a buffer passed by reference. `Image Out` is the same buffer as `Image In`. An in-place operation (`IMAQ LowPass` with `Image Dst` unwired) changes the caller's image. | NumPy arrays; never modify the input. Return a new array when the caller must see a filtered image. |
| `Image Dst` unwired | The result is written into `Image Src` (in place) **(inferred, NI convention)**. A wired `Image Dst` is resized to the result size automatically **(inferred)**. | — |
| Named images | `IMAQ Create` with a name that already exists returns that existing image **(inferred, NI behaviour)**. Both metric VIs use fixed names (`AF_FFT_TMP`, `AF_FFT_CPLX`, `AF_FFT_DST`), so the buffers are process-wide globals. Both VIs dispose of them at the end of every call. | Local arrays only. See defect F3. |
| `IMAQ Create` defaults | `Image Type` unwired = `Grayscale (U8)`, `Border Size` = 3 [Doc6 p.1003]. | Work in the input dtype. |
| Rectangle `Optional Rectangle` | 1-D I32 array, default empty [Doc6 p.943, p.952]. Element order `[Left, Top, Right, Bottom]` **(inferred, NI convention)**; both callers also turn the same array into an overlay rectangle `{Left, Top, Right, Bottom}` with `Array To Cluster` [Doc6 p.606, p.1035]. Empty or unwired = whole image **(inferred, NI convention)**. The ALOX VIs do not rely on it for `Extract`: they skip the call when the array is empty. | `img[top:bottom, left:right]` (§4.1). |
| Coordinates | `X Resolution` = width (columns), `Y Resolution` = height (rows) [Doc6 p.1006]. Origin top-left. | `h, w = img.shape`; index `img[y, x]`. |
| `To Long Integer` (DBL → I32) | Rounds to nearest, **ties to even**, and saturates at the I32 limits. | `np.rint` or Python `round()`, **not** `int()`. |
| Coercion dots | SGL → DBL is exact. The SGL results therefore travel as DBL values with float32 granularity. | `float(np.float32(x))`. |
| Compare Aggregates `=` with an empty array | TRUE only for an empty array. `[0,0,0,0]` is not empty. | `len(rect) == 0`. |
| Unwritten indicator | A sub-VI output that is not written during a call returns its previous value **(inferred, LabVIEW behaviour)**. This applies to `Timer (ms)` in the Error case. | Not reproduced (the output is unused). |
| Dataflow parallelism | Nodes without data dependencies may run in any order or concurrently. NI library nodes are thread-safe calls ("run in any thread") **(inferred)**. Whether the two ALOX VIs are reentrant is **(unknown)** (Q30); the races of §6.7 and §7.4 do not depend on it, because they involve NI nodes or two different VIs. | Sequential code (defects F3, F4). |
| `Tick Count (ms)` | U32 millisecond counter, wraps after about 49.7 days. | `time.perf_counter()` if timing is needed. |
| Persistent state | The metric VIs have no shift registers and no persistent data. The only shared state is the named NI images. | Pure functions. |

NI enum values matter only for a LabVIEW validation harness. EdgeDetection `Method`: the printouts never list the enum items;
the FULL frame places the six constants in the order Differentiation, Gradient, Prewitt, Roberts, Sigma, Sobel [Doc6 p.957],
which matches NI's documented enum Differentiation = 0 … Sobel = 5 **(inferred)**. The NI VI's own default is
`Differentiation` [Doc6 p.987]; the ALOX VI's default is `Sobel` [Doc6 p.956]. `Interpolation Type` default
`Zero Order` [Doc6 p.995] (= 0 **(inferred)**). `Plane` default `Real` [Doc6 p.943] (Real, Imaginary, Magnitude, Phase =
0…3 **(inferred)**).

---

## 3. Interfaces and common structure

### 3.1 `alox_vision_measure_focus_fft.vi` [Doc6 p.946–947]

`D:\ALOX\Vision Core\sources\vision\alox_vision_measure_focus_fft.vi`, modified 22/09/2022.

| Terminal | Dir. | Type | Default | Meaning |
|---|---|---|---|---|
| `Optional Rectangle` | in | I32[] | empty | ROI `[L, T, R, B]`; empty = whole image |
| `Image In` | in | IMAQ image ref | — | image to rate (not modified) |
| `Scale Factor` | in | DBL | **0.25** | subsampling factor |
| `error in (no error)` | in | error cluster | no error | |
| `Image Out` | out | IMAQ image ref | | = `Image In` (wire bypasses all processing) |
| `Focus Score` | out | DBL | 0 | score |
| `Timer (ms)` | out | I32 | 0 | elapsed time of frames 0–2 (unused by callers) |
| `error out` | out | error cluster | | |

### 3.2 `alox_vision_measure_focus_sobel.vi` [Doc6 p.955–956]

`D:\ALOX\Vision Core\sources\vision\alox_vision_measure_focus_sobel.vi`, modified 05/07/2023. The icon reads "SOBEL" with a
red "PREWITT" caption above it (cosmetic).

| Terminal | Dir. | Type | Default | Meaning |
|---|---|---|---|---|
| `Optional Rectangle` | in | I32[] | empty | as above |
| `Image In` | in | IMAQ image ref | — | |
| `Scale Factor` | in | DBL | **0.25** | |
| `Method` | in | EdgeDetection `Method` enum | **Sobel** | Differentiation, Gradient, Prewitt, Roberts, Sigma, Sobel |
| `error in (no error)` | in | error cluster | | |
| `FULL` | in | Boolean | False **(inferred)** | product of all six methods; `Method` is then ignored |
| `Image Out` | out | IMAQ image ref | | = `Image In` |
| `Focus Score` | out | DBL | 0 | |
| `Timer (ms)` | out | I32 | 0 | unused by callers |
| `error out` | out | error cluster | | |

The `FULL` default is read from the front-panel rendering: a silver switch with a dark-green arrow, the same bitmap as other
Boolean controls whose default is False [Doc6 p.956]. Open the VI to confirm (questionnaire Q30).

### 3.3 Common skeleton [Doc6 p.948–949, p.957–959]

```
case error in.status:
  TRUE  ("Error" frame):  Focus Score := 0 ; Image Out := Image In ; error out := error in   # Timer not written
  FALSE ("No Error" frame): stacked sequence
     frame 0: t0 := Tick Count (ms)
     frame 1: computation (§5.1 or §6.1); work images passed to frame 2 through sequence locals
     frame 2: Timer (ms) := I32(Tick Count − t0) ;
              IMAQ Dispose(work image 1) ; IMAQ Dispose(work image 2)       # error terminals not wired
Image In → Image Out: a direct wire outside the sequence
```

In frame 2 the Tick Count and the two Dispose calls are not ordered by any wire, so the Timer may or may not include the
dispose time. The value is unused, and the Python port omits it.

---

## 4. Front end: ROI extraction and subsampling

Both VIs run exactly the same front end [Doc6 p.948, p.957]:

```
(tmp, e1) := IMAQ Create("AF_FFT_TMP")                    # U8, border 3
(second work image, e2) := IMAQ Create(...)               # FFT: "AF_FFT_CPLX", Complex (CSG); edge: "AF_FFT_DST", U8
err := Merge Errors(e1, e2)
if Optional Rectangle == []:                              # Compare Aggregates with an empty I32 array constant
    src := Image In ; dst := tmp
else:
    tmp, err := IMAQ Extract 2(Image Src = Image In, Image Dst = tmp, Optional Rectangle, X Step = 1, Y Step = 1)   # steps unwired = 1
    src := tmp ; dst := tmp                               # resample in place
(W, H), err := IMAQ GetImageSize(src)
X := To I32(DBL(W) × Scale Factor) ; Y := To I32(DBL(H) × Scale Factor)
tmp, err := IMAQ Resample(Image Src = src, Image Dst = dst, X Resolution = X, Y Resolution = Y,
                          Interpolation Type = Zero Order, Optional Rectangle unwired)
```

The rendering confirms the wiring: the upper multiply (width) feeds `X Resolution`, the lower one (height) feeds
`Y Resolution` [Doc6 p.948, p.994].

### 4.1 ROI rule

| Input | Effect |
|---|---|
| empty array | whole image, no `Extract` call |
| 4 elements `[L, T, R, B]` inside the image | sub-image of width `R − L` and height `B − T` **(inferred: right and bottom edges excluded)** |
| `[0,0,0,0]`, `R ≤ L`, `B ≤ T`, partly or fully outside the image, or not 4 elements | still passed to `IMAQ Extract 2`. NI may return an error (score 0) or clip **(unknown)** |

Neither caller clips the rectangle. A refocus "Fin" ROI wider than the image gives a negative `Left` (see
[06-autofocus-refocus.md](06-autofocus-refocus.md) §5.6). An autofocus client can send any array.

### 4.2 Target size and rounding

`X = rint(W_R · s)`, `Y = rint(H_R · s)`, where `rint` rounds to nearest with ties to even. `W_R × H_R` is the ROI size (or
the image size). The multiplication is in DBL.

| `W_R` | `s` | `W_R · s` | `X` |
|---|---|---|---|
| 2448 | 0.25 | 612.0 | 612 |
| 1002 | 0.25 | 250.5 | **250** (tie to even) |
| 1006 | 0.25 | 251.5 | **252** (tie to even) |
| 201 | 0.5 | 100.5 | 100 |
| 203 | 0.5 | 101.5 | 102 |
| 2 | 0.25 | 0.5 | 0, which gives a resample error **(inferred)** and score 0 |
| any | 0 | 0 | 0, same as above. This is the autofocus typedef default `Subsampling = 0` (defect F13) |

With `s = 0.25` or `0.5` the product is exact in binary floating point, so ties really occur and must follow the even rule.
`s > 1` up-samples (pixel replication). No server code path does this, but nothing stops an autofocus client from sending it.

### 4.3 Nearest-neighbour mapping (Zero Order)

`S(y, x) = R(sy(y), sx(x))` for `0 ≤ x < X`, `0 ≤ y < Y`. There is no averaging and no anti-aliasing. NI does not document
which source pixel "zero order" picks **(unknown)**. Three candidates, with indices clamped to `W_R − 1`:

| Phase | `sx(x)` | `s = 0.25` (W/X = 4) | `s = 0.5` | `W = 1002 → X = 250`, x = 0, 1, 124, 125, 126, 249 |
|---|---|---|---|---|
| `corner` (provisional default) | `floor(x·W/X)` | 0, 4, 8, … | 0, 2, 4, … | 0, 4, 496, 501, 505, 997 |
| `corner_round` | `floor(x·W/X + ½)` | 0, 4, 8, … | 0, 2, 4, … | 0, 4, 497, 501, 505, 998 |
| `center` | `floor((x + ½)·W/X)` | 2, 6, 10, … | 1, 3, 5, … | 2, 6, 498, 503, 507, 999 |

Integer arithmetic (`x·W // X`) is the reference. Evaluating `x·(W/X)` in floating point can be off by one at isolated
positions when the ratio is not an integer. The phase changes the absolute scores; it should not change the shape of a focus
curve much **(inferred)**. Test A1 settles it.

### 4.4 Consequences of the front end

- Decimation without a low-pass filter aliases high frequencies. Both metrics should still decrease with defocus
  **(inferred)**; on a synthetic box-blur series the sketch of §9.2 gives monotonically decreasing FFT and Sobel scores. This
  is the behaviour to reproduce, not a bug to fix.
- Only one pixel in `1/s²` contributes. Measured on a fixed scene, the score therefore depends on the sub-pixel phase of the
  scene relative to the sampling grid, and small lateral shifts change it slightly **(inferred)**.

---

## 5. FFT metric

### 5.1 Exact computation [Doc6 p.948]

```
cplx, err := IMAQ FFT(Image Src = tmp (resampled), Image Dst = AF_FFT_CPLX)          # complex single precision (CSG)
First Quadrant? := constant False
rect := First Quadrant? ? [0, 0, To I32(Wc/2), To I32(Hc/2)] : []                    # True branch is dead code
M, err := IMAQ ComplexPlaneToArray(Image = cplx, Plane = Magnitude, Optional Rectangle = rect)   # 2-D SGL array [row][col]
Focus Score := DBL(Add Array Elements(M))                                            # SGL sum, coerced to DBL
error out := err
```

### 5.2 Definition

With `S` the `Y × X` resampled image:

```
F(u, v) = Σ_{y=0}^{Y−1} Σ_{x=0}^{X−1} S(y, x) · exp(−2πi (u·x/X + v·y/Y))       (unnormalised forward DFT, inferred)
Score   = Σ_{v=0}^{Y−1} Σ_{u=0}^{X−1} |F(u, v)|,   |F| = sqrt(Re² + Im²)
```

- NI's printed description says "The FFT creates a complex image in which high frequencies are grouped at the center, while
  low frequencies are located at the edges" [Doc6 p.938]. This is the standard unshifted layout (DC at index (0, 0)), the same
  as `numpy.fft.fft2` without `fftshift` **(inferred)**. The quadrant layout does not affect a sum over the whole plane anyway.
- **DC included**: `|F(0,0)| = Σ S(y,x)`, the sum of the intensities. For a constant image of value `c` the score is
  `c·X·Y` in exact arithmetic (a single-precision FFT adds rounding noise in the other coefficients). The score therefore
  grows with brightness and gain. In a Z scan at constant illumination the DC term is nearly constant, so it should hardly move
  the argmax, but it flattens the relative contrast of the curve **(inferred)**.
- Scaling: none expected **(inferred)**. A constant factor (1/N or 1/√N) would not change an argmax but would change the
  absolute values (test A7).
- Padding: NI may pad to a power of two **(unknown)**. That would change the score substantially. Test A7 compares the size
  of the complex output with the input.
- Input types other than U8 (U16, I16, SGL) **(unknown)** (A10).
- Equivalent fast form for real input: `Σ|F| = Σ_v Σ_u w(u)·|R(u, v)|` with `R = numpy.fft.rfft2(S)` (half spectrum along
  `x`, `u = 0 … X//2`), `w = 2` except `w = 1` for column 0 and, when `X` is even, column `X/2`. The result equals the
  full-FFT sum up to rounding (checked numerically for even and odd `X`).

### 5.3 Dead option `First Quadrant?`

When True, the sum is restricted to `u < rint(X/2)`, `v < rint(Y/2)` (DC plus positive frequencies, through an
`Optional Rectangle` on `ComplexPlaneToArray`). The selector is wired to a constant False [Doc6 p.948]. Do not implement it.

---

## 6. Edge metric

### 6.1 Exact computation [Doc6 p.957]

```
# after the common front end (§4); dst = "AF_FFT_DST" (U8)
if not FULL:                                             # visible frame
    dst, err := IMAQ EdgeDetection(Image Src = tmp, Image Dst = dst, Image Mask unwired,
                                   Method = Method, Threshold Value = 0 (SGL constant))
    report, err := IMAQ Histogram(Image = dst, Number of Classes = 256 (default), Interval Range = (0, 0) (default),
                                  Image Mask unwired)
    score := report.Mean Value                           # 6th element: histogram, Minimal, Maximal, Starting, Interval Width, MEAN, ...
else:                                                    # hidden frame "True"
    for m in [Differentiation, Gradient, Prewitt, Roberts, Sigma, Sobel]:      # six PARALLEL branches, no ordering
        dst_m := IMAQ EdgeDetection(Src = tmp, Dst = dst (the SAME image for all six), Method = m, Threshold = 0)
        mean_m := IMAQ Histogram(dst_m).Mean Value
    score := mean_Diff × mean_Grad × mean_Prewitt × mean_Roberts × mean_Sigma × mean_Sobel   # Compound Arithmetic, SGL
    # error chain: only the Differentiation branch (EdgeDetection + Histogram) is wired; the other five errors are lost
    # and those five nodes have no error in, so they run even after an upstream failure (Extract, Resample...)
Focus Score := DBL(score)
```

After an upstream failure the Differentiation histogram does not run, its `Mean Value` is 0, and the FULL product is 0
**(inferred)**, so the "score 0 on error" rule of §2 still holds.

The `Histogram Report` field order is printed on [Doc6 p.991]: `histogram`, `Minimal Value`, `Maximal Value`,
`Starting Value`, `Interval Width`, `Mean Value`, `Standard Variation`, `Area (pixels)`. All seven Unbundle nodes take the
6th element (`Mean Value`). The rendering of the FULL frame confirms the shared source and destination wires and the single
error chain [Doc6 p.957].

### 6.2 Score

With `E_m` the edge image of method `m` (same size and type as `S`, saturated):

```
FULL = False:  Score = float32( (1/(X·Y)) · Σ_{y,x} E_Method(y, x) )
FULL = True :  Score = Π_{m in Diff, Grad, Prewitt, Roberts, Sigma, Sobel} float32(mean(E_m))     (each partial product rounded to float32, inferred)
```

The threshold of 0 has no effect, because edge values are never negative **(inferred)**. The intended FULL result is the
product above. What LabVIEW actually computes depends on the race (§6.7).

### 6.3 Edge filter definitions **(inferred, NI documentation)**

Notation: `p(x, y)` is the input pixel, `x` the column, `y` the row, and `y − 1` is the row above. Prewitt and Sobel take the
**maximum** of the absolute horizontal and vertical responses. That is neither the Euclidean magnitude nor the sum of the
absolute values.

| Method | Size | Output `P(x, y)` | Max raw value for U8 | Saturates in U8 |
|---|---|---|---|---|
| Differentiation | 2×2 | `max(\|p(x−1,y) − p\|, \|p(x−1,y−1) − p\|, \|p(x,y−1) − p\|)` | 255 | no |
| Gradient | 2×2 | `max(\|p(x,y−1) − p(x,y)\|, \|p(x−1,y−1) − p(x−1,y)\|)`: vertical variations only (horizontal edges) | 255 | no |
| Roberts | 2×2 | `max(\|p(x−1,y−1) − p(x,y)\|, \|p(x,y−1) − p(x−1,y)\|)` | 255 | no |
| Prewitt | 3×3 | `max(\|K1⋆p\|, \|K2⋆p\|)`, `K1 = [[−1,0,1],[−1,0,1],[−1,0,1]]`, `K2 = K1ᵀ` | 765 | **yes** |
| Sobel | 3×3 | `max(\|K1⋆p\|, \|K2⋆p\|)`, `K1 = [[−1,0,1],[−2,0,2],[−1,0,1]]`, `K2 = K1ᵀ` | 1020 | **yes** |
| Sigma | 3×3 | `M`, `S` = mean and standard deviation of the 3×3 block (centre included); `P = M` if `\|p − M\| ≤ S`, else `P = p` | 255 | no |

- The sign and orientation of `K1`/`K2` (correlation or convolution) do not matter because of the absolute values. The
  orientation of Gradient does matter: test A3 (step edges in both directions) checks it.
- Sigma is the least certain definition. NI's help (NI Vision, "Spatial Filtering") groups Sigma with the other five as a
  "nonlinear highpass" (contour extraction) filter, yet the formula above, as recalled from NI's documentation, is a
  smoothing operation. Open points: whether the standard deviation is the population one, whether the interval is inclusive,
  how the non-integer `M` is rounded for U8, and whether EdgeDetection uses this formula at all (A5). Defect F10 describes
  what follows if it does.

### 6.4 Output type and saturation

The edge image has the type of the source (U8 here). Values above 255 are clipped to 255 **(inferred, NI saturation
convention)**. The clipping only affects Prewitt and Sobel, and only at strong edges (a step of at least 64 grey levels for
Sobel, 86 for Prewitt), but it changes the mean on contrasted scenes. Reproduce it. For a U16 source the clip level would be
65535 **(inferred)**.

### 6.5 Border handling **(unknown)**

The filters need a one-pixel frame around the image (top row and left column for the 2×2 filters). NI images carry a border
(3 pixels by default [Doc6 p.1003]) that the filter fills internally. The fill method is **(unknown)**. For a one-pixel
frame there are only three distinguishable candidates:

| Candidate (NumPy `np.pad` mode) | OpenCV | Same as (1-pixel frame) | Signature test (A4) |
|---|---|---|---|
| `edge` (replicate), provisional default | `BORDER_REPLICATE` | `symmetric` / `BORDER_REFLECT` | constant image → all 0; ramp `64 + x` → Sobel column 0 = 4 |
| `reflect` (mirror without repeating the edge) | `BORDER_REFLECT_101` (OpenCV default) | — | constant → 0; ramp → column 0 = 0 |
| `constant` (zeros) | `BORDER_CONSTANT`, value 0 | — | constant image → non-zero frame; ramp → column 0 = 255 |

Impact: frame pixels are `(2X + 2Y − 4)/(X·Y)` of the image. That is 0.7 % for 612×512, but 4 % for a 200×200-pixel refocus
ROI decimated to 100×100, so getting it right matters for small ROIs.

### 6.6 Histogram mean

With 256 classes and an interval of `(0, 0)`, the histogram covers the full range of the type (0–255 for U8)
**(inferred)**. `Mean Value` is the arithmetic mean of all pixels (no mask), returned as SGL. For U8 the class width is 1,
so the binning cannot bias the mean. Reference: `float32(sum(E) / N)` with the sum exact in integer or double arithmetic.
For 16-bit images NI computes the interval from the image's dynamic range **(inferred)**. Whether the mean then comes from
pixel values or from bin centres is **(unknown)** (A6).

### 6.7 FULL mode: the race on `AF_FFT_DST`

The six EdgeDetection nodes write into the same NI image and no wire orders them. Each histogram therefore reads whatever
the shared image holds when it runs.

| LabVIEW schedule (unknown which one occurs) | Result |
|---|---|
| each EdgeDetection immediately followed by its Histogram (the intent) | `Π mean(E_m)` |
| all six EdgeDetections first, then the six Histograms | `mean(E_last)^6`, where `last` is whichever filter ran last |
| concurrent threads / interleaving | any mixture. A histogram may even read a partially written image |

Historical FULL scores may therefore not be reproducible. The Python port computes the intended product sequentially with
separate buffers (defect F4). Test B2 measures what LabVIEW really produced.

---

## 7. Refocus compositions: Sigma and FULL

### 7.1 "Sigma" chain [Doc6 p.1036]

```
img := IMAQ LowPass(Image Src = camera image, Image Mask unwired, Image Dst unwired (in place),
                    Size & Tolerance = {X Size 20, Y Size 20, % Tolerance 40})      # error in AND error out unwired
score := measure_focus_sobel(img, Fin ROI, 0.5, Method = Sigma, FULL = False)
```

- The filter runs on the **whole** camera image, before the ROI, and **in place**: the image displayed and saved as a PNG by
  the refocus is the filtered one (see [06-autofocus-refocus.md](06-autofocus-refocus.md)).
- LowPass errors are lost. Because `error in` is not wired either, the filter also runs after an upstream error (the metric
  VI then returns 0 anyway). If NI rejects the 20×20 size, the image stays unchanged and the score is the Sigma edge score of
  the unfiltered image **(inferred)**. Whether LabVIEW's automatic error handling then opens a dialog is **(unknown)**: it
  depends on an execution property of the refocus VI (Q30 asks the same for the main VI).

### 7.2 `IMAQ LowPass` [Doc6 p.961–962]

- Interface: `Size & Tolerance` cluster `{X Size (I32, default 3), Y Size (I32, default 3), % Tolerance (I32, default 40)}`,
  `Image Src`, `Image Mask`, `Image Dst`, `error in` → `Image Dst Out`, `error out`.
- Printed description: "Calculates the inter-pixel variation between the pixel being processed and those pixels surrounding
  it. If the pixel being processed has a variation greater than a specified percentage, it is set to the average pixel value
  as calculated from the neighboring pixels."
- Working hypothesis **(unknown)**: `M` = mean of the `X Size × Y Size` neighbourhood; if `|p − M| > (T/100)·M` then
  `p := M`, else `p` is unchanged; the result is rounded and saturated to the image type.

| Open point | Candidates | Sketch parameter | Test |
|---|---|---|---|
| reference of the "variation" | `M`, `p`, or full scale | (formula) | A9 |
| centre pixel in `M` | included / excluded | `include_center` | A9 |
| even sizes (20×20) | anchor at `k//2` (offsets −10…+9), symmetric 21×21, or error | `sigma_lowpass=False` models the error case | A9 |
| rounding of `M` for U8 | round / truncate | (`_saturate` rounds) | A9 |
| border fill (10-pixel frame) | as §6.5, plus `symmetric`, which differs from `edge` for frames wider than one pixel | `border` | A9 |

### 7.3 Consequence of the Sigma definition **(inferred)**

The Sigma formula of §6.3 replaces a pixel by the local mean when the pixel is close to it and keeps it otherwise, so its
output has about the brightness of its input. The "edge" image then has a mean close to the mean grey level of the ROI. With
the reference sketch (§9.2), uniform noise of mean 127.5 gives a Sigma score of about 127.4, and a constant image `c` gives
exactly `c`. If NI's EdgeDetection implements that formula, the "Sigma" refocus measures **brightness, not focus**. Its curve
over Z would be flat or driven by illumination changes (defect F10). Test A5 decides it.

### 7.4 "FULL" composition [Doc6 p.1036]

```
in parallel (no wire between them; both get the same image, scale 0.5 and error in):
    s1, err := measure_focus_sobel(img, Optional Rectangle = Fin ROI, 0.5, FULL = True, Method unwired)
    s2, _   := measure_focus_fft  (img, Optional Rectangle unwired → whole image, 0.5)    # error out not wired
score := s1 × s2                                                                         # DBL multiply
```

- The FFT part rates the **whole** image, while the edge part rates only the ROI.
- Both VIs share the named buffer `AF_FFT_TMP`, and each disposes of it at the end. Possible outcomes are correct values,
  values computed on the other VI's resampled image, or an "invalid image" error **(inferred)**. A Sobel-side error is
  propagated (the refocus goes to `Error`). An FFT-side error is silent and gives `s2 = 0`, hence a score of 0.
- Magnitudes: `s1 ≤ 255⁶ ≈ 2.7·10¹⁴`. `s2` is at least its DC term `X·Y·mean`, where `X × Y` is the resampled size
  (1.3·10⁸ for a 2448×2048 frame at `s = 0.5`, i.e. 1224×1024, of mean 100; the sensor size is **(unknown)**, Q7). With the
  sketch, uniform noise gives a product of about 8.8·10²³ for a 2448×2048 frame (1.1·10²³ for 1224×1024). DBL represents
  these without overflow. A client that displays or thresholds them must expect that range.
- Outside FULL, the AF and RF callers execute the metric calls one at a time in the single camera loop, so the shared names
  cause no other race **(inferred)** (see [01-architecture.md](01-architecture.md)).

---

## 8. Numerical precision

| Quantity | LabVIEW type | Python reproduction |
|---|---|---|
| ROI, resampled image | U8 (input type) | same dtype, exact |
| Edge image | U8, saturated | exact integer arithmetic, then clip |
| Histogram mean | SGL | `np.float32(sum / N)`, at most 1 ulp from NI **(inferred)** |
| FULL product (sub-VI) | SGL, five roundings | float32 product in the order Diff, Grad, Prewitt, Roberts, Sigma, Sobel (wiring order **(inferred)**), within 3 ulp |
| FFT coefficients | complex SGL (CSG) | A float64 FFT is more accurate. Measured with NumPy on 612×512 and 613×511 noise images, a single-precision FFT differs from float64 by about 3·10⁻⁸ of `max\|F\|` per coefficient (up to 2·10⁻⁵ relative on the smallest coefficients), and the plane sum by less than 10⁻⁸ relative. NI's FFT algorithm is **(unknown)**, so expect the same order of magnitude **(inferred)** |
| Magnitude array | 2-D SGL | `np.abs(F).astype(np.float32)` |
| FFT sum | SGL (`Add Array Elements` on SGL) | NI's accumulator is **(unknown)**. On 612×512 noise images (five seeds) a sequential float32 sum differs from the exact sum by 10⁻⁶ to 2·10⁻⁵ relative; a pairwise float32 sum by about 10⁻⁸. This, not the FFT, dominates the FFT-score tolerance |
| `Focus Score` | DBL holding an SGL value | `float(np.float32(x))` |
| Refocus FULL product | DBL × DBL | Python float |

Acceptance tolerances follow from this table (§10.4). The argmax of a Z scan is insensitive to all of these effects except
ties, which are rare with float scores.

---

## 9. Python reference implementation

### 9.1 Design rules

1. Pure functions on NumPy arrays. No global or shared buffers. The input is never modified. The Sigma path returns a new
   filtered array, which the caller uses as "the image" to display and save, mimicking LabVIEW's in-place filter.
2. Keep the input dtype (U8 expected). Compute in int64 or float64, saturate edge images to the input dtype, and round scores
   to float32 before returning them as Python `float`.
3. Every unknown NI behaviour is a keyword parameter with a provisional default: `phase="corner"`, `border="edge"`,
   `clip_rect=False`, `sequential_f32=False`, `include_center=True`, `sigma_lowpass=True`. Once §10 has run, freeze the
   measured values in the server configuration (`NiCompatMetrics(**params)`). They are never client-visible.
4. One entry point, `measure(img, method, roi, scale)`, serves both callers and matches the `Metrics` protocol of
   [06-autofocus-refocus.md](06-autofocus-refocus.md) §9.2: the autofocus maps every method other than `FFT` and `Prewitt`
   to `Sobel` before calling it, and the refocus passes `scale = 0.5`.
5. Errors raise `FocusMetricError`; in the implementation this is `FocusError` of
   [09-configuration-logging-errors.md](09-configuration-logging-errors.md) §10.2, with the code proposed there (§10.5), because
   NI's error codes for these failures are **(unknown)**. `measure_lv_style` shows the LabVIEW contract `(image, 0.0, error)`,
   which the state machines of 06 reproduce by recording score 0.
6. Deliberate deviations from LabVIEW, all listed in §11: FULL is computed sequentially (F4), and an FFT error in the refocus
   FULL composition is propagated instead of giving a silent 0 (F6).
7. `Timer (ms)` is not implemented. Callers time the call themselves if they need to.

### 9.2 Reference sketch

Reference sketch. It was executed against the checks of §10 that need no NI data (rounding, phases, rfft identity,
constant-image FFT, border signatures, box filter versus brute force) and through every method of `measure`, including the
error paths. NumPy only; NumPy ≥ 2.0 can also run the FFT in single precision (`np.fft.fft2` of a float32 array returns
complex64), which is closer to NI's CSG.

```python
"""Reference sketch of the ALOX focus metrics (docs/07-focus-metrics.md).

Unknown NI behaviours are exposed as keyword parameters (phase, border, clip_rect, sequential_f32,
sigma_lowpass, include_center); their defaults are provisional until the validation plan
(section 10) has been run.
"""
from __future__ import annotations

from typing import Optional, Sequence, Tuple

import numpy as np

# IMAQ EdgeDetection methods in the order of the FULL frame [Doc6 p.957]; NI enum values 0..5 (inferred).
EDGE_METHODS = ("Differentiation", "Gradient", "Prewitt", "Roberts", "Sigma", "Sobel")
BORDERS = ("edge", "reflect", "constant")      # numpy pad modes; NI's actual fill is unknown


class FocusMetricError(Exception):
    """An NI Vision call inside a metric VI failed: LabVIEW then returns score 0.
    In the implementation this is FocusError (09); the 06 sketches catch it as LVError."""
    image: Optional[np.ndarray] = None   # image the caller keeps (Sigma: already filtered)


def lv_to_i32(x: float) -> int:
    """LabVIEW 'To Long Integer': round to nearest, ties to even, saturate to I32."""
    if not np.isfinite(x):
        raise FocusMetricError(f"non-finite size {x!r}")
    return int(min(max(np.rint(x), -2**31), 2**31 - 1))


def extract_roi(img: np.ndarray, rect: Optional[Sequence[int]], clip_rect: bool = False) -> np.ndarray:
    """IMAQ Extract 2, step 1x1. rect = [Left, Top, Right, Bottom], Right/Bottom exclusive (inferred).
    Empty or None -> whole image. Any non-empty rect (even [0,0,0,0]) is applied."""
    if rect is None or len(rect) == 0:
        return img
    if len(rect) != 4:
        raise FocusMetricError(f"Optional Rectangle has {len(rect)} elements")
    left, top, right, bottom = (int(v) for v in rect)
    h, w = img.shape[:2]
    if clip_rect:                                  # NI behaviour for out-of-image rects is unknown
        left, right = min(max(left, 0), w), min(max(right, 0), w)
        top, bottom = min(max(top, 0), h), min(max(bottom, 0), h)
    if not (0 <= left < right <= w and 0 <= top < bottom <= h):
        raise FocusMetricError(f"rectangle {list(rect)} empty or outside {w}x{h} image")
    return img[top:bottom, left:right]


def resample_nearest(img: np.ndarray, x_res: int, y_res: int, phase: str = "corner") -> np.ndarray:
    """IMAQ Resample, Zero Order. Destination pixel (x, y) takes source pixel (sx(x), sy(y))."""
    if x_res <= 0 or y_res <= 0:
        raise FocusMetricError(f"resample size {x_res}x{y_res}")
    h, w = img.shape[:2]
    xd = np.arange(x_res, dtype=np.int64)
    yd = np.arange(y_res, dtype=np.int64)
    if phase == "corner":            # floor(x*W/X)            (= cv2.INTER_NEAREST up to float rounding)
        xs, ys = xd * w // x_res, yd * h // y_res
    elif phase == "corner_round":    # floor(x*W/X + 1/2)
        xs, ys = (2 * xd * w + x_res) // (2 * x_res), (2 * yd * h + y_res) // (2 * y_res)
    elif phase == "center":          # floor((x + 1/2)*W/X)    (= cv2.INTER_NEAREST_EXACT)
        xs, ys = (2 * xd + 1) * w // (2 * x_res), (2 * yd + 1) * h // (2 * y_res)
    else:
        raise ValueError(phase)
    return img[np.ix_(np.minimum(ys, h - 1), np.minimum(xs, w - 1))]


def subsample(img: np.ndarray, rect, scale: float, phase: str = "corner", clip_rect: bool = False) -> np.ndarray:
    """Common front end of both metric VIs: ROI, then nearest-neighbour resize to rint(W*s) x rint(H*s)."""
    if img.ndim != 2:
        raise FocusMetricError("grayscale image expected")
    roi = extract_roi(img, rect, clip_rect)
    h, w = roi.shape
    return resample_nearest(roi, lv_to_i32(w * scale), lv_to_i32(h * scale), phase)


def focus_fft(img: np.ndarray, rect=(), scale: float = 0.25, *, phase: str = "corner",
              clip_rect: bool = False, sequential_f32: bool = False) -> float:
    """alox_vision_measure_focus_fft.vi: sum of |DFT| over the whole plane (DC included)."""
    s = subsample(img, rect, scale, phase, clip_rect)
    mag = np.abs(np.fft.fft2(s.astype(np.float64))).astype(np.float32)   # Magnitude plane, SGL
    if sequential_f32:               # emulates a running float32 accumulator
        total = np.cumsum(mag.ravel(), dtype=np.float32)[-1]
    else:
        total = np.float32(mag.sum(dtype=np.float64))
    return float(total)              # SGL -> DBL


def _saturate(out: np.ndarray, dtype: np.dtype) -> np.ndarray:
    dt = np.dtype(dtype)
    if dt.kind in "ui":
        info = np.iinfo(dt)
        if out.dtype.kind == "f":
            out = np.rint(out)       # rounding of non-integer results (Sigma) is inferred
        return np.clip(out, info.min, info.max).astype(dt)
    return out.astype(dt)


def ni_edge(img: np.ndarray, method: str, border: str = "edge") -> np.ndarray:
    """IMAQ EdgeDetection with Threshold 0, no mask. Formulas from NI documentation (not in the
    printouts). border: numpy pad mode used for the 1-pixel frame ('edge' = replicate, equal to
    'symmetric' for a 1-pixel frame; 'reflect' = mirror without repeating the edge; 'constant' = 0)."""
    integer = img.dtype.kind in "ui"
    a = img.astype(np.int64 if integer and method != "Sigma" else np.float64)
    p = np.pad(a, 1, mode=border)
    c = p[1:-1, 1:-1]
    up, down, left, right = p[:-2, 1:-1], p[2:, 1:-1], p[1:-1, :-2], p[1:-1, 2:]
    ul, ur, dl, dr = p[:-2, :-2], p[:-2, 2:], p[2:, :-2], p[2:, 2:]
    if method == "Differentiation":
        out = np.maximum.reduce([np.abs(left - c), np.abs(ul - c), np.abs(up - c)])
    elif method == "Gradient":
        out = np.maximum(np.abs(up - c), np.abs(ul - left))
    elif method == "Roberts":
        out = np.maximum(np.abs(ul - c), np.abs(up - left))
    elif method in ("Prewitt", "Sobel"):
        k = 1 if method == "Prewitt" else 2
        gx = (ur + k * right + dr) - (ul + k * left + dl)
        gy = (dl + k * down + dr) - (ul + k * up + ur)
        out = np.maximum(np.abs(gx), np.abs(gy))
    elif method == "Sigma":          # least certain definition (see 6.3)
        stack = np.stack([ul, up, ur, left, c, right, dl, down, dr])
        m, sd = stack.mean(axis=0), stack.std(axis=0)      # population std (inferred)
        out = np.where(np.abs(c - m) <= sd, m, c)
    else:
        raise ValueError(method)
    return _saturate(out, img.dtype)


def histogram_mean(img: np.ndarray) -> np.float32:
    """IMAQ Histogram -> Mean Value (SGL), whole image, no mask."""
    return np.float32(img.sum(dtype=np.float64) / img.size)


def focus_edge(img: np.ndarray, rect=(), scale: float = 0.25, method: str = "Sobel",
               full: bool = False, *, phase: str = "corner", border: str = "edge",
               clip_rect: bool = False) -> float:
    """alox_vision_measure_focus_sobel.vi: mean of the edge image, or product of the 6 means if FULL."""
    s = subsample(img, rect, scale, phase, clip_rect)
    if not full:
        return float(histogram_mean(ni_edge(s, method, border)))
    prod = np.float32(1.0)
    for m in EDGE_METHODS:           # sequential, separate buffers (fixes the LabVIEW race)
        prod = np.float32(prod * histogram_mean(ni_edge(s, m, border)))
    return float(prod)


def ni_lowpass(img: np.ndarray, x_size: int = 3, y_size: int = 3, tolerance_pct: float = 40.0,
               border: str = "edge", include_center: bool = True) -> np.ndarray:
    """IMAQ LowPass -- HYPOTHESIS ONLY (formula unknown, see 7.2): pixel replaced by the
    neighbourhood mean M when |p - M| > tolerance% * M. Window anchor k//2 (OpenCV convention)."""
    a = img.astype(np.float64)
    ax, ay = x_size // 2, y_size // 2
    p = np.pad(a, ((ay, y_size - 1 - ay), (ax, x_size - 1 - ax)), mode=border)
    c = np.pad(p.cumsum(axis=0).cumsum(axis=1), ((1, 0), (1, 0)))
    h, w = a.shape
    s = c[y_size:y_size + h, x_size:x_size + w] - c[:h, x_size:x_size + w] \
        - c[y_size:y_size + h, :w] + c[:h, :w]
    n = x_size * y_size
    m = s / n if include_center else (s - a) / (n - 1)
    out = np.where(np.abs(a - m) > tolerance_pct / 100.0 * m, m, a)
    return _saturate(out, img.dtype)


# --- Method dispatch of the two callers (06-autofocus-refocus.md §4.5 and §5.7) ------------
REFOCUS_EDGE = {"Sobel": "Sobel", "Prewitt": "Prewitt", "Diff": "Differentiation", "Gradient": "Gradient"}


def measure(img: np.ndarray, method: str, roi, scale: float, *, phase: str = "corner",
            border: str = "edge", clip_rect: bool = False, sequential_f32: bool = False,
            sigma_lowpass: bool = True, include_center: bool = True) -> Tuple[np.ndarray, float]:
    """Returns (image, score). The image is the LowPass-filtered copy for "Sigma", else the input.
    The autofocus passes only "FFT", "Prewitt" or "Sobel": it maps every other method to "Sobel"
    before the call (06). "Prewitt" assumes the unwired FULL input defaults to False (Q30)."""
    edge_kw = dict(phase=phase, border=border, clip_rect=clip_rect)
    fft_kw = dict(phase=phase, clip_rect=clip_rect, sequential_f32=sequential_f32)
    if method == "FFT":
        return img, focus_fft(img, roi, scale, **fft_kw)
    if method in REFOCUS_EDGE:
        return img, focus_edge(img, roi, scale, REFOCUS_EDGE[method], **edge_kw)
    if method == "Sigma":            # refocus only: LowPass on the WHOLE image (in place in LabVIEW)
        if sigma_lowpass:            # False models NI rejecting the 20x20 kernel (error unwired)
            img = ni_lowpass(img, 20, 20, 40, border, include_center)
        try:
            return img, focus_edge(img, roi, scale, "Sigma", **edge_kw)
        except FocusMetricError as e:
            e.image = img            # LabVIEW keeps the filtered image even when the metric fails
            raise
    if method == "FULL":             # refocus only: six-filter product on the ROI x FFT on the WHOLE image
        return img, focus_edge(img, roi, scale, full=True, **edge_kw) * focus_fft(img, (), scale, **fft_kw)
    raise ValueError(method)         # the refocus case structure has no Default case


class NiCompatMetrics:
    """The Metrics object of 06 (measure(img, method, roi, scale) -> (image, score)), with the NI
    behaviour parameters frozen after the validation campaign (section 10)."""

    def __init__(self, **ni_params):
        self.ni_params = ni_params

    def measure(self, img: np.ndarray, method: str, roi, scale: float) -> Tuple[np.ndarray, float]:
        return measure(img, method, roi, scale, **self.ni_params)


def measure_lv_style(img, method, roi, scale, **kw) -> Tuple[np.ndarray, float, Optional[Exception]]:
    """LabVIEW contract of the metric VIs: on error the score is 0 and the error is returned."""
    try:
        out, score = measure(img, method, roi, scale, **kw)
        return out, score, None
    except FocusMetricError as e:
        return (img if e.image is None else e.image), 0.0, e
```

### 9.3 OpenCV equivalents and traps

| Step | OpenCV | Trap |
|---|---|---|
| ROI | slicing | — |
| Resample `corner` / `center` | `cv2.resize(roi, (X, Y), interpolation=cv2.INTER_NEAREST)` / `cv2.INTER_NEAREST_EXACT` | `dsize` is `(width, height)`. The source index is computed in floating point, so it can differ by one pixel from the integer formula for non-integer ratios **(inferred)**. Keep the NumPy indexing as the reference. |
| Prewitt / Sobel | `cv2.filter2D(s, cv2.CV_32F, K, borderType=…)` for `K1` and `K2` (for Sobel, `cv2.Sobel(s, cv2.CV_32F, 1, 0, ksize=3, borderType=…)` and `(…, 0, 1, …)` give the same `Gx`, `Gy`), then `np.maximum(abs, abs)` and clip | Do **not** use `cv2.magnitude` or `\|Gx\| + \|Gy\|`. Always pass `borderType`, because the OpenCV default is `BORDER_REFLECT_101`. `filter2D` correlates (no kernel flip), which does not matter here because of the absolute values. |
| Differentiation, Gradient, Roberts, Sigma | no direct equivalent | NumPy slicing as in the sketch |
| Mean | `cv2.mean(e)[0]` | round to float32 |
| FFT | `F = cv2.dft(np.float32(s), flags=cv2.DFT_COMPLEX_OUTPUT)`, then `cv2.magnitude(F[..., 0], F[..., 1])` | float32 arithmetic, closer to NI's CSG **(inferred)**; `np.fft.fft2` on float32 input does the same with NumPy ≥ 2.0 |
| LowPass box mean | `cv2.blur(a, (kx, ky), borderType=…)` | For an even size the anchor is `k//2`, the same as the sketch. Faster than the NumPy integral image of the sketch. |

---

## 10. Numerical validation plan against NI Vision

The goal is to settle every **(unknown)** of §1.3, to fix the provisional parameters of §9, and to prove score equivalence.
Tier A needs a LabVIEW 2018 + NI Vision 2018 machine (Q29). Tier B needs only the running LabVIEW server and its normal
outputs.

### 10.1 Tier A harness (LabVIEW side)

A small test VI reads each test image (`IMAQ ReadFile`, 8-bit grey PNG), calls **one NI function** with explicit parameters
(no ALOX code), and exports the result:

- integer images (U8/U16) as PNG with `IMAQ WriteFile` (lossless);
- float and complex results as arrays (`IMAQ ImageToArray`; for complex images `IMAQ ComplexPlaneToArray` with Plane = Real
  and Imaginary) saved with `Write to Binary File`. LabVIEW writes **big-endian** and, by default, prepends one I32 size per
  dimension **(inferred)**. Read in Python with `np.frombuffer(b, '>i4', 2)` for the shape and `'>f4'` for the data;
- scalars, error codes and error sources into a CSV line per test case.

The test images come from a Python generator committed with the tests (fixed seeds), so both sides use identical inputs.

### 10.2 Test images

| ID | Content (U8) | Purpose |
|---|---|---|
| T1 | constant 100, 640×480 | border = zeros? FFT = `c·X·Y`? |
| T2 | ramp `64 + (x mod 128)` and its transpose, 640×480 | border mode, Gradient orientation |
| T3 | index images `x mod 256` and `y mod 256`, 2448×2048 and 1002×750 | resample phase, ROI exclusivity |
| T4 | vertical and horizontal step edges 0/255; checkerboards 1 px and 8 px | saturation, orientation, Sigma |
| T5 | uniform noise (seed 1) at 640×480, 612×512, 613×511 | general equality, FFT size/scaling |
| T6 | single impulse 100 on 0, at the centre and at (0, 0) | kernel footprint, border |
| T7 | real Z stacks (≥ 21 frames per camera, through focus), after flip/flat field | end-to-end, argmax |

### 10.3 Test matrix

| ID | NI call | Inputs and parameters | Decides | Pass criterion |
|---|---|---|---|---|
| A1 | IMAQ Resample, Zero Order | T3: 2448→612 and 1224; 1002→250; 750→188 | `phase` | exact equality for one candidate |
| A2 | IMAQ Extract 2 | T3: `[10,20,110,220]`, `[0,0,0,0]`, `[−5,0,100,100]`, `[2400,0,2500,100]`, 3 and 5 elements | exclusivity, clip vs error (`clip_rect`), error codes | size 100×200 if exclusive; record errors |
| A3 | IMAQ EdgeDetection ×6 | T4, T6 | formulas, Gradient orientation | exact |
| A4 | IMAQ EdgeDetection ×6 | T1, T2 | `border` (signatures §6.5) | exact including the frame |
| A5 | IMAQ EdgeDetection ×6 | T5, T7 | saturation, Sigma definition and rounding | exact (Sigma: identify the formula) |
| A6 | IMAQ Histogram | outputs of A5; also a U16 image | `Mean Value` definition | within 1 ulp (float32) |
| A7 | IMAQ FFT + ComplexPlaneToArray (Real, Imaginary, Magnitude) | T1, T5 (three sizes) | output size (padding), scaling, layout | max abs error ≤ 10⁻⁵ · max `\|F\|`; T1 gives `c·X·Y` at (0, 0) only |
| A8 | Add Array Elements on the A7 magnitude array | T5 | accumulator (`sequential_f32`) | relative ≤ 10⁻⁶ against the chosen emulation |
| A9 | IMAQ LowPass | T2, T4, T5; sizes {3,3}, {5,5}, {4,4}, {20,20}; tolerances 0, 40, 100 | formula, centre, even sizes, rounding, border | exact; record errors |
| A10 | Resample, EdgeDetection and FFT with a U16 source and a U8 work image (as in the VIs) | U16 noise | type handling | record outputs or errors |
| A11 | Degenerate cases | Resample to 0×0, 1×1; FFT of 1×1 | errors versus outputs | record |

### 10.4 Tier B: end-to-end checks

| ID | Procedure | Pass criterion |
|---|---|---|
| B1 | Call the two ALOX VIs directly from a test VI on T5 and T7: scales 0.25 and 0.5, ROI empty and set, the six methods, FULL False | FFT relative error ≤ 10⁻⁴; edge relative error ≤ 10⁻⁶; identical argmax on every Z stack |
| B2 | FULL = True, 50 repeated calls on the same image; refocus "FULL" composition 50 times | Record the distribution and compare it with the intended product and with `mean(E_m)^6`. This documents the non-determinism (F4) and is not a pass/fail test. |
| B3 | Without a Vision machine: run real refocus scans with `Save Images = True` for each method, then read `Cam Get Autofocus Data` | The saved PNG at index *i* is the metric's input image (after LowPass for Sigma) and its score is row *i*. Recompute the Fin ROI from `Fin ROI Width/Height` and the image size (06 §5.6) and apply the B1 tolerances. |
| B4 | Python regression suite | Golden files from A and B stored under `tests/focus/golden/`. Pytest parametrised over (test, parameters). Tolerances as above. Plus property tests: a constant image `c` gives edge score 0 for every method except Sigma (score `c`) and except with `border='constant'`, and FFT score `c·X·Y`. As a sanity check (not fidelity), FFT and Sobel scores decrease along a synthetic blur series. |

B3 cannot validate `IMAQ LowPass` itself, because the saved Sigma image is already filtered. It does validate all the other
steps on real data.

### 10.5 Decision record

After the campaign, write one table into the test README: parameter → measured value → test ID → date. Examples:
`phase = corner (A1)`, `border = edge (A4)`, `clip_rect = False (A2)`. Then remove the parameters that turned out not to be
needed.

---

## 11. Defects and oddities in the LabVIEW implementation

| # | Finding | Evidence | Impact | Recommendation |
|---|---|---|---|---|
| F1 | The FFT score includes the DC term (sum of intensities). No mean subtraction, window or band selection. | [Doc6 p.948] | The score depends on brightness and gain; the curve contrast is reduced. | **Reproduce**. Ask whether an optional DC-free variant is wanted. |
| F2 | Decimation by plain nearest neighbour (no anti-aliasing). Only one pixel in `1/s²` is used. | [Doc6 p.948, p.957] | Aliasing; dependence on the sampling phase | **Reproduce** (the phase comes from A1). |
| F3 | Work images are process-wide named buffers. `AF_FFT_TMP` is shared by both VIs, and each disposes of it at the end of every call. | [Doc6 p.948, p.957] | Race in the refocus FULL composition (F5); allocation churn on every call | **Fix**: local arrays. |
| F4 | FULL mode: six EdgeDetections write the same destination in parallel. Each histogram reads an unpredictable image. Only the Differentiation branch carries errors. | [Doc6 p.957] | Non-deterministic FULL scores, possibly `mean(E_last)^6`; five errors lost | **Fix** (sequential, separate buffers). **Ask** whether old FULL values must remain comparable (B2 quantifies it). |
| F5 | Refocus FULL runs the Sobel-FULL VI and the FFT VI in parallel on the same named buffer. The FFT rates the whole image while the edge part rates the ROI. | [Doc6 p.1036] | Possible wrong values or errors; ROI inconsistency | **Fix** the race. **Ask** about the whole-image FFT; keep it by default. |
| F6 | Refocus FULL: the FFT VI's `error out` is not wired. An FFT failure gives 0 and the product is silently 0. | [Doc6 p.1036] | A silent zero score at that position | **Fix**: propagate the error (deviation from LabVIEW). |
| F7 | FULL multiplies seven quantities with unrelated units and ranges (about 9·10²³ on a 2448×2048 noise frame, §7.4). | [Doc6 p.957, p.1036] | Hard to display or threshold; any zero factor zeroes the score | **Reproduce**. Document the range for clients. |
| F8 | Sigma: `IMAQ LowPass` filters the camera image in place, with `error in` and `error out` unwired. | [Doc6 p.1036] | Displayed and saved images are filtered; filter failures are invisible | **Reproduce** by default. **Ask** whether saved images should be unfiltered (06 D18). |
| F9 | LowPass kernel 20×20 is even and large. NI may reject it or anchor it asymmetrically **(unknown)**. | [Doc6 p.1036] | The filter may have no effect at all (Q27) | **Validate** (A9), then reproduce what NI does (`sigma_lowpass`). |
| F10 | The Sigma formula recalled from NI's documentation is a smoothing operation, so the Sigma score would be about the mean grey level of the ROI **(inferred)**. | NI documentation, §6.3, §7.3 | The "Sigma" refocus may measure brightness, not focus | **Validate** (A5). **Ask** whether "Sigma" was ever used and what it should measure. |
| F11 | Any error gives `Focus Score = 0`, which looks like a legitimate score. The error case does not write `Timer (ms)`. | [Doc6 p.949, p.959] | Callers record 0 as data ([06](06-autofocus-refocus.md) D15) | **Fix** internally (exception). Keep the 0 where 06 requires it on the wire. Drop the Timer. |
| F12 | Only an **empty** ROI means "whole image". `[0,0,0,0]`, inverted, out-of-image and non-4-element rectangles go to `IMAQ Extract 2` unvalidated, and neither caller clips the refocus ROI. | [Doc6 p.948, p.957] | NI-defined error or clipping **(unknown)** | **Reproduce** NI's behaviour once A2 has measured it. Until then the sketch raises `FocusMetricError` (score 0). |
| F13 | `Subsampling = 0` (the autofocus typedef default) gives a 0×0 resample, so error and score 0 **(inferred)**. | [Doc6 p.600, p.948] | An autofocus with default parameters yields only zeros | Metric: **reproduce** (error, score 0). Command: validate at the start, as [06](06-autofocus-refocus.md) D8 recommends. **Ask** whether to substitute the VI default 0.25 instead. |
| F14 | The `FULL` default is not printed. The autofocus `Prewitt` case leaves it unwired. If the default were True, autofocus "Prewitt" would compute the six-filter product. | [Doc6 p.605, p.956] | Wrong method in autofocus Prewitt | **Ask** (Q30). Assume False. |
| F15 | The work images are created as U8 whatever the input type. A Mono16 camera turns the source image into U16 (`IMAQdx Get Image2` adapts the type), which would mix types. | [Doc6 p.895, p.948, p.957, p.1003] | Behaviour **(unknown)** (error or conversion) | **Ask** for the pixel format (Q7). Python works in the input dtype. |
| F16 | Dead code: `First Quadrant?` constant False; Timer in frame 2 unordered with Dispose; icon caption "PREWITT" on the Sobel VI. | [Doc6 p.948, p.958] | none | **Drop**. |
| F17 | Scores are SGL sums and products (float32 granularity, order-dependent rounding). | [Doc6 p.948, p.957] | Last-digit differences between implementations | **Reproduce** float32 rounding; use tolerances (§10). |

---

## 12. Open questions

Only questions that change the implementation or its acceptance:

1. **Required fidelity.** Must scores match LabVIEW numerically (clients that threshold them, comparisons with historical
   data), or is the same curve shape and argmax enough? The answer decides whether Tier A of §10 is mandatory.
2. **LabVIEW 2018 + Vision machine (Q29).** It is needed for Tier A (resampling phase, border, FFT scaling, Sigma and LowPass
   formulas). Without it only B3 is possible, and LowPass stays a hypothesis.
3. **NI Vision Runtime allowed on the new server (Q6)?** If yes, calling NI Vision's C library through `ctypes` for
   resample, edge detection, histogram, FFT and LowPass would reproduce NI's own outputs without reverse engineering
   **(inferred: the library exports these functions)**; only LabVIEW's SGL sum and products (§8) would still be emulated. If
   no, the NumPy implementation of §9 is the only route.
4. **Camera pixel format (Q7).** U8 or U16? It determines saturation levels, histogram behaviour and whether F15 applies.
5. **`FULL` default (Q30).** It decides what autofocus "Prewitt" computes (F14).
6. **FULL semantics (F4, F5).** Should FULL keep the whole-image FFT factor and the product form, and must old FULL values stay
   comparable?
7. **"Sigma" method (F8–F10).** Is it used in production? Should the saved and displayed images stay filtered? Is a
   brightness-like score acceptable, or should Sigma be redefined?
8. **Out-of-image ROI policy (F12).** If NI clips, reproduce the clipping. If NI fails, should the new server fail too or clip?
   This interacts with the uncentred Low Mag refocus ROI (Q21).
