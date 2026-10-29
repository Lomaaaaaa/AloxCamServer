# Alox Camera Server: migration analysis

This folder specifies the LabVIEW 2018 **Alox Camera Server** (`alox_camera_server.vi`, ALOX Vision Core, module `extcamera`)
in enough detail to re-implement it in another language. Python is the recommended target.

## Reading order

| Start here | |
|---|---|
| [10-migration-plan.md](10-migration-plan.md) | Recommendation (language, target design), library mapping, phased plan, validation strategy, risks |
| [open-questions.md](open-questions.md) | **Questionnaire for the system owner**: fill-in questions that block or refine the design |

| Chapter | Content |
|---|---|
| [01-architecture.md](01-architecture.md) | What the server is, start-up/shutdown sequence, the three queues and three loops, shared state, concurrency hazards, Python thread design |
| [02-tcp-protocol.md](02-tcp-protocol.md) | Wire format byte by byte: frames, LabVIEW flattening, variants, replies, heartbeat, notifier correlation, compatibility switches |
| [03-command-reference.md](03-command-reference.md) | API catalogue: every command (camera, Z, `Exit`, internal), routing by enum range, inputs, behaviour, replies, errors |
| [04-data-types.md](04-data-types.md) | Every typedef (cluster order, types, defaults, evidence), enums, Python dataclasses and flatten/unflatten sketch |
| [05-camera-acquisition.md](05-camera-acquisition.md) | NI-IMAQdx session lifecycle, GenICam attributes, acquisition state machine, flip / flat-field / metadata, simulation, LED/bulb, camera SDK options |
| [06-autofocus-refocus.md](06-autofocus-refocus.md) | Autofocus (slow/fast) and refocus state machines, Z interaction, returned data, image logging |
| [07-focus-metrics.md](07-focus-metrics.md) | Exact maths of the FFT and edge-detection sharpness scores, NI conventions, NumPy reference, numerical validation plan |
| [08-z-stage.md](08-z-stage.md) | PI Mercury (GCS 2.0) controller over RS-232: every Z command and GCS frame, referencing, waits, macros, `pipython` replacement |
| [09-configuration-logging-errors.md](09-configuration-logging-errors.md) | `alox_setup.ini` / `alox.ini` keys and NI INI rules, folders, log files, error clusters and custom error codes, Python error design |

## How this analysis was made

The source code was not available as `.vi` files. Everything was reconstructed from LabVIEW printouts:

- the PDF "VI documentation" of the whole VI hierarchy (273 VIs and controls, about 85 of them ALOX code; the rest are NI, PI and
  OpenG libraries);
- the HTML export of the main VI (254 block-diagram images, including every hidden case frame).

Each subsystem was analysed independently, then each chapter was written, reviewed adversarially against the printouts, and
checked for consistency with the other chapters.

### Conventions

- **(inferred)**: deduced from the evidence (wiring, conventions, cross-references) but not directly visible.
  **(unknown)**: cannot be determined from the printouts. Untagged statements are observed facts.
- Citations: `[Doc6 p.N]` is page N of the PDF documentation of the VI hierarchy ("Labview Document 6.pdf"). `[main VI p.N]` is
  page N of the main VI's PDF export. `[main VI frame dN]` is block-diagram image N of the main VI's HTML export (`d0` = main
  diagram, `d1`…`d253` = hidden frames).
- Each chapter ends with the defects found in the LabVIEW code (with a reproduce / fix / ask recommendation) and its open questions.

## Status

- The architecture, the command set, the algorithms, the Z-stage protocol and the configuration are fully specified from the printouts.
- A few facts **cannot be read from printouts** and are collected in [open-questions.md](open-questions.md):
  - the numeric values and order of the enums, which define the wire codes and the command routing;
  - the exact byte format of LabVIEW variants and of NI's image serialisation (a network capture settles both);
  - the camera and controller models;
  - the real INI files.
