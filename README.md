# AloxCamServer

Re-implementation of the ALOX Vision Core **Alox Camera Server** (`alox_camera_server.vi`, LabVIEW 2018).

The LabVIEW server is a TCP server that drives:

- two cameras (High Mag / Low Mag) through NI-IMAQdx;
- a PI Mercury Z stage through the PI GCS driver over RS-232;
- LED and bulb lighting through the cameras' digital outputs;
- autofocus and refocus routines.

## Documentation

The LabVIEW code has been analysed into a specification for the re-implementation. It is in [`docs/`](docs/README.md).

- [`docs/README.md`](docs/README.md): index and reading order.
- [`docs/10-migration-plan.md`](docs/10-migration-plan.md): recommended language, target design, phased plan, risks.
- [`docs/open-questions.md`](docs/open-questions.md): questions for the system owner, as a fill-in questionnaire.
