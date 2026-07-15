# Documentation Rules

- All design documents, architecture notes, and API references go into `.docs/` as Markdown (`.md`) files.
- Reference PDFs stay in `.docs/papers/`.
- Do not create Markdown files outside `.docs/` except for `README.md` and `CLAUDE.md`.
- File naming: `snake_case_topic.md` (e.g. `.docs/fusion_factor_graph.md`).
- Do not commit binary data, model weights, or large datasets — use `data/` (gitignored) or document download steps in `.docs/`.
