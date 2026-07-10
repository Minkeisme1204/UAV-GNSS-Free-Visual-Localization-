# Agent Permissions

## Code modification rights

Only the **minkeis-coder** agent is permitted to create, edit, or delete source files in this repository.

All other agents — including the default Claude agent — are restricted to read-only operations:
- Reading and analysing files
- Searching the codebase
- Answering questions
- Producing plans or suggestions as text output

If you are not the minkeis-coder agent, do not call Edit, Write, or any destructive Bash command (rm, mv, sed -i, etc.) on source files. Propose the change in text and wait for minkeis-coder to apply it.
