# Config Storage Migration

## Goal

Introduce a database-backed persistence layer for `extender_ui` applications and screen configurations without losing the current JSON-based assets during the migration.

The backend role is to become the durable storage boundary used by the UI while still preserving JSON import/export compatibility.

## Constraints

- no existing JSON configuration file is deleted during the migration
- import behavior must be additive/upsert by default
- export must preserve payload structure so rollback stays possible
- tests must exist before the database becomes the source of truth

## Current Status

- done:
  - SQLite storage layer exists and is tested
  - payload-level import/export helpers exist and preserve structured widget payloads
  - a FastAPI router exposes additive storage snapshot endpoints
  - the storage API now follows a more idiomatic FastAPI structure with router installation and request-time dependency resolution
  - the storage API is covered by real HTTP integration tests over uvicorn
  - the WebSocket server can include that router without changing the current UI behavior
- not switched yet:
  - the storage API is not the frontend default path
  - JSON/localStorage flows remain the active source of truth

## Phase 1

### Scope

- add a tested SQLite repository inside `tablet_interface`
- keep it isolated from the production WebSocket/API flow for now
- keep current frontend behavior unchanged

### Backend deliverables

- schema initialization
- typed repository methods for:
  - applications
  - configurations
- snapshot import/export helpers
- tests that verify:
  - schema creation
  - upsert behavior
  - round-trip payload integrity
  - no-loss additive import

### Frontend contract during phase 1

- the UI still uses localStorage and folder sync
- frontend repositories are introduced so the backend API can replace them later without another page-level refactor

### Exit criteria

- SQLite repository is stable and tested
- backend test suite remains green
- no production storage path has been switched yet

### Phase 1 status

- complete for this iteration
- verified with:
  - `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./.venv/bin/pytest`
  - `uv run python -m compileall tablet_interface`

## Phase 2

### Scope

- expose CRUD endpoints from `tablet_interface`
- make the backend database the source of truth
- keep JSON import/export tooling for compatibility and recovery

### Backend deliverables

- API endpoints for applications/configurations
- initial import from existing JSON payloads
- export endpoint or utility for JSON snapshots
- startup path that can seed an empty database without deleting bundled JSON defaults

### Frontend deliverables

- HTTP-backed repository implementation
- switch default persistence from browser local storage to backend storage
- keep explicit export/import UI actions for JSON-based workflows

### Exit criteria

- database-backed flow is default
- JSON folder sync is no longer the source of truth
- rollback/export path still exists

### Next Phase 2 slices

- add per-domain CRUD endpoints or a coordinated snapshot service layer for the UI repositories
- wire the snapshot API behind a dedicated frontend repository implementation
- seed an empty database from existing JSON assets without deleting or rewriting those assets
- decide explicit semantics for deletions before the database becomes the default source of truth

## Safety Checks

- run `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./.venv/bin/pytest`
- keep import/export tests around as permanent regression coverage
- do not mix schema design and endpoint rollout in one big step
