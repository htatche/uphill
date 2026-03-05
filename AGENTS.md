# AGENTS.md

This file defines how coding agents should work in this repository.

## Project Overview
- Stack: TypeScript + Vite
- Entry point: `src/main.ts`
- Core modules: `src/api.ts`, `src/graph.ts`, `src/mapUI.ts`
- Type definitions: `src/types/*.ts`
- Build output: `dist/`

## Goals For Agent Work
- Keep changes minimal and focused on the requested behavior.
- Preserve strict typing and avoid `any` unless explicitly justified.
- Prefer small, composable functions over large inlined logic.
- Keep browser code lightweight and readable.

## Commands
- Install deps: `npm ci`
- Dev server: `npm run dev`
- Build: `npm run build`
- Preview build: `npm run preview`

## Code Style
- Use TypeScript types/interfaces from `src/types` when possible.
- Add explicit return types on exported functions.
- Add JSDoc/TSDoc comments for every non-trivial function and class; trivial ones can omit docs.
- Prefer early returns to reduce branching complexity.
- Avoid introducing new dependencies unless needed.
- Keep file/module names aligned with current naming style.

## Validation Checklist (before finishing)
1. Run `npm run build` and ensure it succeeds.
2. If behavior changed, test quickly in `npm run dev`.
3. Confirm no unrelated files were modified.
4. Update docs if public behavior or setup changed.

## File-Level Guidance
- `src/main.ts`: app bootstrap and high-level wiring only.
- `src/api.ts`: network/data-fetching concerns.
- `src/graph.ts`: graph/domain logic.
- `src/mapUI.ts`: map rendering and interactions.
- `src/types/`: shared domain and API types.
