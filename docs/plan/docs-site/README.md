---
status: in-progress
issue: branes-ai/reflex#50
---
# Plan: generate the site from docs/, with docs-kit

Issue: [#50](https://github.com/branes-ai/reflex/issues/50), under the epic [architecture#3](https://github.com/branes-ai/architecture/issues/3) (one docs-site workflow for every repo, generated from `docs/`). It builds on [architecture#4](https://github.com/branes-ai/architecture/issues/4) (the shared generator, [docs-kit](https://github.com/branes-ai/docs-kit)) and [architecture#5](https://github.com/branes-ai/architecture/issues/5) (MDX pages in `docs/`, docs-kit `v0.2.0`).

## Goal

reflex's site is generated from `docs/`, like anatomy's, racing-drone's and synapse's. The documents we write and review live in `docs/`, where GitHub renders them, and a broken relative link or anchor fails the PR. Nothing is written by hand under `docs-site/src/content/`.

## Design

- **The pages move to `docs/`** at the same paths, so every route (`/control/pid/`, …) stays. `docs/index.mdx` is the landing page. The 19 MDX pages import `TracePlayer`, the traces and the figures through docs-kit's `@site/` alias (the site's `src/`) instead of `../../../` paths. Links written as routes (`/control/pid/`) become relative links to the documents, so docs-kit checks them.
- **`docs/README.md` is the documentation map**, one row per sidebar group in today's order, and within a folder's row the links in today's order. "Roadmap" is a top-level document with its own row.
- **The nested groups come from front matter.** Each block's four pages carry `sidebar.group` (for example "PID Controller") and `sidebar.label` (Explainer, Developer Guide, V&V Plan, Deployment Guide). The group is listed where its first page would be.
- **The site code stays in `docs-site/`:** the components, `traces/`, `schemas/`, `scripts/trace/*`, the figures and their generators, and the WebAssembly build's output in `public/wasm/`.
- **`docs-site/docs-kit.config.mjs`** holds what docs-kit doesn't default: the title and description, the "Signal Processing" label for `dsp/`, the amber accent, `math: true` (KaTeX, as before), `src/styles/custom.css` (now only the rule that scrolls wide display math), and the API reference over `sdk/include` (it replaces the `Doxyfile`). The package pins `github:branes-ai/docs-kit#v0.2.1`; Astro, Starlight and KaTeX come with it.
- **The deploy keeps its own workflow**, `deploy-docs.yml`, because its build needs Emscripten for the WebAssembly module, and docs-kit's shared deploy can't add a toolchain. It runs the site's `npm run api` and `npm run build`, and now also triggers on `docs/**`.
- **CI:** the `Docs traces` job already builds the site; it now runs `npm run check` after the build, which checks every link and anchor in it.
- **The records**, `docs/plan/` and `docs/adr/`, are published on the [architecture site](https://branes-ai.github.io/architecture/reflex/), which lists reflex in its `sources.yaml`; this site redirects to them.

## Acceptance

1. `docs-site/src/content/` has no committed content; every page comes from `docs/`.
2. Every route on today's site still exists, with the same content and the same sidebar order: a comparison of each page's content in the built `dist/`, the sidebar, and a screenshot.
3. The live widgets and trace players work on the deployed site.
4. A broken relative link in `docs/` fails the PR.

## Departures

- The sidebar gains the "Documentation map" (`/overview/`) and a "Plans and decisions" group linking to the architecture site, as on the other sites.
- KaTeX's stylesheet is 0.16.47, the version that renders the math; it was 0.17.0, a mismatch.
- `standards/building-blocks.md` names the pages' new location, `docs/<layer>/`.
