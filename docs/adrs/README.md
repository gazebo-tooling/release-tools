This directory captures the key architectural decisions made related to Gazebo
releasing, packaging, infra, testing and other devops related topics.

## What is an ADR?

An Architecture Decision Record is a short document that describes a significant
decision made about the software architecture of the project. Each ADR describes
the context, the decision itself, and the consequences — both positive and
negative.

## File Naming

ADRs are numbered sequentially:

```
NNNN-short-title.md
```

For example: `0001-the-rotary-rolling-release.md`

## Metadata Fields

Every ADR includes a YAML front-matter block with four fields:

| Field        | Values                                                       | Description                            |
|--------------|--------------------------------------------------------------|----------------------------------------|
| **status**   | `proposed` · `accepted` · `deprecated` · `superseded`        | Current lifecycle stage of the decision |
| **implementation** | `not-started` · `in-progress` · `done` `              | Progress of the *work* that follows the decision |
| **date**     | `YYYY-MM-DD`                                                 | Date the ADR was created or last updated |
| **category** | Free-form tag(s), e.g. `infrastructure`, `releasing`, `packaging`   | Domain the decision relates to          |
| **impact**   | `low` · `medium` · `high` · `critical`                       | How broadly the decision affects the project |

## Lifecycle

```
proposed  ──►  accepted  ──►  deprecated
                         ──►  superseded by NNNN
```

- **Proposed** — Under discussion, not yet binding.
- **Accepted** — The team has agreed; this is the current policy.
- **Deprecated** — No longer relevant (e.g. feature removed).
- **Superseded** — Replaced by a newer ADR (link to it in the body).

## How to Create a New ADR

1. Copy `0000-template.md` to a new file with the next available number.
2. Fill in the front-matter and all sections.
3. Submit as a pull request for Gazebo PMC to review.
4. Once ready, update the status to `accepted` and add an entry to the index below and merge.

## Index
 
| #    | Title                                  | Status    | Implementation | Date       | Category        | Impact   |
|------|----------------------------------------|-----------|----------------|------------|-----------------|----------|
| 0000 | Template for ADRs                | proposed  | not-started           | 2026-03-18 | documentation  | low |
| 0001 | The Rotary Rolling Release       | accepted  | in-progress           | 2026-02-14 | releasing, packaging | high |
