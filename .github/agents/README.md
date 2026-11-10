# Workflow roles

| Role | Responsibility | Tool boundary |
| --- | --- | --- |
| algorithm-researcher | Source research and experiment/design preflight | Read/search/web; no source edits or execution |
| algorithm-implementer | Approved minimal implementation | Read/search/edit/execute; unique writer |
| algorithm-validator | Impact-selected validation and artifacts | Read/search/execute; no source fixes |
| algorithm-reviewer | Independent design/diff/evidence review | Read/search; no execution or edits |
| experiment-reporter | Evidence consolidation | Read/search; no execution or edits |

The foreground coordinator owns task lifecycle and final acceptance. Roles do
not delegate recursively or prescribe a model; runtime user preferences apply.
Each role opts into CLI repository instructions with
`include-custom-instructions: true` and explicitly reads [AGENTS.md](../../AGENTS.md) and the
[workflow contract](../../.agents/workflows/README.md), even if custom instruction
inheritance is unavailable. Tool aliases follow the
[official configuration reference](https://docs.github.com/en/copilot/reference/custom-agents-configuration).
The CLI-specific inheritance field is documented in the
[CLI reference](https://docs.github.com/en/copilot/reference/copilot-cli-reference/cli-command-reference#custom-agent-frontmatter-fields).
Execute access is not a sandbox: validators must not use shell commands to edit
source, install dependencies, or publish.
