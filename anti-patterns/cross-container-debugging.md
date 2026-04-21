# Cross-container debugging

Do **not** probe for, switch to, or rely on other users' dev containers while
working on this repository.

## Why this is an anti-pattern

Container discovery can find multiple Apollo dev environments on the same host,
but those environments may belong to different users, have unrelated runtime
state, or carry different background services. Debug conclusions become noisy if
an investigation silently jumps from the current session environment into
somebody else's container.

For this workspace, the safe rule is:

1. stay inside the current session environment,
2. use the current user's container when container execution is required, and
3. treat other discovered containers as out of scope unless the user explicitly
   asks to inspect them.

## What to do instead

1. First inspect the current environment and the container already in use for
   the task.
2. If runtime state looks wrong, debug or clean up that same environment.
3. If the current environment is insufficient, ask before expanding the scope to
   another container or user context.

