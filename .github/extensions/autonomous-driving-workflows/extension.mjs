import { defineWorkflow, joinSession } from "@github/copilot-sdk/extension";
import { definitions, inputSchema, runWorkflow } from "./runtime.mjs";

await joinSession({
    workflows: Object.entries(definitions).map(([name, definition]) =>
        defineWorkflow({
            meta: {
                name,
                description: definition.description,
                argsSchema: inputSchema,
                phases: definition.stages.map(({ id }) => ({ title: id })),
            },
            run: (ctx) => runWorkflow(name, ctx),
        })
    ),
});
