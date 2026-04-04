---
description: "Start an orchestrated plan-execute-review workflow for a numerical-toolbox development task. Routes through planning (Claude Opus 4.6), implementation, and code review stages with handoff buttons between each step."
agent: "orchestrator"
argument-hint: "Describe the algorithm, feature, bug fix, or change you want to implement"
model: "Claude Sonnet 4.6"
---

Analyze the following task for the **numerical-toolbox** project — a numerical algorithms library for DSP, control algorithms, filters, optimizers, and estimators targeting resource-constrained embedded systems. Gather relevant context from the codebase — identify affected modules, existing patterns, numeric types involved (`float`, `math::Q15`, `math::Q31`), and documentation requirements. Then provide a brief scope summary and use the handoff buttons to route to the appropriate specialist:

- **Plan Implementation**: For complex tasks needing detailed upfront design (new algorithms, architectural changes, multi-file modifications)
- **Execute Directly**: For straightforward changes with a clear path (bug fixes, small improvements)
- **Review Code**: For reviewing existing or recently changed code against project standards

Task to orchestrate:
