---
id: 003
title: Amend Constitution for Phase 2 Requirements
stage: constitution
date: 2025-12-08
surface: agent
model: gemini
feature: none
branch: main
user: n/a
command: /sp.constitution
labels: [constitution, project-planning, phase2]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - None
---

## Prompt

/sp.constitution We are now starting Phase 2 of the project. The textbook generated in Phase 1 is fully complete and FROZEN — no regeneration or modifications to existing chapters, images, or Docusaurus content.

I want you to update only the constitution.md file by adding a new section called ‘Phase 2: Interactive Systems Requirements’ at the end of the document.

This section must describe the new Phase-2 objectives:

1. Integrated RAG Chatbot Development

Build a Retrieval-Augmented Generation (RAG) chatbot embedded into the published Docusaurus website.

Use OpenAI Agents / ChatKit SDKs.

Use FastAPI backend.

Use Neon Serverless Postgres for user data + chat history.

Use Qdrant Cloud Free Tier for embeddings + retrieval.

Support:

answering from full book

answering from user-selected text only

Integrate UI widget inside Docusaurus /static + React component.

2. Optional Bonus: Better-Auth Signup/Signin

Use MCP server context7 to read & follow: https://www.better-auth.com/docs/

Implement signup + login flows.

During signup, ask user’s background (software, hardware, robotics, AI experience).

Store background in Neon Postgres.

Personalize chatbot answers using this background.

3. Phase 2 Constraints

Phase 1 book content must NOT be changed or regenerated.

Only additive work: backend, embeddings pipeline, RAG API, auth system, Docusaurus widget.

Must respect existing textbook architecture.

Add this as a new high-level governance requirement beneath the existing Governance section.
DON’T rewrite or alter the Phase 1 constitution content.
Only append the new section.continue please

## Response snapshot

The constitution has been updated to version 1.2.0, adding the "Phase 2: Interactive Systems Requirements" section with details on the RAG chatbot and optional authentication features.

## Outcome

- ✅ Impact: High - The constitution now reflects the project's new phase and its specific technical and architectural requirements.
- 🧪 Tests: None
- 📁 Files: 1 modified
- 🔁 Next prompts: n/a
- 🧠 Reflection: The constitution was successfully amended as requested, clearly defining the scope and constraints for the next phase of development.

## Evaluation notes (flywheel)

- Failure modes observed: n/a
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): n/a
- Next experiment (smallest change to try): n/a
