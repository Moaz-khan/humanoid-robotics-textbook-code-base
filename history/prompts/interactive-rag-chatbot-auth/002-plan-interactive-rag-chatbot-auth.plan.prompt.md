---
id: 002
title: Plan Interactive RAG Chatbot and Auth
stage: plan
date: 2025-12-08
surface: agent
model: gemini
feature: interactive-rag-chatbot-auth
branch: 003-interactive-rag-chatbot-auth
user: n/a
command: /sp.plan
labels: [plan, architecture, rag, chatbot, auth]
links:
  spec: specs/003-interactive-rag-chatbot-auth/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/003-interactive-rag-chatbot-auth/plan.md
 - specs/003-interactive-rag-chatbot-auth/research.md
 - specs/003-interactive-rag-chatbot-auth/data-model.md
 - specs/003-interactive-rag-chatbot-auth/quickstart.md
 - specs/003-interactive-rag-chatbot-auth/contracts/openapi.yaml
tests:
 - None
---

## Prompt

/sp.plan # Phase 2 — Interactive RAG Chatbot Plan

## Overview
Phase 2 adds an interactive RAG chatbot to the Physical AI & Humanoid Robotics textbook on Docusaurus. The chatbot must answer questions based strictly on textbook content and optionally provide personalized answers for logged-in users. This plan outlines tasks for backend, frontend, data, testing, and deployment.

## Milestones
- **M1:** FastAPI backend setup complete  
- **M2:** Qdrant embedding collections ready (book_chunks & selected_chunks)  
- **M3:** Docusaurus React chatbot widget integrated  
- **M4:** Optional Better-Auth signup/login & personalization implemented  
- **M5:** Full testing & QA passed  
- **M6:** Deployment to Render/Vercel/Cloud

---

## Tasks

### T1 — Backend Setup (RAG Core)
- **Description:** Setup FastAPI backend to handle RAG queries, support two modes: normal RAG and selection-only mode.  
- **Priority:** P1  
- **Dependencies:** None  
- **Tools:** Python, FastAPI, OpenAI Agents SDK, Qdrant SDK  
- **Notes:** Must reject out-of-scope queries, return chapter/section citations, handle 100-300 queries/day reliably.  

### T2 — Qdrant Embedding Collections
- **Description:** Prepare Qdrant collections for full book embeddings (`book_chunks`) and per-query temporary embeddings (`selected_chunks`).  
- **Priority:** P1  
- **Dependencies:** T1  
- **Tools:** Qdrant Cloud, Python  
- **Notes:** Store metadata: chapter, section, file path. Ensure “answer from selection only” works.  

### T3 — Embedding Pipeline
- **Description:** Chunk Markdown files, generate embeddings for all chapters. Store metadata for citation reference.  
- **Priority:** P1  
- **Dependencies:** T2  
- **Tools:** Python, OpenAI embeddings  
- **Notes:** Chunk size optimized for semantic retrieval.  

### T4 — Neon DB Setup
- **Description:** Create Postgres tables: `users`, `user_background`, `chat_history`, `sessions`.  
- **Priority:** P2  
- **Dependencies:** None  
- **Tools:** Neon Serverless Postgres  
- **Notes:** Support optional auth flow and personalized answers.  

### T5 — Docusaurus Chatbot Widget (Frontend)
- **Description:** Create React component for chat widget, integrate floating button and navbar entry.  
- **Priority:** P1  
- **Dependencies:** T1, T2  
- **Tools:** React, TypeScript, ChatKit JS/TS SDK  
- **Notes:** Support light/dark mode, mobile responsive, simple UX.  

### T6 — “Answer From Selection Only” Feature
- **Description:** Enable users to highlight text and get answers based only on that selection.  
- **Priority:** P1  
- **Dependencies:** T1, T2, T5  
- **Tools:** Frontend JS, FastAPI backend  
- **Notes:** Backend must generate temporary embeddings for highlighted text and query Qdrant.  

### T7 — Grounded Answers & Zero Hallucination QA
- **Description:** Ensure every chatbot answer cites correct chapter/section and rejects out-of-scope queries.  
- **Priority:** P1  
- **Dependencies:** T1, T2, T3, T5, T6  
- **Tools:** Python, Qdrant, OpenAI Agents SDK  
- **Notes:** Implement test suite for 95%+ accuracy.  

---

## Optional Better-Auth & Personalization Tasks

### T8 — Auth System Integration
- **Description:** Implement signup/login using Better-Auth, JWT-based tokens.  
- **Priority:** P3  
- **Dependencies:** T4  
- **Tools:** Better-Auth SDK, FastAPI  
- **Notes:** Include signup questionnaire for background info (software/hardware/robotics/AI familiarity).  

### T9 — Personalized Responses
- **Description:** Tailor chatbot answers based on user background.  
- **Priority:** P3  
- **Dependencies:** T8, T1, T5  
- **Tools:** Python, FastAPI, OpenAI Agents SDK  
- **Notes:** Beginner → simpler steps, Advanced → optimization suggestions.  

---

## Testing & QA

### T10 — Student Scenario Testing
- Highlight text → Ask AI → Check answer is grounded in selection.  
- Ask general book questions → Verify citations & accuracy.  

### T11 — Instructor Scenario Testing
- 10 sample questions → verify 10 citation-backed answers.  
- Test edge cases → queries not in book → return proper “I cannot find this in the book” message.  

### T12 — Auth & Personalization QA
- Signup flow works, background stored correctly.  
- Personalized answers match beginner/advanced settings.  

### T13 — Non-Functional Testing
- Ensure 100-300 queries/day reliability.  
- Test floating widget responsiveness across devices.  
- Light/Dark mode verification.  

---

## Deployment Tasks

### T14 — Backend Deployment
- Deploy FastAPI backend on Render/Vercel/Cloud.  

### T15 — Docusaurus Build & Integration
- Integrate widget in Docusaurus site.  
- Run full site build → ensure no errors.  

---

## Notes
- Phase 1 book content is frozen.  
- Do not use internet for answers — only textbook embeddings.  
- Tasks T8 & T9 are optional bonus, but recommended for enhanced user experience.  
- Keep modular structure so additional features (like multi-agent RAG) can be added later.

## Response snapshot

The planning phase for the interactive RAG chatbot is complete. The implementation plan, research decisions, data model, and API contracts have been created.

## Outcome

- ✅ Impact: High - Provides a clear architectural and implementation roadmap for the Phase 2 feature.
- 🧪 Tests: None
- 📁 Files: 5 created
- 🔁 Next prompts: /sp.adr, /sp.tasks
- 🧠 Reflection: The detailed user prompt for the plan allowed for a comprehensive set of design artifacts to be generated, setting a clear path for the task generation phase.

## Evaluation notes (flywheel)

- Failure modes observed: n/a
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): n/a
- Next experiment (smallest change to try): n/a