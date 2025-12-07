---
id: 001
title: Specify Interactive RAG Chatbot and Auth
stage: spec
date: 2025-12-08
surface: agent
model: gemini
feature: interactive-rag-chatbot-auth
branch: 003-interactive-rag-chatbot-auth
user: n/a
command: /sp.specify
labels: [spec, rag, chatbot, auth, phase2]
links:
  spec: specs/003-interactive-rag-chatbot-auth/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/003-interactive-rag-chatbot-auth/spec.md
 - specs/003-interactive-rag-chatbot-auth/checklists/requirements.md
tests:
 - None
---

## Prompt

Physical AI & Humanoid Robotics Book — Interactive Systems Upgrade** Feature Branch: 003-interactive-rag-chatbot-auth Created: 2025-12-05 Status: Draft Phase: 2 (Additive — Book content remains frozen) Input: Extend existing Docusaurus book with interactive RAG system + optional auth layer. 1. User Scenarios & Testing (Mandatory) User Story 1 — Student Using Chatbot (Priority: P1) As a student, I want an AI chatbot inside the textbook so I can ask questions about robotics topics and get answers based ONLY on the textbook content. Independent Test Student selects text → hits “Ask AI” → chatbot answers only from that selection. Acceptance Scenarios Given the student highlights text in a chapter When they click “Ask AI” Then the chatbot responds using only embeddings for that selected text. Given the student asks a general book question When the chatbot searches Qdrant Then the answer is grounded in the correct chapter content. User Story 2 — Instructor (Priority: P2) As an instructor, I need a reliable RAG system so I can verify that answers match the book and do not hallucinate. Independent Test Instructor asks 10 questions → 10 correct, citation-backed answers. Acceptance Scenarios Instructor asks “Explain ROS2 nodes” → chatbot responds with citations. Instructor asks about Capstone → chatbot references the correct section. User Story 3 — Optional Bonus Auth User (Priority: P3) As a logged-in student, I want the chatbot to give personalized answers based on my hardware/software experience. Independent Test The chatbot modifies suggestions depending on user background. Acceptance Scenarios Beginner user → chatbot suggests simpler steps. Advanced user → chatbot gives optimization advice. 2. Functional Requirements RAG Chatbot Core (Mandatory) FR-001 Provide a fully functional RAG chatbot embedded in the Docusaurus site. FR-002 Use these official tools: OpenAI Agents SDK (Python) ChatKit JS/TS SDK FastAPI backend Neon Serverless Postgres for user metadata + chat history Qdrant Cloud Free Tier for embeddings storage FR-003 — Two Retrieval Modes Mode A: Normal RAG → full book embeddings Mode B: “Answer from selected text only” User highlights text Text passed to backend Backend generates embeddings live RAG answers from only that chunk FR-004 — Docusaurus UI Integration Chat widget rendered as React component Lives in /src/components/chatbot/ Static assets inside /static/ Button inside navbar + floating widget FR-005 — Grounded Answers Each answer must cite the chapter/section used Must reject out-of-scope queries Better-Auth Integration (Optional Bonus) FR-010 — Auth System Implement signup/signin using Better-Auth Guided by MCP Server context7 using: https://www.better-auth.com/docs/ FR-011 — Background Questionnaire At signup, required fields: Software experience Hardware experience Robotics familiarity AI/ML familiarity FR-012 — Personalized Responses Chatbot tailors answers e.g.: “Aap beginner ho → pehle ROS2 basics try kro.” “Aap advanced ho → ye optimization techniques use kro.” 3. Data Requirements DR-001 — Qdrant Collections book_chunks → Complete book embeddings selected_chunks → per-query temporary embeddings DR-002 — Neon DB Tables users user_background chat_history sessions DR-003 — Embedding Pipeline Chunk Markdown files Store metadata: chapter, section, file path 4. Non-Functional Requirements NFR-001 — Reliability System must handle 100‒300 queries/day without crashing. NFR-002 — Zero Hallucination Rule Chatbot must answer ONLY from RAG sources. If info not found → “I cannot find this in the book.” NFR-003 — Security Auth tokens must be JWT-based via Better-Auth. NFR-004 — UI UX Simple floating chat button Light/dark mode support Mobile responsive 5. Success Criteria SC-001 Chatbot answers correctly from book content 95% accuracy. SC-002 “Answer from selection only” mode works perfectly. SC-003 FastAPI backend deploys successfully on Render/Vercel/Cloud. SC-004 Optional auth flow stores user background correctly. SC-005 Docusaurus site builds with widget integrated, no build errors. 6. Out of Scope No image or video generation No cloud autoscaling setup No voice mode No modification to Phase 1 textbook content 7. Assumptions Phase 1 book is complete and frozen. Docusaurus build is already functional. You have access to OpenAI API keys, Neon, Qdrant, and Better-Auth. 8. Clarifications (Updated 2025-12-05) Q: Can the book be modified? A: No. Phase 1 is locked. Only additive features allowed. Q: Do we need a custom UI? A: Yes, a simple React widget integrated into Docusaurus. Q: How much personalization required? A: Only based on stored user background. Q: Can chatbot use internet? A: No. Only textbook embeddings

## Response snapshot

The user-provided specification has been successfully processed and saved. A new branch has been created, and the specification is ready for the planning phase.

## Outcome

- ✅ Impact: High - The specification for Phase 2 has been formally captured, setting the stage for planning and implementation.
- 🧪 Tests: None
- 📁 Files: 2 created
- 🔁 Next prompts: /sp.plan
- 🧠 Reflection: The user provided a complete specification, so the agent's role was to correctly process and file it, which was successful.

## Evaluation notes (flywheel)

- Failure modes observed: n/a
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): n/a
- Next experiment (smallest change to try): n/a
