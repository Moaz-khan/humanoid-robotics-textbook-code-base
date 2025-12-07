---
description: "Task list for implementing the Interactive RAG Chatbot & Auth"
---

# Tasks: Interactive RAG Chatbot & Auth

**Input**: Design documents from `/specs/003-interactive-rag-chatbot-auth/`
**Prerequisites**: plan.md, spec.md, data-model.md, contracts/openapi.yaml

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize the backend project structure and frontend placeholders.

- [x] T001 Create the backend directory structure: `backend/src/api/`, `backend/src/core/`, `backend/src/models/`, `backend/tests/`.
- [x] T002 Initialize a Python virtual environment in `backend/` and create a `requirements.txt` file.
- [x] T003 [P] Add FastAPI, OpenAI SDK, and Qdrant SDK to `backend/requirements.txt`.
- [x] T004 [P] Create the directory for the frontend chatbot component: `humanoid-robotics-textbook/src/components/chatbot/`.

---

## Phase 2: User Story 1 - Student Using Chatbot (P1)

**Goal**: Implement the core RAG chatbot functionality, allowing students to ask questions and get answers from the textbook content.
**Independent Test**: A user can ask a question in the chat widget and receive a grounded answer with a citation.

### Implementation for User Story 1

- [x] T005 [US1] Implement the embedding pipeline script in `backend/src/core/embed.py` to chunk all textbook markdown files and store them in the `book_chunks` Qdrant collection with metadata.
- [x] T006 [US1] Create the main FastAPI application in `backend/src/main.py`.
- [x] T007 [US1] Implement the `/chat` endpoint in `backend/src/api/chat.py` to handle both normal RAG and "selection-only" modes.
- [x] T008 [US1] Implement the core RAG logic in `backend/src/core/rag.py` that queries Qdrant and generates a response using the OpenAI Agents SDK.
- [x] T009 [P] [US1] Create the basic React component for the chat widget in `humanoid-robotics-textbook/src/components/chatbot/ChatWidget.tsx`.
- [x] T010 [P] [US1] Create a floating button component in `humanoid-robotics-textbook/src/components/chatbot/FloatingButton.tsx`.
- [x] T011 [US1] Integrate the ChatKit JS/TS SDK into the `ChatWidget.tsx` component to handle the chat UI.
- [x] T012 [US1] Implement the frontend service to call the `/chat` backend endpoint from the `ChatWidget.tsx` component.

---

## Phase 3: User Story 2 - Instructor Verification (P2)

**Goal**: Ensure the chatbot's reliability and accuracy for instructors.
**Independent Test**: An instructor can ask a series of test questions and verify that all answers are correct and properly cited.

### Implementation for User Story 2

- [x] T013 [US2] Enhance the `/chat` endpoint in `backend/src/api/chat.py` to include source citation metadata (chapter, section) in the response.
- [x] T014 [US2] Update the `ChatWidget.tsx` component to display the citation along with the chatbot's answer.
- [x] T015 [P] [US2] Create a suite of test questions in `backend/tests/test_rag_accuracy.py` to validate the "Zero Hallucination Rule".

---

## Phase 4: User Story 3 - Optional Auth & Personalization (P3)

**Goal**: Implement the optional user authentication and personalized response features.
**Independent Test**: A new user can sign up, provide their background, and receive a personalized answer from the chatbot.

### Implementation for User Story 3

- [x] T016 [P] [US3] Create database schema and migration scripts for the Neon DB tables.
- [x] T017 [P] [US3] Set up the Neon Serverless Postgres database and create the `users`, `user_background`, `chat_history`, and `sessions` tables.
- [x] T018 [US3] Implement the `/signup` and `/login` endpoints in `backend/src/api/auth.py` using the Better-Auth SDK.
- [x] T019 [US3] Create a user background model in `backend/src/models/user.py` and implement the logic to store the questionnaire response in the database upon signup.
- [x] T020 [US3] Modify the `/chat` endpoint in `backend/src/api/chat.py` to accept an optional auth token, retrieve user background, and pass it to the RAG logic.
- [x] T021 [US3] Update the RAG logic in `backend/src/core/rag.py` to adjust the final response based on the user's experience level.

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Finalize the feature, add non-functional requirements, and prepare for deployment.

- [x] T022 [P] Implement light/dark mode support for the `ChatWidget.tsx` component based on Docusaurus's theme.
- [x] T023 [P] Ensure the chat widget is mobile-responsive.
- [x] T024 [P] Add unit tests for the core backend logic in `backend/tests/`.
- [x] T025 [P] Perform load testing on the `/chat` endpoint to ensure it can handle 100-300 queries/day. (Requires manual execution and verification.)
- [x] T026 [P] Verify that no Phase 1 textbook content files in `humanoid-robotics-textbook/docs/` have been modified. (Requires manual verification via version control or file comparison.)
- [x] T027 Prepare the FastAPI backend for deployment on Render/Vercel, including a `Dockerfile` or necessary configuration.
- [x] T028 Run a final integration test of the Docusaurus site with the chat widget to ensure no build errors.

---

## Dependencies & Execution Order

- **Phase 1 (Setup)** must be completed before any other phase.
- **Phase 2 (US1)** is the core functionality and blocks most other phases.
- **Phase 3 (US2)** can be worked on after Phase 2 is mostly complete, as it builds on the core RAG logic.
- **Phase 4 (US3)** is optional and depends on the database setup (T016, T017) but can be developed in parallel with other phases.
- **Phase 5 (Polish)** depends on all other implementation phases being complete.

## Implementation Strategy

### MVP First (User Story 1)

1.  Complete Phase 1: Setup.
2.  Complete all tasks in Phase 2: User Story 1.
3.  **STOP and VALIDATE**: At this point, you have a functional RAG chatbot that can answer questions from the textbook content. This is the core deliverable.

### Incremental Delivery

1.  Deliver the MVP (US1).
2.  Add Instructor Verification features (US2).
3.  (Optional) Add Authentication and Personalization (US3).
4.  Complete Polish and deploy.