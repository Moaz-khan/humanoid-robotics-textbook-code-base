# Feature Specification: Physical AI & Humanoid Robotics Book — Interactive Systems Upgrade

**Feature Branch**: `003-interactive-rag-chatbot-auth`
**Created**: 2025-12-08
**Status**: Draft
**Phase**: 2 (Additive — Book content remains frozen)
**Input**: Extend existing Docusaurus book with interactive RAG system + optional auth layer.

## 1. User Scenarios & Testing (Mandatory)

### User Story 1 — Student Using Chatbot (Priority: P1)
As a student, I want an AI chatbot inside the textbook so I can ask questions about robotics topics and get answers based ONLY on the textbook content.

**Independent Test**: Student selects text → hits “Ask AI” → chatbot answers only from that selection.

**Acceptance Scenarios**:
1.  **Given** the student highlights text in a chapter
2.  **When** they click “Ask AI”
3.  **Then** the chatbot responds using only embeddings for that selected text.
4.  **Given** the student asks a general book question
5.  **When** the chatbot searches Qdrant
6.  **Then** the answer is grounded in the correct chapter content.

### User Story 2 — Instructor (Priority: P2)
As an instructor, I need a reliable RAG system so I can verify that answers match the book and do not hallucinate.

**Independent Test**: Instructor asks 10 questions → 10 correct, citation-backed answers.

**Acceptance Scenarios**:
1.  Instructor asks “Explain ROS2 nodes” → chatbot responds with citations.
2.  Instructor asks about Capstone → chatbot references the correct section.

### User Story 3 — Optional Bonus Auth User (Priority: P3)
As a logged-in student, I want the chatbot to give personalized answers based on my hardware/software experience.

**Independent Test**: The chatbot modifies suggestions depending on user background.

**Acceptance Scenarios**:
1.  Beginner user → chatbot suggests simpler steps.
2.  Advanced user → chatbot gives optimization advice.

## 2. Functional Requirements

### RAG Chatbot Core (Mandatory)
-   **FR-001**: Provide a fully functional RAG chatbot embedded in the Docusaurus site.
-   **FR-002**: Use these official tools:
    -   OpenAI Agents SDK (Python)
    -   ChatKit JS/TS SDK
    -   FastAPI backend
    -   Neon Serverless Postgres for user metadata + chat history
    -   Qdrant Cloud Free Tier for embeddings storage
-   **FR-003 — Two Retrieval Modes**:
    -   **Mode A**: Normal RAG → full book embeddings
    -   **Mode B**: “Answer from selected text only”
        -   User highlights text
        -   Text passed to backend
        -   Backend generates embeddings live
        -   RAG answers from only that chunk
-   **FR-004 — Docusaurus UI Integration**:
    -   Chat widget rendered as React component
    -   Lives in `/src/components/chatbot/`
    -   Static assets inside `/static/`
    -   Button inside navbar + floating widget
-   **FR-005 — Grounded Answers**:
    -   Each answer must cite the chapter/section used
    -   Must reject out-of-scope queries

### Better-Auth Integration (Optional Bonus)
-   **FR-010 — Auth System**: Implement signup/signin using Better-Auth
    -   Guided by MCP Server context7 using: https://www.better-auth.com/docs/
-   **FR-011 — Background Questionnaire**: At signup, required fields:
    -   Software experience
    -   Hardware experience
    -   Robotics familiarity
    -   AI/ML familiarity
-   **FR-012 — Personalized Responses**: Chatbot tailors answers
    -   e.g.: “Aap beginner ho → pehle ROS2 basics try kro.”
    -   “Aap advanced ho → ye optimization techniques use kro.”

## 3. Data Requirements
-   **DR-001 — Qdrant Collections**:
    -   `book_chunks` → Complete book embeddings
    -   `selected_chunks` → per-query temporary embeddings
-   **DR-002 — Neon DB Tables**:
    -   `users`
    -   `user_background`
    -   `chat_history`
    -   `sessions`
-   **DR-003 — Embedding Pipeline**:
    -   Chunk Markdown files
    -   Store metadata: chapter, section, file path

## 4. Non-Functional Requirements
-   **NFR-001 — Reliability**: System must handle 100‒300 queries/day without crashing.
-   **NFR-002 — Zero Hallucination Rule**: Chatbot must answer ONLY from RAG sources. If info not found → “I cannot find this in the book.”
-   **NFR-003 — Security**: Auth tokens must be JWT-based via Better-Auth.
-   **NFR-004 — UI UX**: Simple floating chat button, Light/dark mode support, Mobile responsive

## 5. Success Criteria
-   **SC-001**: Chatbot answers correctly from book content 95% accuracy.
-   **SC-002**: “Answer from selection only” mode works perfectly.
-   **SC-003**: FastAPI backend deploys successfully on Render/Vercel/Cloud.
-   **SC-004**: Optional auth flow stores user background correctly.
-   **SC-005**: Docusaurus site builds with widget integrated, no build errors.

## 6. Out of Scope
-   No image or video generation
-   No cloud autoscaling setup
-   No voice mode
-   No modification to Phase 1 textbook content

## 7. Assumptions
-   Phase 1 book is complete and frozen.
-   Docusaurus build is already functional.
-   You have access to OpenAI API keys, Neon, Qdrant, and Better-Auth.

## 8. Clarifications (Updated 2025-12-05)
-   Q: Can the book be modified? A: No. Phase 1 is locked. Only additive features allowed.
-   Q: Do we need a custom UI? A: Yes, a simple React widget integrated into Docusaurus.
-   Q: How much personalization required? A: Only based on stored user background.
-   Q: Can chatbot use internet? A: No. Only textbook embeddings.