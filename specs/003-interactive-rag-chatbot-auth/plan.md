# Implementation Plan: Interactive RAG Chatbot & Auth

**Branch**: `003-interactive-rag-chatbot-auth` | **Date**: 2025-12-08 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/003-interactive-rag-chatbot-auth/spec.md`

## Summary

This plan outlines the architecture and implementation for adding a Phase 2 interactive RAG chatbot and an optional authentication layer to the existing Docusaurus-based "Physical AI & Humanoid Robotics" textbook. The system will leverage a FastAPI backend, OpenAI and ChatKit SDKs, Neon and Qdrant databases, and a custom React widget. All work will be additive, ensuring the Phase 1 textbook content remains frozen as per the constitution. All work will be additive, ensuring the Phase 1 textbook content remains frozen as per the constitution. All work will be additive, with the Phase 1 textbook content remaining frozen.

## Technical Context

**Language/Version**: Python 3.10+, TypeScript (for React)
**Primary Dependencies**: FastAPI, OpenAI Agents SDK, ChatKit JS/TS SDK, Neon Serverless Postgres, Qdrant Cloud, Docusaurus, React
**Storage**: Neon Serverless Postgres (user data, chat history), Qdrant Cloud (embeddings)
**Testing**: Pytest (for backend), Jest/React Testing Library (for frontend), manual QA for all user stories.
**Target Platform**: FastAPI backend deployed on Render/Vercel, Docusaurus site on GitHub Pages.
**Project Type**: Web application (backend + frontend).
**Performance Goals**: System must handle 100-300 queries/day without crashing.
**Constraints**: Phase 1 content is frozen. The system must not access the internet for answers and must reject out-of-scope queries.
**Scale/Scope**: RAG chatbot with two retrieval modes and an optional authentication/personalization layer.

## Constitution Check

*GATE: All gates pass. The plan aligns with the established constitution for Phase 2.*

- [x] **Accuracy & Technical Authenticity**: N/A (Phase 2 is additive; Phase 1 content is frozen).
- [x] **Clarity for Students**: The chatbot feature is designed to enhance student learning.
- [x] **Educational Rigor**: N/A (Phase 2 is a feature, not new educational content).
- [x] **Reproducibility**: The plan includes clear deployment steps for the backend and frontend.
- [x] **Consistency with Course Structure**: The RAG system is designed to work with the existing course structure.
- [x] **Standards Compliance**: The plan uses the specified tools and follows the defined structure.
- [x] **Constraints Adherence**: The plan respects all project constraints, including the frozen state of Phase 1 content.

## Project Structure

### Documentation (this feature)

```text
specs/003-interactive-rag-chatbot-auth/
├── plan.md              # This file
├── research.md          # Technology and integration decisions
├── data-model.md        # DB schema and Qdrant collection structure
├── contracts/           # OpenAPI schema for FastAPI backend
├── quickstart.md        # Local setup for backend and frontend
└── tasks.md             # Detailed implementation tasks
```

### Source Code (repository root)

The project will be structured as a web application with a separate backend and frontend components.

```text
backend/
├── src/
│   ├── api/             # FastAPI endpoints
│   ├── core/            # Core logic for RAG, embeddings, DB access
│   └── models/          # Pydantic models for request/response
└── tests/

humanoid-robotics-textbook/
├── src/
│   ├── components/
│   │   └── chatbot/     # React component for the chat widget
│   └── services/        # Services for interacting with the backend
└── tests/
```

**Structure Decision**: A separate `backend/` directory will be created for the FastAPI application to cleanly separate it from the existing `humanoid-robotics-textbook/` Docusaurus project. The frontend chat widget will be developed as a new React component within the Docusaurus project's `src/components/` directory.

## Complexity Tracking

No violations of the constitution were detected.