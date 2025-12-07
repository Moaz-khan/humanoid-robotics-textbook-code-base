<!--
---
Sync Impact Report
---
Version change: 1.0.0 → 1.1.0
Modified principles:
- Accuracy → Accuracy & Technical Authenticity
- Clarity → Clarity for Students (Beginner → Advanced)
- Reproducibility → Reproducibility
- Rigor → Educational Rigor
- Ethical AI → Consistency with Course Structure
Added sections:
- Writing Standards
- Citations & References
- Structure Standards
- Content Quality Gates
Removed sections:
- The previous 'Key Standards' section has been replaced with more specific standards.
Templates requiring updates:
- ✅ .specify/templates/plan-template.md
Follow-up TODOs:
- None
-->
# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### Accuracy & Technical Authenticity
All explanations must align with real robotics standards: ROS 2, Gazebo, Unity, NVIDIA Isaac, VSLAM, sensors, locomotion, and VLA models. No hallucinated APIs, commands, or robot capabilities.

### Clarity for Students (Beginner → Advanced)
Writing should be simple, structured, and progressive. Topics must flow from foundations → tools → simulation → AI → humanoid robotics → capstone.

### Educational Rigor
All modules must include: Concepts, Diagrams (described in text for now), Real-world examples, Labs / hands-on tasks, and Checkpoints / quizzes. Every learning outcome from the course must be addressed in the book.

### Reproducibility
Every tool setup (ROS 2, Gazebo, Isaac Sim, Jetson) must include complete installation steps that work on Ubuntu 22.04. All code samples must be runnable and tested. All simulation instructions must be verifiable.

### Consistency with Course Structure
Book must reflect the exact modules from the course document: Physical AI concepts, ROS 2, Gazebo & Unity, NVIDIA Isaac, Humanoid robotics, Vision-Language-Action systems, and the Capstone project.

## Key Standards

### Writing Standards
- Clear, technical, student-friendly tone
- No unnecessary jargon
- Step-by-step flows for hands-on labs

### Citations & References
Any factual robotics claim must cite official sources: Open Robotics (ROS 2 docs), NVIDIA Isaac docs, Gazebo/Unity docs, and Sensor manufacturer docs. Citations may be inline or at chapter end.

### Structure Standards
The book must include: An introduction section, One chapter per module, a Hardware requirement chapter, a Capstone walkthrough, a Glossary for robotics terms, and an FAQ.

### Content Quality Gates
- No outdated versions (ROS 2 Humble / Isaac Sim latest)
- No copying from vendor docs; rewrite everything
- All diagrams must be correct
- All instructions must be verified

## Constraints
- Book built with Docusaurus
- Generated via Spec-Kit Plus
- Deployment target: GitHub Pages
- All assets must be open-source compatible
- Chapters must be generated in Markdown only
- Audience: university-level robotics & AI students
- Book size: no max limit, but each module must cover theory + labs

## Success Criteria
The project is successful when:
- A complete Docusaurus book exists with all modules converted into chapters.
- All chapters meet quality gates of accuracy, clarity, reproducibility.
- Book builds & deploys successfully to GitHub Pages without errors.
- Spec-Kit Plus prompts generate consistent, structured content.
- All learning outcomes in the course document are fully covered.
- The book can stand alone as a full Physical AI course textbook.

## Phase 2: Interactive Systems Requirements

### 1. Integrated RAG Chatbot Development
- Build a Retrieval-Augmented Generation (RAG) chatbot embedded into the published Docusaurus website.
- The implementation MUST use OpenAI Agents / ChatKit SDKs, with a FastAPI backend.
- User data and chat history MUST be stored in Neon Serverless Postgres.
- Embeddings and retrieval MUST use the Qdrant Cloud Free Tier.
- The chatbot MUST support answering questions from the full book content and from user-selected text only.
- The UI widget MUST be integrated inside Docusaurus as a custom React component.

### 2. Optional Bonus: Better-Auth Signup/Signin
- This optional feature, if implemented, MUST follow the documentation from `https://www.better-auth.com/docs/` (to be read via MCP server context7).
- Implement user signup and login flows.
- During signup, the system MUST ask for the user’s background (e.g., software, hardware, robotics, AI experience).
- User background information MUST be stored in the Neon Serverless Postgres database.
- The RAG chatbot SHOULD personalize its answers based on the user's background.

### 3. Phase 2 Constraints
- Phase 1 book content is FROZEN and MUST NOT be changed or regenerated.
- All Phase 2 work MUST be additive (e.g., backend services, embeddings pipelines, RAG API, authentication system, Docusaurus widget).
- The new systems MUST respect the existing textbook architecture.

## Governance
This constitution outlines the fundamental principles and standards for the project. All contributions must adhere to these guidelines. Amendments to this constitution require team consensus and must be documented.

**Version**: 1.2.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-08