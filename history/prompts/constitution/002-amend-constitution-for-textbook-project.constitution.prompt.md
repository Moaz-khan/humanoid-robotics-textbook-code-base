---
id: 002
title: Amend Constitution for Textbook Project
stage: constitution
date: 2025-12-05
surface: agent
model: gemini
feature: none
branch: 001-humanoid-robotics-textbook
user: n/a
command: /sp.constitution
labels: [constitution, project-setup, amendment]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - .specify/templates/plan-template.md
tests:
 - None
---

## Prompt

/sp.constitution Project:
Create a complete Physical AI & Humanoid Robotics textbook using Docusaurus, generated through Spec-Kit Plus + Gemini, and deploy it on GitHub Pages.

Core Principles
1. Accuracy & Technical Authenticity

All explanations must align with real robotics standards: ROS 2, Gazebo, Unity, NVIDIA Isaac, VSLAM, sensors, locomotion, and VLA models.

No hallucinated APIs, commands, or robot capabilities.

2. Clarity for Students (Beginner → Advanced)

Writing should be simple, structured, and progressive.

Topics must flow from foundations → tools → simulation → AI → humanoid robotics → capstone.

3. Educational Rigor

All modules must include:

Concepts

Diagrams (described in text for now)

Real-world examples

Labs / hands-on tasks

Checkpoints / quizzes

Every learning outcome from the course must be addressed in the book.

4. Reproducibility

Every tool setup (ROS 2, Gazebo, Isaac Sim, Jetson) must include complete installation steps that work on Ubuntu 22.04.

All code samples must be runnable and tested.

All simulation instructions must be verifiable.

5. Consistency with Course Structure

Book must reflect the exact modules from the course document:

Physical AI concepts

ROS 2

Gazebo & Unity

NVIDIA Isaac

Humanoid robotics

Vision-Language-Action systems

Capstone project

Key Standards
Writing Standards

Clear, technical, student-friendly tone

No unnecessary jargon

Step-by-step flows for hands-on labs

Citations & References

Any factual robotics claim must cite official sources:

Open Robotics (ROS 2 docs)

NVIDIA Isaac docs

Gazebo/Unity docs

Sensor manufacturer docs

Citations may be inline or at chapter end.

Structure Standards

The book must include:

Introduction section

One chapter per module

Hardware requirement chapter

Capstone walkthrough

Glossary for robotics terms

FAQ

Content Quality Gates

No outdated versions (ROS 2 Humble / Isaac Sim latest)

No copying from vendor docs; rewrite everything

All diagrams must be correct

All instructions must be verified

Constraints

Book built with Docusaurus

Generated via Spec-Kit Plus

Deployment target: GitHub Pages

All assets must be open-source compatible

Chapters must be generated in Markdown only

Audience: university-level robotics & AI students

Book size: no max limit, but each module must cover theory + labs

Success Criteria

The project is successful when:

A complete Docusaurus book exists with all modules converted into chapters.

All chapters meet quality gates of accuracy, clarity, reproducibility.

Book builds & deploys successfully to GitHub Pages without errors.

Spec-Kit Plus prompts generate consistent, structured content.

All learning outcomes in the course document are fully covered.

The book can stand alone as a full Physical AI course textbook.

## Response snapshot

Updated the project constitution to version 1.1.0, refining the core principles and standards to be more specific to the textbook's creation. The plan template was also updated to reflect these changes.

## Outcome

- ✅ Impact: High - The constitution is now more detailed and aligned with the project's specific goals.
- 🧪 Tests: None
- 📁 Files: 2 modified
- 🔁 Next prompts: n/a
- 🧠 Reflection: This amendment makes the constitution a much more effective guide for developing the textbook content.

## Evaluation notes (flywheel)

- Failure modes observed: n/a
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): n/a
- Next experiment (smallest change to try): n/a
