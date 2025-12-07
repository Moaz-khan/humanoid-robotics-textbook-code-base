# Research & Design Decisions

This document records the key technology and pedagogical decisions for the "Physical AI & Humanoid Robotics" textbook, as clarified during the specification phase.

## Core Technology Stack

- **Content Format**: Markdown
  - **Rationale**: Markdown is lightweight, easy to write, and is the native format for Docusaurus.
- **Website Framework**: Docusaurus v2
  - **Rationale**: Docusaurus is a modern static site generator designed for documentation, providing features like versioning, search, and theming out-of-the-box. It's built with React, allowing for rich customizations.
- **Deployment Platform**: GitHub Pages
  - **Rationale**: GitHub Pages offers free, reliable hosting for static sites directly from a GitHub repository, which is ideal for this open-source project.
- **Development Environment**: Ubuntu 22.04
  - **Rationale**: This is the standard platform for ROS 2 development and ensures consistency for all labs.
- **Core Robotics Platforms**:
  - ROS 2 Humble/Iron
  - Gazebo
  - NVIDIA Isaac Sim
  - **Rationale**: These are the current industry and academic standards for robotics simulation and development, as required by the project's "Accuracy & Technical Authenticity" principle.

## Pedagogical (Teaching) Decisions

- **Decision**: Assume audience has proficiency in Python and the Linux command line.
  - **Rationale**: This allows the textbook to focus on robotics-specific content rather than re-teaching programming fundamentals, which is appropriate for a university-level course.
  - **Alternatives Considered**: Assuming no prior knowledge (would make the book too verbose and basic), assuming prior ROS knowledge (would exclude many target students).
- **Decision**: Labs will be structured as detailed, step-by-step guided tutorials.
  - **Rationale**: This format ensures students can successfully complete the exercises, building a strong and confident foundation before tackling more complex problems.
  - **Alternatives Considered**: Challenge-based labs (better for advanced students, but too difficult for foundational topics), open-ended projects (only suitable for capstone).
- **Decision**: The Capstone Project will use a hybrid guidance model.
  - **Rationale**: This model provides students with high-level goals, milestones, and architectural hints, encouraging them to apply what they've learned without being hand-held. It balances guidance with independent problem-solving.
  - **Alternatives Considered**: A fully guided capstone (less of a "capstone" experience), a fully open-ended project (too high a risk of failure/frustration).
- **Decision**: Each module will have a dedicated troubleshooting or FAQ section.
  - **Rationale**: This centralizes solutions to common problems, making it easier for students to self-service issues without cluttering the primary lab instructions.
- **Decision**: Optional/advanced content will be clearly marked within chapters or placed in appendices.
  - **Rationale**: This allows for the inclusion of enrichment material without disrupting the flow of the core curriculum, providing a clear path for all students while offering depth for those who want it.
