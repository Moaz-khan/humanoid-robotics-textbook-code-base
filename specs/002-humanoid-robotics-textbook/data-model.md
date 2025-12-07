# Data Model (Content Schema)

This document defines the schema for the content of the "Physical AI & Humanoid Robotics" textbook. As this is a documentation project, the "data model" refers to the structure and components of the Markdown files.

## Textbook Entity

The root entity is the **Textbook**, which is a collection of Chapters and global assets.

- **Attributes**:
  - `title`: String (e.g., "Physical AI & Humanoid Robotics")
  - `version`: String (e.g., "1.0.0")
  - `authors`: Array<String>
  - `docusaurus_config`: Object (Defines sidebar, navbar, footer, etc.)

## Chapter Entity

A **Chapter** is a self-contained Markdown file representing a major section of the book (usually a module).

- **Attributes**:
  - `id`: String (Unique identifier, e.g., `01-module-ros2`)
  - `title`: String (e.g., "Module 1: The Robotic Nervous System (ROS 2)")
  - `sidebar_position`: Integer
- **Content Sections (within the Markdown file)**:
  - **`Theory`**: Prose, images, and diagrams explaining core concepts.
  - **`Labs`**: A collection of Lab sections. Each lab is a guided, step-by-step tutorial.
  - **`Code Examples`**: Runnable code snippets with explanations.
  - **`Checkpoints`**: A short, multiple-choice quiz to test understanding.
  - **`Troubleshooting`**: A dedicated section for common errors and their solutions for that chapter's content.
  - **`Optional Content`**: A clearly marked section for advanced topics, marked with a Docusaurus admonition (e.g., `:::note Optional`).

## Lab Schema

- **`Objective`**: A short description of what the lab will accomplish.
- **`Prerequisites`**: A list of required software or previous labs.
- **`Steps`**: An ordered list of instructions.
  - Each step contains commands to be executed in a terminal, code to be written, and expected output (text or screenshots).
- **`Verification`**: A final step to confirm the lab was completed successfully.

## Code Example Schema

- Rendered using Docusaurus's live code blocks or standard Markdown code fences with syntax highlighting.
- Accompanied by a brief explanation of what the code does.

## Diagram Schema

- Initially, a textual description in a blockquote.
  - Example: `> Diagram: A diagram showing a ROS 2 node graph with a publisher, a topic, and a subscriber.`
- To be replaced by an image file linked in the Markdown.
  - Example: `![ROS 2 Node Graph](../../static/img/ros2-node-graph.png)`

## State Transitions

The "state" of the textbook transitions as follows:
1.  **Drafting**: Chapters are being written and code is being developed.
2.  **Validation**: Labs and code are being tested for reproducibility.
3.  **Review**: Content is being proofread and checked against the constitution.
4.  **Published**: The Docusaurus site is built and deployed to GitHub Pages.
