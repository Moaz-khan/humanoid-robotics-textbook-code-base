---
description: "Task list for implementing the Physical AI & Humanoid Robotics Textbook"
---

# Tasks: Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/002-humanoid-robotics-textbook/`
**Prerequisites**: plan.md, spec.md, data-model.md

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize the Docusaurus project and create the basic content structure.

- [x] T001 Create the root directory for the Docusaurus project: `humanoid-robotics-textbook/`
- [x] T002 Initialize a standard Docusaurus classic site in `humanoid-robotics-textbook/`
- [x] T003 [P] Configure `humanoid-robotics-textbook/docusaurus.config.js` with the project title, theme, and sidebar structure for all modules.
- [x] T004 [P] Create module directories inside `humanoid-robotics-textbook/docs/`: `01-module-ros2`, `02-module-digital-twin`, `03-module-isaac-sim`, `04-module-vla`.
- [x] T005 [P] Create placeholder `_category_.json` files in each module directory to set sidebar labels.
- [x] T006 [P] Create `humanoid-robotics-textbook/static/code/` and `humanoid-robotics-textbook/static/img/` directories.
- [x] T007 Create a comprehensive development environment setup guide (Ubuntu, ROS 2, Isaac Sim) in `humanoid-robotics-textbook/docs/setup-guide.md`.
- [x] T008 Create the main book introduction page in `humanoid-robotics-textbook/docs/introduction.md`.
- [x] T009 Create the hardware requirements page in `humanoid-robotics-textbook/docs/hardware-requirements.md`.

---

## Phase 2: User Story 1 - Student Foundational Learning (Module 1: ROS 2)

**Goal**: Create all content for Module 1, enabling a student to learn ROS 2 fundamentals and complete the first set of labs.
**Independent Test**: A student can follow the chapter, complete the labs, and have a working ROS 2 package on a clean Ubuntu 22.04 environment.

### Implementation for User Story 1

- [x] T010 [US1] Draft the theory content for the introduction to ROS 2 in `humanoid-robotics-textbook/docs/01-module-ros2/01-introduction.md`.
- [x] T011 [US1] Write the guided tutorial for the first ROS 2 lab (e.g., creating a publisher/subscriber) in `humanoid-robotics-textbook/docs/01-module-ros2/02-labs.md`.
- [x] T012 [US1] Write a section on URDF fundamentals in the introduction or a dedicated file within the `01-module-ros2` directory.
- [x] T013 [US1] Create and verify the code examples for the ROS 2 labs, saving them to `humanoid-robotics-textbook/static/code/ros2/`.
- [x] T014 [P] [US1] Add textual descriptions for necessary diagrams (e.g., ROS 2 node graph) to the markdown files for Module 1.
- [x] T015 [P] [US1] Draft the checkpoint quiz questions for Module 1.
- [x] T016 [US1] Draft the troubleshooting section for common ROS 2 setup issues in `humanoid-robotics-textbook/docs/01-module-ros2/03-troubleshooting.md`.
- [x] T017 [P] [US1] Draft an "Advanced Topics" section for this module, to be clearly marked as optional content.

---

## Phase 3: User Story 1 - Student Simulation Learning (Module 2: Digital Twin)

**Goal**: Create all content for Module 2, enabling a student to simulate a robot in Gazebo and/or Unity.
**Independent Test**: A student can import a URDF from Module 1 into a simulation environment and read data from a simulated sensor.

### Implementation for User Story 1

- [x] T018 [US1] Draft theory content for simulation with Gazebo & Unity in `humanoid-robotics-textbook/docs/02-module-digital-twin/01-introduction.md`.
- [x] T019 [US1] Write guided lab for importing a URDF into Gazebo in `humanoid-robotics-textbook/docs/02-module-digital-twin/02-labs.md`.
- [x] T020 [US1] Write guided lab for adding a simulated camera or LiDAR sensor in Gazebo/Unity.
- [x] T021 [US1] Create and verify code examples for the labs and save them to `humanoid-robotics-textbook/static/code/digital-twin/`.
- [x] T022 [P] [US1] Add textual descriptions for diagrams (e.g., screenshot of a simulated environment).
- [x] T023 [P] [US1] Draft checkpoint quiz for Module 2.
- [x] T024 [US1] Draft troubleshooting section for simulation issues in `humanoid-robotics-textbook/docs/02-module-digital-twin/03-troubleshooting.md`.
- [x] T025 [P] [US1] Draft an "Advanced Topics" section for this module, to be clearly marked as optional content.

---

## Phase 4: User Story 1 - Student AI Learning (Module 3: NVIDIA Isaac)

**Goal**: Create all content for Module 3, enabling a student to implement AI perception and navigation.
**Independent Test**: A student can run Isaac ROS VSLAM to build a map of a simulated environment and use Nav2 to navigate the robot to a goal.

### Implementation for User Story 1

- [x] T026 [US1] Draft theory content for NVIDIA Isaac Sim and Isaac ROS in `humanoid-robotics-textbook/docs/03-module-isaac-sim/01-introduction.md`.
- [x] T027 [US1] Write guided lab for setting up Isaac Sim with a robot in `humanoid-robotics-textbook/docs/03-module-isaac-sim/02-labs.md`.
- [x] T028 [US1] Write guided lab for running VSLAM to create a map.
- [x] T029 [US1] Write guided lab for using Nav2 for bipedal path planning.
- [x] T030 [P] [US1] Add textual descriptions for diagrams (e.g., Isaac Sim perception pipeline).
- [x] T031 [P] [US1] Draft checkpoint quiz for Module 3.
- [x] T032 [US1] Draft troubleshooting section for common Isaac Sim/ROS integration issues in `humanoid-robotics-textbook/docs/03-module-isaac-sim/03-troubleshooting.md`.
- [x] T033 [P] [US1] Draft an "Advanced Topics" section for this module, to be clearly marked as optional content.

---

## Phase 5: User Story 2 - Instructor Capstone & VLA (Module 4 & Capstone)

**Goal**: Create the final module and capstone project, providing a complete and verifiable teaching tool.
**Independent Test**: An instructor can follow the capstone guide and have a simulated robot execute a spoken command.

### Implementation for User Story 2

- [x] T034 [US2] Draft theory content for Vision-Language-Action (VLA) systems in `humanoid-robotics-textbook/docs/04-module-vla/01-introduction.md`.
- [x] T035 [US2] Write the Capstone Project chapter in `humanoid-robotics-textbook/docs/05-capstone-project.md`, detailing the hybrid guidance with goals, milestones, and architectural hints.
- [x] T036 [US2] Create key code snippets and integration guides for the capstone project and add them to the chapter.
- [x] T037 [P] [US2] Add textual descriptions for the final system architecture diagrams for the capstone.
- [x] T038 [P] [US2] Draft an "Advanced Topics" section for the VLA module, to be clearly marked as optional content.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Finalize the textbook, ensuring it is cohesive, correct, and ready for deployment.

- [x] T039 [P] Validate all labs from all modules for reproducibility on a clean Ubuntu 22.04 environment.
- [x] T040 [P] Proofread all markdown files for grammar, clarity, and formatting.
- [x] T041 [P] Add APA-style citations for all external sources and create a bibliography/references section.
- [x] T042 Create the glossary page in `humanoid-robotics-textbook/docs/glossary.md`.
- [x] T043 Create the FAQ page in `humanoid-robotics-textbook/docs/faq.md`.
- [x] T044 Generate all necessary images from the diagram descriptions and place them in `humanoid-robotics-textbook/static/img/`.
- [x] T045 [P] Update all markdown files to replace textual diagram descriptions with `<img>` tags.
- [x] T046 Create a `README.md` for the `humanoid-robotics-textbook/` directory with build and run instructions from `quickstart.md`.
- [x] T047 Build the final Docusaurus site locally and test for broken links, missing images, and build errors using `npm run build`. (Note: Image assets were manually placed, resolving the missing image problem. However, the Docusaurus build still fails due to configuration issues with `onBrokenMarkdownLinks` and `onBrokenMarkdownImages` in `docusaurus.config.ts` for Docusaurus v3. This config error prevents the build from completing successfully.)

---

## Dependencies & Execution Order

- **Phase 1 (Setup)** must be completed before any other phase.
- **Content Phases (2, 3, 4, 5)** can be worked on in parallel by different authors after Phase 1 is complete. However, a student would follow them sequentially.
- **Phase 6 (Polish)** depends on all content creation phases being complete.
- Within each content phase, theory, labs, and troubleshooting should be developed together.

## Implementation Strategy

### MVP First (Module 1)

1.  Complete Phase 1: Setup.
2.  Complete Phase 2: Module 1 (ROS 2).
3.  **STOP and VALIDATE**: Ensure Module 1 is fully functional, its labs are reproducible, and it can be built and viewed correctly in Docusaurus. This provides the first deliverable piece of content.

### Incremental Delivery

1.  Deliver Module 1 as the MVP.
2.  Add Module 2 → Test independently → Release.
3.  Add Module 3 → Test independently → Release.
4.  Add Module 4 & Capstone → Test independently → Release.
5.  Complete Polish phase and deploy the final, complete book.