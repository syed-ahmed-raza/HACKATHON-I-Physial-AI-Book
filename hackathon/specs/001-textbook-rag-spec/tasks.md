# Tasks: Physical AI & Humanoid Robotics Textbook Master Specification

**Input**: Design documents from `/specs/001-textbook-rag-spec/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story?] Description with file path`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/`

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create Docusaurus project in `frontend/`
- [ ] T002 Create FastAPI project in `backend/`
- [ ] T003 [P] Configure Docusaurus (e.g., `frontend/docusaurus.config.js`)
- [ ] T004 [P] Configure FastAPI environment and dependencies (e.g., `backend/requirements.txt`)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T005 Setup Neon Serverless Postgres for metadata storage (configuration and connection string in `backend/`)
- [x] T006 Setup Qdrant Cloud (Free Tier) and implement ingestion script in `backend/ingest.py`
- [c] T007 Implement basic database connection and ORM setup for Neon Postgres in `backend/` (e.g., `backend/db/session.py`, `backend/db/base.py`) -- CANCELLED: User redirected focus to frontend content.
- [c] T008 Implement basic vector database client for Qdrant in `backend/` (e.g., `backend/vector_db/client.py`) -- CANCELLED: User redirected focus to frontend content.
- [c] T009 Create base models/entities for textbook content metadata in FastAPI (`backend/app/models/content.py`) -- CANCELLED: User redirected focus to frontend content.

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Comprehensive Textbook Access (Priority: P1) 🎯 MVP

**Goal**: Deliver the static textbook content via Docusaurus, including initial module content.

**Independent Test**: Navigate Docusaurus site and verify content availability, structure, and content details.

### Content Generation Tasks (Module 1 - Docusaurus Integration)

- [c] T010 [US1] Create directory `../physical-ai-book/docs/module-1/` -- CANCELLED: Files now directly created in Docusaurus docs.
- [c] T011 [US1] Generate `../physical-ai-book/docs/module-1/01-foundations-of-physical-ai.md` with detailed explanation of embodied intelligence, Sense-Think-Act cycle, and difference between physical laws and digital data. Author: Syed Ahmed Raza, Sidebar Position: 1. -- CANCELLED: Files now directly created in Docusaurus docs.
- [c] T012 [US1] Generate `../physical-ai-book/docs/module-1/02-sensor-systems.md` with technical deep dive into LiDAR, IMU, Force/Torque sensors, and their role in humanoid balance. Author: Syed Ahmed Raza, Sidebar Position: 2. -- CANCELLED: Files now directly created in Docusaurus docs.
- [c] T013 [US1] Generate `../physical-ai-book/docs/module-1/03-ros2-architecture.md` with professional guide on Nodes, Topics, Services, Actions, including a Mermaid diagram or text representation of a ROS 2 graph. Author: Syed Ahmed Raza, Sidebar Position: 3. -- CANCELLED: Files now directly created in Docusaurus docs.
- [c] T014 [US1] Generate `../physical-ai-book/docs/module-1/04-building-packages-python.md` with a real Python code snippet using 'rclpy' to create a basic node and explanation of Launch files. Author: Syed Ahmed Raza, Sidebar Position: 4. -- CANCELLED: Files now directly created in Docusaurus docs.

- [ ] T010 [US1] Delete all existing files/folders in `frontend/docs/`
- [ ] T011 [US1] Delete all existing files/folders in `frontend/blog/`
- [ ] T012 [US1] Create directory `frontend/docs/module-1-ros2/`
- [ ] T013 [US1] Generate `frontend/docs/module-1-ros2/01-foundations-of-physical-ai.md` with detailed explanation of Embodied Intelligence, the Sense-Think-Act cycle, and why Physical AI needs to understand physics laws. Author: Syed Ahmed Raza, Sidebar position: 1.
- [ ] T014 [US1] Generate `frontend/docs/module-1-ros2/02-sensor-systems.md` with technical deep dive into LiDAR (Light Detection and Ranging), IMUs (6-axis/9-axis), and Force/Torque sensors for humanoid balance. Author: Syed Ahmed Raza, Sidebar position: 2.
- [ ] T015 [US1] Generate `frontend/docs/module-1-ros2/03-ros2-architecture.md` with full guide on Nodes, Topics, Services, and Actions. Explain the DDS (Data Distribution Service) layer. Author: Syed Ahmed Raza, Sidebar position: 3.
- [ ] T016 [US1] Generate `frontend/docs/module-1-ros2/04-building-packages-python.md` with a complete tutorial on creating a ROS 2 package, including a working 'rclpy' Python node example and a Launch file explanation. Author: Syed Ahmed Raza, Sidebar position: 4.

### Content Generation Tasks (Module 2 - Digital Twin)

- [x] T017 [US1] Create directory `frontend/docs/module-2-simulation/`
- [x] T018 [US1] Generate `frontend/docs/module-2-simulation/01-gazebo-simulation-setup.md` with detailed explanation of Gazebo setup, world files (.world), and physics engines (ODE/Bullet). Author: Syed Ahmed Raza, Sidebar position: 5.
- [x] T019 [US1] Generate `frontend/docs/module-2-simulation/02-robot-description-formats.md` with detailed explanation of URDF vs SDF, including a simplified XML code block for a humanoid leg link. Author: Syed Ahmed Raza, Sidebar position: 6.
- [x] T020 [US1] Generate `frontend/docs/module-2-simulation/03-simulating-sensors.md` with detailed explanation of implementing virtual LiDAR, Depth Cameras, and IMUs in a simulated world. Author: Syed Ahmed Raza, Sidebar position: 7.
- [x] T021 [US1] Generate `frontend/docs/module-2-simulation/04-unity-for-robotics.md` with detailed introduction to Unity Robotics Hub and importing URDFs for high-fidelity visualization. Author: Syed Ahmed Raza, Sidebar position: 8.

### Content Generation Tasks (Module 3 - The AI-Robot Brain)

- [x] T022 [US1] Create directory `frontend/docs/module-3-isaac/`
- [x] T023 [US1] Generate `frontend/docs/module-3-isaac/01-nvidia-isaac-ecosystem.md` with detailed overview of NVIDIA Isaac Sim, Isaac ROS, and the Omniverse platform for robotics. Author: Syed Ahmed Raza, Sidebar position: 9.
- [x] T024 [US1] Generate `frontend/docs/module-3-isaac/02-perception-and-navigation.md` with technical guide on Visual SLAM (VSLAM), Nav2 for humanoid path planning, and Isaac ROS GEMs. Author: Syed Ahmed Raza, Sidebar position: 10.
- [x] T025 [US1] Generate `frontend/docs/module-3-isaac/03-reinforcement-learning.md` with explanation on using Isaac Gym for training bipedal walking policies using Reinforcement Learning (RL). Author: Syed Ahmed Raza, Sidebar position: 11.
- [x] T026 [US1] Generate `frontend/docs/module-3-isaac/04-sim-to-real-transfer.md` with strategies for Domain Randomization and deploying trained models to physical hardware like NVIDIA Jetson Orin. Author: Syed Ahmed Raza, Sidebar position: 12.

### Content Generation Tasks (Module 4 - Vision-Language-Action & Generative AI)

- [x] T027 [US1] Create directory `frontend/docs/module-4-vla/`
- [x] T028 [US1] Generate `frontend/docs/module-4-vla/01-voice-to-action.md` with technical implementation of OpenAI Whisper for converting voice commands into text for robots. Author: Syed Ahmed Raza, Sidebar Position: 13.
- [x] T029 [US1] Generate `frontend/docs/module-4-vla/02-cognitive-planning-llms.md` with explanation on how to use LLMs (like GPT-4o) to translate natural language ("Clean the room") into a sequence of ROS 2 actions (Plan-Execute pattern). Author: Syed Ahmed Raza, Sidebar Position: 14.
- [x] T030 [US1] Generate `frontend/docs/module-4-vla/03-vla-models.md` with introduction to Vision-Language-Action models (like Google RT-2) and how they merge computer vision with language reasoning. Author: Syed Ahmed Raza, Sidebar Position: 15.
- [x] T031 [US1] Generate `frontend/docs/module-4-vla/04-capstone-project.md` with a complete architectural breakdown of the "Autonomous Humanoid" Project (robot that hears command, plans, navigates, manipulates). Author: Syed Ahmed Raza, Sidebar Position: 16.

### Landing Page Content (Phase 3: User Story 1 - Comprehensive Textbook Access)

- [x] T032 [US1] Overwrite `frontend/src/pages/index.tsx` with a highly professional React layout for the landing page.
- [x] T033 [US1] Implement a Dark-Themed Hero Section with the main title "Physical AI & Humanoid Robotics", subtitle "Mastering ROS 2, Sim-to-Real Transfer, and VLA Models", Author Badge "Authored by Syed Ahmed Raza", and a "Start Reading 🚀" button linking to "/docs/module-1-ros2/01-foundations-of-physical-ai".
- [x] T034 [US1] Implement a "Curriculum" Section displaying 4 Cards for each Module: "The Nervous System" (Focus: ROS 2 & Middleware), "The Digital Twin" (Focus: Gazebo & Unity Simulation), "The AI Brain" (Focus: NVIDIA Isaac & Reinforcement Learning), "Generative Robotics" (Focus: VLA Models & GPT-4o).
- [x] T035 [US1] Ensure proper use of Docusaurus `<Layout>` and `<Link>` components.
- [x] T036 [US1] Remove all default text about "Dinosaurs" from `frontend/src/pages/index.tsx`.

### Docusaurus Integration Tasks

- [ ] T037 [US1] Configure Docusaurus sidebar to include "Module 1", "Module 2", "Module 3", and "Module 4" content. (`frontend/docusaurus.config.js` and `frontend/sidebars.ts`)
- [ ] T038 [US1] Ensure all Docusaurus pages display "Author: Syed Ahmed Raza" (global configuration or template modification in `frontend/`)
- [ ] T039 [US1] Verify successful local build and navigation of Docusaurus site with all modules content.

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - RAG Chatbot Interaction (Priority: P1)

**Goal**: Implement the RAG chatbot for interactive explanations.

**Independent Test**: Select text in Docusaurus and receive RAG-based explanations.

### Implementation for User Story 2

- [ ] T040 [US2] Implement text selection mechanism in Docusaurus frontend (`frontend/src/components/TextSelector.js`)
- [x] T041 [US2] Create FastAPI endpoint /chat for RAG requests (`backend/main.py`)
- [ ] T042 [US2] Integrate OpenAI SDK into FastAPI for LLM (`backend/app/services/llm_service.py`)
- [ ] T043 [US2] Develop RAG logic in FastAPI to:
    - Receive selected text and context.
    - Create vector embeddings for query.
    - Retrieve relevant document chunks from Qdrant.
    - Retrieve metadata from Neon Postgres.
    - Formulate prompt for LLM using retrieved info (`backend/app/services/rag_service.py`)
- [x] T044 [US2] Create Docusaurus frontend component to display chatbot interface and responses (`frontend/src/components/ChatBot.tsx`)
- [x] T045 [US2] Connect Docusaurus frontend component to FastAPI `/chat` endpoint (`frontend/src/pages/index.tsx`).
- [ ] T046 [US2] Implement error handling for RAG chatbot (e.g., no relevant information found, API errors).

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T047 [P] Ensure all generated files and documentation consistently include "Author: Syed Ahmed Raza" in headers or metadata.
- [ ] T048 Code cleanup and refactoring for both `frontend/` and `backend/`.
- [ ] T049 Performance optimization for RAG chatbot response times (SC-002).
- [ ] T050 Security hardening for FastAPI backend.
- [ ] T051 Implement basic CI/CD for Docusaurus deployment to GitHub Pages.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence