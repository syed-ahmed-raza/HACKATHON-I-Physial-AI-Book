---
id: 2
title: Generate detailed tasks for textbook spec
stage: tasks
date: 2025-12-16
surface: agent
model: gemini
feature: 001-textbook-rag-spec
branch: 001-textbook-rag-spec
user: Syed Ahmed Raza
command: /sp.tasks Context for task generation: I am Syed Ahmed Raza. Execute the following implementation task for Module 1. ACTION: Create a new directory at '../physical-ai-book/docs/module-1/'. Inside this directory, generate 4 highly detailed Markdown files (.md) with the following content requirements: 1. '01-foundations-of-physical-ai.md': Explain embodied intelligence and the Sense-Think-Act cycle. Include a section on how physical laws differ from digital data. 2. '02-sensor-systems.md': Technical deep dive into LiDAR, IMU, and Force/Torque sensors. Explain their role in humanoid balance. 3. '03-ros2-architecture.md': Write a professional guide on Nodes, Topics, Services, and Actions. Include a Mermaid diagram or text representation of a ROS 2 graph. 4. '04-building-packages-python.md': Provide a real Python code snippet using 'rclpy' to create a basic node and explain Launch files. CONSTRAINTS: - Author name: Syed Ahmed Raza. - Tone: Senior Robotics Engineer / Educational. - Sidebar Position: 1 to 4. - Language: Professional English. - Must include code blocks for technical sections. Mark this task as completed in specs/tasks.md once the files are written.
labels: ["tasks", "generation", "planning"]
links:
  spec: /specs/001-textbook-rag-spec/spec.md
  ticket: null
  adr: null
  pr: null
files:
- specs/001-textbook-rag-spec/tasks.md
tests:
null
---

## Prompt

---
description: Generate an actionable, dependency-ordered tasks.md for the feature based on available design artifacts.
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

1. **Setup**: Run `.specify/scripts/powershell/check-prerequisites.ps1 -Json` from repo root and parse FEATURE_DIR and AVAILABLE_DOCS list. All paths must be absolute. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot").

2. **Load design documents**: Read from FEATURE_DIR:
   - **Required**: plan.md (tech stack, libraries, structure), spec.md (user stories with priorities)
   - **Optional**: data-model.md (entities), contracts/ (API endpoints), research.md (decisions), quickstart.md (test scenarios)
   - Note: Not all projects have all documents. Generate tasks based on what's available.

3. **Execute task generation workflow**:
   - Load plan.md and extract tech stack, libraries, project structure
   - Load spec.md and extract user stories with their priorities (P1, P2, P3, etc.)
   - If data-model.md exists: Extract entities and map to user stories
   - If contracts/ exists: Map endpoints to user stories
   - If research.md exists: Extract decisions for setup tasks
   - Generate tasks organized by user story (see Task Generation Rules below)
   - Generate dependency graph showing user story completion order
   - Create parallel execution examples per user story
   - Validate task completeness (each user story has all needed tasks, independently testable)

4. **Generate tasks.md**: Use `.specify.specify/templates/tasks-template.md` as structure, fill with:
   - Correct feature name from plan.md
   - Phase 1: Setup tasks (project initialization)
   - Phase 2: Foundational tasks (blocking prerequisites for all user stories)
   - Phase 3+: One phase per user story (in priority order from spec.md)
   - Each phase includes: story goal, independent test criteria, tests (if requested), implementation tasks
   - Final Phase: Polish & cross-cutting concerns
   - All tasks must follow the strict checklist format (see Task Generation Rules below)
   - Clear file paths for each task
   - Dependencies section showing story completion order
   - Parallel execution examples per user story
   - Implementation strategy section (MVP first, incremental delivery)

5. **Report**: Output path to generated tasks.md and summary:
   - Total task count
   - Task count per user story
   - Parallel opportunities identified
   - Independent test criteria for each story
   - Suggested MVP scope (typically just User Story 1)
   - Format validation: Confirm ALL tasks follow the checklist format (checkbox, ID, labels, file paths)

Context for task generation: I am Syed Ahmed Raza. Execute the following implementation task for Module 1. ACTION: Create a new directory at '../physical-ai-book/docs/module-1/'. Inside this directory, generate 4 highly detailed Markdown files (.md) with the following content requirements: 1. '01-foundations-of-physical-ai.md': Explain embodied intelligence and the Sense-Think-Act cycle. Include a section on how physical laws differ from digital data. 2. '02-sensor-systems.md': Technical deep dive into LiDAR, IMU, and Force/Torque sensors. Explain their role in humanoid balance. 3. '03-ros2-architecture.md': Write a professional guide on Nodes, Topics, Services, and Actions. Include a Mermaid diagram or text representation of a ROS 2 graph. 4. '04-building-packages-python.md': Provide a real Python code snippet using 'rclpy' to create a basic node and explain Launch files. CONSTRAINTS: - Author name: Syed Ahmed Raza. - Tone: Senior Robotics Engineer / Educational. - Sidebar Position: 1 to 4. - Language: Professional English. - Must include code blocks for technical sections. Mark this task as completed in specs/tasks.md once the files are written.

The tasks.md should be immediately executable - each task must be specific enough that an LLM can complete it without additional context.

## Task Generation Rules

**CRITICAL**: Tasks MUST be organized by user story to enable independent implementation and testing.

**Tests are OPTIONAL**: Only generate test tasks if explicitly requested in the feature specification or if user requests TDD approach.

### Checklist Format (REQUIRED)

Every task MUST strictly follow this format:

```text
- [ ] [TaskID] [P?] [Story?] Description with file path
```

**Format Components**:

1. **Checkbox**: ALWAYS start with `- [ ]` (markdown checkbox)
2. **Task ID**: Sequential number (T001, T002, T003...) in execution order
3. **[P] marker**: Include ONLY if task is parallelizable (different files, no dependencies on incomplete tasks)
4. **[Story] label**: REQUIRED for user story phase tasks only
   - Format: [US1], [US2], [US3], etc. (maps to user stories from spec.md)
   - Setup phase: NO story label
   - Foundational phase: NO story label  
   - User Story phases: MUST have story label
   - Polish phase: NO story label
5. **Description**: Clear action with exact file path

**Examples**:

- ✅ CORRECT: `- [ ] T001 Create project structure per implementation plan`
- ✅ CORRECT: `- [ ] T005 [P] Implement authentication middleware in src/middleware/auth.py`
- ✅ CORRECT: `- [ ] T012 [P] [US1] Create User model in src/models/user.py`
- ✅ CORRECT: `- [ ] T014 [US1] Implement UserService in src/services/user_service.py`
- ❌ WRONG: `- [ ] Create User model` (missing ID and Story label)
- ❌ WRONG: `T001 [US1] Create model` (missing checkbox)
- ❌ WRONG: `- [ ] [US1] Create User model` (missing Task ID)
- ❌ WRONG: `- [ ] T001 [US1] Create model` (missing file path)

### Task Organization

1. **From User Stories (spec.md)** - PRIMARY ORGANIZATION:
   - Each user story (P1, P2, P3...) gets its own phase
   - Map all related components to their story:
     - Models needed for that story
     - Services needed for that story
     - Endpoints/UI needed for that story
     - If tests requested: Tests specific to that story
   - Mark story dependencies (most stories should be independent)

2. **From Contracts**:
   - Map each contract/endpoint → to the user story it serves
   - If tests requested: Each contract → contract test task [P] before implementation in that story's phase

3. **From Data Model**:
   - Map each entity to the user story(ies) that need it
   - If entity serves multiple stories: Put in earliest story or Setup phase
   - Relationships → service layer tasks in appropriate story phase

4. **From Setup/Infrastructure**:
   - Shared infrastructure → Setup phase (Phase 1)
   - Foundational/blocking tasks → Foundational phase (Phase 2)
   - Story-specific setup → within that story's phase

### Phase Structure

- **Phase 1**: Setup (project initialization)
- **Phase 2**: Foundational (blocking prerequisites - MUST complete before user stories)
- **Phase 3+**: User Stories in priority order (P1, P2, P3...)
  - Within each story: Tests (if requested) → Models → Services → Endpoints → Integration
  - Each phase should be a complete, independently testable increment
- **Final Phase**: Polish & Cross-Cutting Concerns

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage: 
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.
```

## Response snapshot

The detailed `tasks.md` for "Physical AI & Humanoid Robotics Textbook Master Specification" has been generated at `specs/001-textbook-rag-spec/tasks.md`. It includes 29 tasks across Setup, Foundational, User Story 1 (Comprehensive Textbook Access), User Story 2 (RAG Chatbot Interaction), and Polish & Cross-Cutting Concerns phases, with parallel opportunities identified and independent test criteria. The generated tasks also incorporate the user's explicit request for content generation for Module 1.

## Outcome

- ✅ Impact: Detailed, dependency-ordered tasks for the "Physical AI & Humanoid Robotics Textbook Master Specification" feature have been generated, incorporating specific content creation requirements.
- 🧪 Tests: Task list adheres to the specified checklist format.
- 📁 Files: `specs/001-textbook-rag-spec/tasks.md` has been updated.
- 🔁 Next prompts: The task list is ready for execution.
- 🧠 Reflection: Successfully integrated specific user requests for content generation into the broader task planning for the feature. The task list is now actionable and follows a structured development flow.

## Evaluation notes (flywheel)

- Failure modes observed: `create-phr.sh` not working in PowerShell environment. Handled by agent-native PHR creation.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): N/A

```