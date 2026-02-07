# Phase 1: Project Management & Infrastructure
## Work Breakdown Structure - Detailed Execution Guide

---

**Document Information**

| Field | Value |
|-------|-------|
| **Document Title** | WBS Phase 1: Project Management & Infrastructure |
| **Version** | 2.0 |
| **Date** | February 8, 2026 |
| **Author** | Ratan Lal Bunkar |
| **Status** | Active |
| **Phase** | 1.0 - Infrastructure Setup |
| **WBS IDs Covered** | 1.0, 1.1-1.4 (all subsections) |

---

### Document Navigation

- **Master Overview**: [00_WBS_Master_Overview.md](./00_WBS_Master_Overview.md)
- **Previous Phase**: None (this is the first phase)
- **Next Phase**: [02_WBS_Phase2_SITL_Baseline.md](./02_WBS_Phase2_SITL_Baseline.md)
- **All Phases**: [See Master Overview Navigation Guide](./00_WBS_Master_Overview.md#15-navigation-to-detail-documents)

---

## Phase 1 Overview

### Scope and Objectives

Phase 1 establishes the foundational project infrastructure required to support all subsequent technical work. This infrastructure-focused phase creates the systems, processes, and tools that enable effective requirements management, version control, continuous integration, documentation, and project governance. Completing this phase ensures the project team operates from a common technical foundation with established workflows, automated quality gates, and formal decision processes.

### Total Duration and Effort

- **Duration**: 2 weeks
- **Total Effort**: 120 person-hours
- **Work Packages**: 12 Level 3 deliverables across 4 Level 2 subsystems

### Key Deliverables

1. **Requirements Management System**: Operational database with traceability matrix and review processes
2. **Version Control Infrastructure**: Git repository with CI/CD pipelines for both simulation and firmware
3. **Documentation Framework**: Templates, automated generation, and version-controlled documentation repository
4. **Stage Gate Procedures**: Defined criteria, schedules, and review procedures for all four project stage gates

### Prerequisites and Dependencies

- **Prerequisites**: None (Phase 1 is the entry point for the project)
- **Dependencies**: Phase 2 (SITL Baseline) depends on completion of core infrastructure from this phase

### Required Resources

**Personnel**:
- Systems Engineer (lead role, 60 hours)
- Software Configuration Manager or Senior Developer (40 hours)
- Technical Writer or Systems Engineer for documentation (20 hours)
- Project Manager (support role throughout)

**Hardware**:
- Development workstations for team
- CI/CD server (physical or cloud-based, 8+ CPU cores, 16+ GB RAM recommended)

**Software/Tools**:
- Version control: Git, GitLab/GitHub/Bitbucket
- Requirements management: DOORS/Jama/Helix RM (or open-source alternatives)
- CI/CD: Jenkins/GitLab CI/GitHub Actions
- Documentation: Sphinx/Doxygen, Markdown editors
- Build tools: Docker, ROS/Gazebo, ARM GCC toolchain

**Budget**: $700-1500 total
- Requirements management tool licenses: $200-500
- CI/CD hosting: $200-500 (can use free tiers initially)
- Documentation hosting: $0-100 (usually free)
- Development tools: $300-500 (or $0 if using entirely open-source stack)

### Success Criteria

Phase 1 is considered successful when:

✅ Requirements database is operational with at least 50 initial requirements entered
✅ Requirements traceability matrix framework established
✅ Git repository accessible to team with defined structure and branching strategy
✅ CI/CD pipelines execute successfully for both simulation and firmware code
✅ Documentation templates created and automation pipeline working
✅ Stage gate criteria defined for all four transitions with stakeholder approval

### Risk Summary

| Risk | Probability/Impact | Mitigation |
|------|-------------------|------------|
| Tool selection disagreements | Medium/Low | Early stakeholder consensus building |
| Learning curve for new tools | Medium/Medium | Allocate training time, use familiar tools where possible |
| Infrastructure scope creep | Medium/Medium | Limit to minimum viable capabilities |
| CI/CD complexity | Medium/High | Start simple, add incrementally |
| Documentation drift from code | High/High | Automate documentation generation |

### Phase Timeline

**Week 1**:
- Days 1-3: Requirements elicitation and documentation (1.1.1, 1.1.2)
- Days 1-2: Repository structure and branching strategy (1.2.1, 1.2.2)
- Days 1-3: Documentation templates creation (1.3.1)
- Days 1-3: Stage gate criteria definition (1.4.1)

**Week 2**:
- Days 1-2: Requirements traceability matrix and review process (1.1.3, 1.1.4)
- Days 1-4: CI/CD pipeline development for simulation and firmware (1.2.3, 1.2.4 - parallel)
- Days 1-2: Documentation repository setup (1.3.2)
- Days 1-2: Stage gate procedures, schedule, and stakeholder ID (1.4.2, 1.4.3, 1.4.4)

---

## Detailed WBS Dictionary

### 1.0 PROJECT MANAGEMENT & INFRASTRUCTURE

**Level**: 1
**Parent**: Root
**Duration**: Entire project (ongoing)
**Effort**: 120 person-hours

**Description**:
This phase establishes the foundational project infrastructure required to support all subsequent technical work. It encompasses requirements management processes, version control systems, continuous integration/continuous deployment pipelines, documentation frameworks, and stage gate review procedures. This infrastructure enables systematic tracking of requirements traceability, automated testing and validation, consistent documentation practices, and formal decision points between project stages. All infrastructure must be in place before significant technical work begins in Stage 1.

**Deliverables**:
- Requirements management database and traceability matrix
- Version control repository with CI/CD pipelines
- Documentation framework and templates
- Stage gate review procedures and criteria

**Prerequisites/Dependencies**:
- None (first phase to execute)

**Required Resources**:
- **Personnel**: Systems engineer, software configuration manager
- **Hardware**: Development workstations
- **Software/Tools**: Git, GitLab/GitHub, requirements management tool (DOORS/Jama/Helix), documentation tools (Sphinx/Doxygen), CI/CD platform (Jenkins/GitLab CI)
- **Budget**: $500-1000 for tool licenses if not using open-source alternatives

**Success Criteria**:
- Requirements database operational with all initial requirements entered
- Git repository accessible with at least one successful CI pipeline run
- Documentation templates created and approved
- Stage gate criteria documented and approved by stakeholders

**Risk Factors**:
- **Tool selection disagreements** (Medium/Low): Mitigate through early stakeholder consensus
- **Learning curve for new tools** (Medium/Medium): Allocate time for training
- **Scope creep in infrastructure setup** (Medium/Medium): Limit to essential capabilities for project start

**Notes**:
This phase should be completed within the first 2 weeks of project initiation. Resist the temptation to over-engineer infrastructure—focus on minimum viable capabilities that enable technical work to proceed.

**Parallelization**: Can run partially in parallel with early Stage 1 work (2.1), but must complete core infrastructure before Stage 1 progresses significantly.

---

### 1.1 Requirements Management

**Level**: 2
**Parent**: 1.0
**Duration**: 1.5 weeks
**Effort**: 40 person-hours

**Description**:
Establish a systematic requirements management process to capture, document, trace, and verify all project requirements across functional, performance, interface, and constraint categories. This subsystem ensures requirements completeness, consistency, and traceability from high-level system requirements down to individual design and test elements. The requirements management process forms the foundation for verification and validation activities throughout the project lifecycle.

**Deliverables**:
- Requirements management plan
- Requirements database populated with initial requirements
- Requirements traceability matrix (RTM) framework
- Requirements review procedures

**Prerequisites/Dependencies**:
- None (can start immediately)

**Required Resources**:
- **Personnel**: Systems engineer (lead), domain experts for requirements elicitation
- **Hardware**: None specific
- **Software/Tools**: Requirements management tool (DOORS/Jama/Helix RM or open-source alternative)
- **Budget**: $200-500 for requirements management tool if commercial

**Success Criteria**:
- At least 50 initial requirements captured spanning all requirement types
- Traceability matrix established with parent-child linkages
- Requirements review process defined with review checklist

**Risk Factors**:
- **Incomplete requirements capture** (High/High): Conduct thorough stakeholder interviews and use domain expertise
- **Requirements ambiguity** (Medium/High): Apply requirements quality criteria (clear, testable, traceable)

**Notes**:
Requirements will evolve throughout the project—this initial effort establishes the framework and captures baseline requirements. Expect requirements refinement at each stage gate.

**Parallelization**: Must complete before Stage 1 technical work begins in earnest; can run in parallel with 1.2 (Version Control setup).

---

### 1.1.1 Requirements Elicitation

**Level**: 3
**Parent**: 1.1
**Duration**: 3 days
**Effort**: 16 person-hours

**Description**:
Conduct structured requirements gathering activities through stakeholder interviews, literature review of UAV flight controller specifications, analysis of reinforcement learning deployment constraints, and embedded systems requirements analysis. This task identifies functional requirements (what the system must do), performance requirements (how well it must perform), interface requirements (how it interacts with external systems), and constraint requirements (limitations imposed by technology, regulations, or resources). Use structured elicitation techniques including use case analysis, operational concept documents, and constraint analysis.

**Deliverables**:
- Requirements elicitation notes and meeting minutes
- Stakeholder interview summaries
- Initial requirements list (unstructured)

**Prerequisites/Dependencies**:
- None

**Required Resources**:
- **Personnel**: Systems engineer, project stakeholders
- **Hardware**: None
- **Software/Tools**: Meeting software, note-taking tools
- **Budget**: None

**Success Criteria**:
- At least 3 stakeholder interviews conducted
- Minimum 40 candidate requirements identified
- Requirements span all categories (functional, performance, interface, constraint)

**Risk Factors**:
- **Stakeholder unavailability** (Medium/Medium): Schedule interviews early and confirm attendance
- **Conflicting stakeholder needs** (Medium/High): Document conflicts and escalate to project leadership for resolution

**Notes**:
Focus on high-level requirements in this initial elicitation. Lower-level derived requirements will emerge during design phases.

**Parallelization**: Can run in parallel with 1.2.1 (Repository Structure definition).

---

### 1.1.2 Requirements Documentation

**Level**: 3
**Parent**: 1.1
**Duration**: 4 days
**Effort**: 16 person-hours

**Description**:
Transform elicited requirements into formal, structured requirements statements that are clear, concise, testable, and traceable. Each requirement must follow the standard format with unique identifier, statement using "shall" language, rationale, priority, and verification method. Requirements are organized hierarchically into system-level requirements and derived subsystem requirements. Ambiguous or poorly stated requirements are refined through iterative review. Apply requirements quality criteria including atomicity, consistency, feasibility, unambiguity, and verifiability.

**Deliverables**:
- System Requirements Specification (SRS) document
- Requirements database entries with all required fields populated
- Requirements organized by subsystem and category

**Prerequisites/Dependencies**:
- 1.1.1: Requirements Elicitation must be complete

**Required Resources**:
- **Personnel**: Systems engineer
- **Hardware**: None
- **Software/Tools**: Requirements management tool, word processor
- **Budget**: None

**Success Criteria**:
- All requirements follow standard format with unique IDs
- Each requirement includes verification method
- Requirements pass quality review using criteria checklist
- SRS document approved

**Risk Factors**:
- **Requirements ambiguity** (High/High): Apply structured review process, use specific language, avoid "TBD" placeholders
- **Over-specification** (Medium/Medium): Balance detail with flexibility for design iteration

**Notes**:
Use IEEE 29148 or similar standard as a guide for requirements documentation. Prioritize requirements as critical, important, or desirable to guide development focus.

**Parallelization**: Must follow 1.1.1 sequentially; can run in parallel with 1.2 (Version Control setup).

---

### 1.1.3 Requirements Traceability Matrix

**Level**: 3
**Parent**: 1.1
**Duration**: 2 days
**Effort**: 8 person-hours

**Description**:
Create a requirements traceability matrix that establishes bidirectional traceability links between system requirements, design elements, implementation components, and verification activities. The RTM ensures complete requirement coverage and enables impact analysis when requirements change. Traceability includes upward links (derived requirements to parent requirements), downward links (requirements to design and implementation), and horizontal links (requirements to verification test cases). The RTM is implemented within the requirements management tool with automated reporting capabilities.

**Deliverables**:
- Requirements Traceability Matrix in database format
- Traceability reporting templates
- Initial traceability links for all documented requirements

**Prerequisites/Dependencies**:
- 1.1.2: Requirements Documentation must be complete

**Required Resources**:
- **Personnel**: Systems engineer
- **Hardware**: None
- **Software/Tools**: Requirements management tool with traceability capability
- **Budget**: None

**Success Criteria**:
- All documented requirements have at least one traceability link
- RTM can be automatically generated from database
- Traceability completeness report shows 100% coverage

**Risk Factors**:
- **Incomplete traceability** (Medium/High): Establish traceability as requirements are documented, not retroactively
- **Tool limitations** (Low/Medium): Verify tool supports needed traceability relationships before committing

**Notes**:
Traceability maintenance is ongoing throughout the project. This task establishes the framework; subsequent work packages will add design and verification links.

**Parallelization**: Must follow 1.1.2 sequentially; can run in parallel with 1.3 (Documentation Framework).

---

### 1.1.4 Requirements Review Process

**Level**: 3
**Parent**: 1.1
**Duration**: 1 day
**Effort**: 4 person-hours

**Description**:
Define a formal requirements review process that specifies review roles, responsibilities, entry and exit criteria, review checklists, and approval workflows. The process includes peer reviews for requirements quality, stakeholder reviews for requirement accuracy and completeness, and technical reviews for requirements feasibility. Establish review cadence tied to project milestones and stage gates. Document escalation procedures for unresolved requirements issues and change control procedures for requirements modifications.

**Deliverables**:
- Requirements Review Procedure document
- Requirements review checklist
- Requirements change control procedure

**Prerequisites/Dependencies**:
- 1.1.2: Requirements Documentation (should be defined after initial requirements are documented)

**Required Resources**:
- **Personnel**: Systems engineer, project manager
- **Hardware**: None
- **Software/Tools**: None specific
- **Budget**: None

**Success Criteria**:
- Review procedure documented and approved
- Review checklist covers all quality criteria
- Change control procedure integrated with version control system

**Risk Factors**:
- **Review process too heavyweight** (Medium/Medium): Tailor process complexity to project size and team structure
- **Lack of stakeholder engagement** (Medium/High): Clearly communicate review importance and expectations

**Notes**:
The first formal requirements review should occur after Stage 1 requirements are fully documented, prior to the Stage 1→2 gate.

**Parallelization**: Can run in parallel with 1.3 and 1.4 activities.

---

### 1.2 Version Control & CI/CD Setup

**Level**: 2
**Parent**: 1.0
**Duration**: 1.5 weeks
**Effort**: 40 person-hours

**Description**:
Establish version control infrastructure and continuous integration/continuous deployment pipelines for both simulation software and embedded firmware. This subsystem includes Git repository setup with appropriate branching strategies, automated build and test pipelines, code quality checks, and deployment automation. Separate pipelines are required for simulation environment (Linux-based, ROS/Gazebo) and firmware (cross-compilation for embedded targets). The CI/CD infrastructure enables rapid iteration, automated testing, and consistent builds across the development team.

**Deliverables**:
- Git repository with defined structure
- Branching strategy documentation
- Simulation CI/CD pipeline operational
- Firmware CI/CD pipeline operational

**Prerequisites/Dependencies**:
- None (can start immediately, in parallel with 1.1)

**Required Resources**:
- **Personnel**: Software configuration manager or senior developer
- **Hardware**: CI/CD server (physical or cloud-based)
- **Software/Tools**: Git, GitLab/GitHub, Jenkins/GitLab CI/GitHub Actions, Docker for build environments
- **Budget**: $200-500 for CI/CD hosting if using commercial services

**Success Criteria**:
- Repository accessible to all team members with appropriate permissions
- At least one successful automated build and test run for simulation code
- At least one successful cross-compilation and test run for firmware
- Build status visible via dashboard or badges

**Risk Factors**:
- **CI/CD complexity** (Medium/High): Start with simple pipelines and add complexity incrementally
- **Build environment reproducibility** (Medium/High): Use containerized build environments
- **Cross-compilation challenges** (High/Medium): Allocate extra time for embedded toolchain setup

**Notes**:
Invest adequate time in this infrastructure—it will significantly accelerate development and reduce integration issues later in the project.

**Parallelization**: Can run fully in parallel with 1.1 (Requirements Management).

---

### 1.2.1 Repository Structure

**Level**: 3
**Parent**: 1.2
**Duration**: 2 days
**Effort**: 8 person-hours

**Description**:
Define and implement the Git repository structure that organizes code, documentation, tests, and configuration files in a logical hierarchy. Structure should separate simulation code, firmware code, RL training code, hardware design files, documentation, and test infrastructure into distinct directories. Consider using a monorepo approach with clear subsystem boundaries or multiple repositories with defined integration points. Include README files, .gitignore configurations, and directory structure documentation. Establish naming conventions for files, directories, and branches.

**Deliverables**:
- Repository created with directory structure
- Repository structure documentation (README)
- .gitignore files configured for each subsystem
- Naming conventions documented

**Prerequisites/Dependencies**:
- None

**Required Resources**:
- **Personnel**: Software configuration manager
- **Hardware**: None
- **Software/Tools**: Git, GitLab/GitHub
- **Budget**: None (assuming use of free tier hosting)

**Success Criteria**:
- Repository accessible to team with appropriate permissions
- Directory structure supports all project code types
- README files provide clear navigation guidance
- .gitignore prevents accidental commit of build artifacts

**Risk Factors**:
- **Structure too rigid** (Low/Medium): Design for flexibility while maintaining organization
- **Monorepo vs. multi-repo decision** (Medium/Medium): Evaluate based on team size and integration complexity

**Notes**:
Recommended structure includes top-level directories: /simulation, /firmware, /training, /hardware, /docs, /tests. Use Git submodules if integrating external repositories.

**Parallelization**: Can run in parallel with all other Level 3 tasks in Phase 1.

---

### 1.2.2 Branching Strategy

**Level**: 3
**Parent**: 1.2
**Duration**: 1 day
**Effort**: 4 person-hours

**Description**:
Define the Git branching strategy that governs how features are developed, how releases are managed, and how hotfixes are applied. Common approaches include Git Flow, GitHub Flow, or trunk-based development. Strategy must specify main branch(es), development branches, feature branch naming, merge procedures, code review requirements, and tag conventions for releases. Consider the project's stage-gate structure when defining release branches. Document merge policies including whether to use merge commits, squash merging, or rebasing.

**Deliverables**:
- Branching strategy document
- Branch naming conventions
- Merge policy and procedures
- Release tagging conventions

**Prerequisites/Dependencies**:
- 1.2.1: Repository Structure must be defined

**Required Resources**:
- **Personnel**: Software configuration manager, development team leads
- **Hardware**: None
- **Software/Tools**: Git
- **Budget**: None

**Success Criteria**:
- Branching strategy documented and approved by development team
- Branch naming conventions clear and unambiguous
- Merge policy supports code quality goals (e.g., requires code review)

**Risk Factors**:
- **Strategy too complex for team size** (Medium/Medium): Tailor complexity to team needs
- **Strategy doesn't support stage gates** (Low/High): Ensure release branches align with project stages

**Notes**:
For this project, consider using main branches for each stage (main-stage1, main-stage2, etc.) with feature branches and pull request workflow. Tag each stage gate approval.

**Parallelization**: Must follow 1.2.1; can run in parallel with 1.2.3 and 1.2.4.

---

### 1.2.3 CI/CD Pipeline for Simulation

**Level**: 3
**Parent**: 1.2
**Duration**: 4 days
**Effort**: 16 person-hours

**Description**:
Implement automated build, test, and deployment pipeline for simulation software stack (Gazebo, ROS/ROS2, RL training code). Pipeline should trigger on code commits and pull requests, execute in containerized environment for reproducibility, run unit tests and integration tests, perform static code analysis, and generate build artifacts. Include stages for dependency installation, build compilation, test execution, and reporting. Configure pipeline to fail on test failures or code quality violations. Deploy successful builds to artifact repository or container registry.

**Deliverables**:
- CI/CD pipeline configuration files
- Dockerfiles for simulation build environment
- Automated test execution scripts
- Build status dashboard

**Prerequisites/Dependencies**:
- 1.2.1: Repository Structure
- 1.2.2: Branching Strategy

**Required Resources**:
- **Personnel**: Software developer with DevOps experience
- **Hardware**: CI/CD server with sufficient compute for simulation (8+ CPU cores, 16+ GB RAM recommended)
- **Software/Tools**: GitLab CI/Jenkins/GitHub Actions, Docker, ROS/Gazebo, pytest/gtest
- **Budget**: $100-300/month for CI/CD compute resources

**Success Criteria**:
- Pipeline executes successfully on sample simulation code
- Test failures cause pipeline to fail
- Build artifacts accessible for download
- Pipeline execution time under 15 minutes for typical commit

**Risk Factors**:
- **Long build times** (High/Medium): Use caching and incremental builds; consider distributed build system
- **Flaky tests** (Medium/High): Isolate and fix non-deterministic tests; quarantine if necessary
- **Resource constraints** (Medium/Medium): Monitor resource usage and scale appropriately

**Notes**:
Gazebo simulation tests may require GPU acceleration or X virtual framebuffer (xvfb) for headless operation. Consider using Docker Compose for multi-container test scenarios.

**Parallelization**: Can run in parallel with 1.2.4 (Firmware CI/CD pipeline).

---

### 1.2.4 CI/CD Pipeline for Firmware

**Level**: 3
**Parent**: 1.2
**Duration**: 4 days
**Effort**: 16 person-hours

**Description**:
Implement automated cross-compilation, test, and deployment pipeline for embedded firmware. Pipeline must support ARM Cortex-M cross-compilation toolchain, execute unit tests via hardware abstraction layer mocks or emulation (QEMU), perform static analysis with embedded-specific rules, and generate flashable binary artifacts. Include linting, memory usage reports, and code size analysis. Configure pipeline to run on firmware commits with faster execution time than simulation pipeline. Support multiple target platforms if developing for different MCU evaluation boards.

**Deliverables**:
- Firmware CI/CD pipeline configuration
- Cross-compilation Docker container
- Firmware test framework with mocking
- Binary artifact generation and storage

**Prerequisites/Dependencies**:
- 1.2.1: Repository Structure
- 1.2.2: Branching Strategy

**Required Resources**:
- **Personnel**: Embedded software developer
- **Hardware**: CI/CD server (can share with simulation pipeline)
- **Software/Tools**: GitLab CI/Jenkins/GitHub Actions, Docker, ARM GCC toolchain, QEMU, CMake/Make, Ceedling/Unity for testing
- **Budget**: Minimal (uses same CI/CD infrastructure as simulation)

**Success Criteria**:
- Firmware builds successfully for target MCU
- Unit tests execute via mocking or emulation
- Memory usage reports generated
- Binary artifact size tracked over time

**Risk Factors**:
- **Toolchain setup complexity** (High/Medium): Use pre-built Docker images with toolchain; document setup thoroughly
- **Limited testing without hardware** (High/High): Prioritize HAL mocking; plan for hardware-based testing in Stage 3
- **Binary size bloat** (Medium/Medium): Track binary size in CI and set thresholds

**Notes**:
Consider using PlatformIO for simplified embedded build management. Initial pipeline can be simple (build + basic tests); expand as firmware develops.

**Parallelization**: Can run in parallel with 1.2.3 (Simulation CI/CD pipeline).

---

### 1.3 Documentation Framework

**Level**: 2
**Parent**: 1.0
**Duration**: 1 week
**Effort**: 24 person-hours

**Description**:
Establish documentation infrastructure including templates, tools, repository structure, and review procedures for all project documentation. Documentation types include requirements, design documents, test reports, user manuals, API documentation, and developer guides. Framework should support both human-readable documentation (Markdown, LaTeX) and auto-generated documentation from code (Doxygen, Sphinx). Implement documentation version control integrated with code repository and establish documentation review processes aligned with code review workflows.

**Deliverables**:
- Documentation templates for all document types
- Documentation repository structure
- Automated documentation generation pipeline
- Documentation review procedures

**Prerequisites/Dependencies**:
- 1.2.1: Repository Structure (documentation should be version-controlled)

**Required Resources**:
- **Personnel**: Technical writer or systems engineer
- **Hardware**: None
- **Software/Tools**: Sphinx/Doxygen, Markdown editor, LaTeX, documentation hosting (ReadTheDocs or similar)
- **Budget**: Minimal (most tools are open-source)

**Success Criteria**:
- Templates created for at least 5 document types
- Sample documentation successfully generates via automation
- Documentation accessible via web interface
- Review process integrated with Git workflow

**Risk Factors**:
- **Documentation overhead** (Medium/Medium): Balance thoroughness with agility; prioritize essential documentation
- **Documentation drift from code** (High/High): Automate documentation generation where possible; link docs to code reviews

**Notes**:
Focus on "documentation as code" approach—documentation lives in repository, versioned with code, and uses lightweight markup languages.

**Parallelization**: Can run in parallel with all other Phase 1 tasks.

---

### 1.3.1 Documentation Templates

**Level**: 3
**Parent**: 1.3
**Duration**: 3 days
**Effort**: 12 person-hours

**Description**:
Create standardized templates for all project documentation types to ensure consistency and completeness. Templates should include required sections, formatting guidelines, placeholder content with instructions, and examples. Document types include System Requirements Specification, Design Description Document, Test Plan, Test Report, User Manual, API Documentation, and Technical Memos. Templates use lightweight markup (Markdown) where possible for version control friendliness. Include front matter templates with metadata fields (document ID, version, date, author, approvers).

**Deliverables**:
- Template files for each document type (minimum 6 templates)
- Template usage guide
- Example documents using templates
- Style guide for documentation

**Prerequisites/Dependencies**:
- None

**Required Resources**:
- **Personnel**: Technical writer or systems engineer
- **Hardware**: None
- **Software/Tools**: Text editor, Markdown processor
- **Budget**: None

**Success Criteria**:
- Templates exist for all major document types
- Templates include all required sections per industry standards (e.g., IEEE standards)
- At least one example document created using each template
- Style guide covers formatting, terminology, and structural conventions

**Risk Factors**:
- **Templates too prescriptive** (Medium/Low): Design for flexibility while maintaining structure
- **Template adoption resistance** (Medium/Medium): Involve team in template review; demonstrate value

**Notes**:
Reference IEEE standards for technical documentation where applicable (IEEE 829 for test documentation, IEEE 1016 for design documentation). Keep templates lightweight for agility.

**Parallelization**: Can run in parallel with all other documentation framework tasks.

---

### 1.3.2 Documentation Repository

**Level**: 3
**Parent**: 1.3
**Duration**: 2 days
**Effort**: 8 person-hours

**Description**:
Set up documentation repository structure within the main Git repository or as a separate documentation repository. Organize documentation by type (requirements, design, test, user) and by project phase/stage. Configure automated documentation building tools (Sphinx/Doxygen) to generate static websites from source documentation. Set up documentation hosting on ReadTheDocs, GitHub Pages, or internal server. Ensure documentation build integrates with CI/CD pipeline so documentation is automatically updated on commits. Include search capability and navigation structure.

**Deliverables**:
- Documentation directory structure in repository
- Automated documentation build configuration
- Documentation website deployed
- Documentation navigation and search functional

**Prerequisites/Dependencies**:
- 1.2.1: Repository Structure
- 1.3.1: Documentation Templates (templates should exist before repository is populated)

**Required Resources**:
- **Personnel**: Software developer with documentation tools experience
- **Hardware**: Web server for documentation hosting (can use free services)
- **Software/Tools**: Sphinx/Doxygen, Git, ReadTheDocs/GitHub Pages
- **Budget**: None (using free hosting)

**Success Criteria**:
- Documentation builds automatically on commit
- Documentation website accessible to team
- Search functionality works across all documents
- Version selector shows documentation for different project stages

**Risk Factors**:
- **Build failures** (Medium/Medium): Test build configuration thoroughly; include in CI/CD checks
- **Hosting service limitations** (Low/Low): Free tiers usually sufficient for small projects

**Notes**:
Consider using Sphinx with MyST for Markdown support, or MkDocs for simpler setup. Structure should support both API documentation (auto-generated from code) and narrative documentation (manually written).

**Parallelization**: Must follow 1.2.1 and ideally 1.3.1; can run in parallel with 1.3.3.

---

### 1.3.3 Documentation Review Process

**Level**: 3
**Parent**: 1.3
**Duration**: 1 day
**Effort**: 4 person-hours

**Description**:
Define documentation review procedures that ensure documentation quality, accuracy, and completeness before publication. Process should mirror code review workflow using pull requests for documentation changes. Specify review criteria including technical accuracy, completeness per template, clarity of writing, correct terminology usage, and proper formatting. Define reviewer responsibilities and qualifications (e.g., design documents reviewed by technical leads, user documentation reviewed by intended users). Establish approval workflows and documentation release procedures tied to project milestones.

**Deliverables**:
- Documentation review procedure document
- Documentation review checklist
- Reviewer assignment matrix
- Documentation approval workflow

**Prerequisites/Dependencies**:
- 1.3.1: Documentation Templates

**Required Resources**:
- **Personnel**: Technical writer or systems engineer, project manager
- **Hardware**: None
- **Software/Tools**: None specific (uses existing Git/CI/CD infrastructure)
- **Budget**: None

**Success Criteria**:
- Review procedure documented and approved
- Review checklist covers quality, accuracy, and completeness
- All documentation types have assigned reviewer roles

**Risk Factors**:
- **Review bottlenecks** (Medium/Medium): Identify backup reviewers; set review turnaround time expectations
- **Insufficient reviewer expertise** (Medium/High): Match reviewers to documentation types based on domain knowledge

**Notes**:
Documentation review can be less formal than code review for informal documents (memos, meeting notes) but should be rigorous for deliverable documents (requirements, design, test reports).

**Parallelization**: Can run in parallel with 1.3.2 and other Phase 1 tasks.

---

### 1.4 Stage Gate Reviews

**Level**: 2
**Parent**: 1.0
**Duration**: 1 week
**Effort**: 16 person-hours

**Description**:
Establish stage gate review processes that provide formal decision points between project stages. Stage gates ensure that each stage meets its exit criteria before the project proceeds to the next stage, reducing risk of downstream problems due to incomplete or inadequate prior work. This subsystem defines gate criteria, review procedures, stakeholder roles, documentation requirements, and go/no-go decision processes. Four major stage gates exist in this project: Stage 1→2, Stage 2→3, Stage 3→4, and Stage 4→Integration.

**Deliverables**:
- Stage gate review procedures document
- Gate criteria for each stage transition
- Review schedule and milestones
- Stakeholder roles and responsibilities matrix

**Prerequisites/Dependencies**:
- 1.1: Requirements Management (criteria should trace to requirements)

**Required Resources**:
- **Personnel**: Project manager, systems engineer, stakeholders
- **Hardware**: None
- **Software/Tools**: None specific
- **Budget**: None

**Success Criteria**:
- Gate criteria defined for all four stage transitions
- Criteria are measurable and verifiable
- Review procedures specify preparation, conduct, and decision-making steps
- Stakeholder roles clearly assigned

**Risk Factors**:
- **Criteria too stringent** (Medium/High): Balance rigor with pragmatism; allow conditional approval with action items
- **Stakeholder unavailability** (Medium/Medium): Schedule gate reviews well in advance; identify backup decision-makers

**Notes**:
Stage gate reviews are critical for this project's phased approach. Gates should be scheduled at least 2 weeks before planned stage transition to allow time for remediation if criteria not met.

**Parallelization**: Can run in parallel with other Phase 1 tasks.

---

### 1.4.1 Stage Gate Criteria Definition

**Level**: 3
**Parent**: 1.4
**Duration**: 3 days
**Effort**: 8 person-hours

**Description**:
Define specific, measurable criteria that must be satisfied before proceeding from one project stage to the next. Criteria include technical completion (deliverables produced, tests passed, performance requirements met), documentation completion (required documents approved), risk mitigation (identified risks addressed or accepted), and readiness assessment (resources and prerequisites for next stage confirmed). Each criterion should be objective and verifiable through inspection, analysis, demonstration, or test. Organize criteria into exit criteria (for departing stage) and entry criteria (for entering stage).

**Deliverables**:
- Stage gate criteria document covering all four gates
- Verification methods for each criterion
- Criterion priority/criticality designation (mandatory vs. desired)

**Prerequisites/Dependencies**:
- 1.1.2: Requirements Documentation (criteria should align with requirements)

**Required Resources**:
- **Personnel**: Systems engineer, project manager
- **Hardware**: None
- **Software/Tools**: None
- **Budget**: None

**Success Criteria**:
- Minimum 8-12 criteria defined per gate
- Each criterion has clear verification method
- Criteria cover technical, documentation, and readiness aspects
- Criteria approved by project stakeholders

**Risk Factors**:
- **Subjective criteria** (High/High): Ensure all criteria are objective and measurable
- **Overly complex criteria** (Medium/Medium): Focus on essential quality indicators, not exhaustive checklists

**Notes**:
Example Stage 1→2 exit criteria: "SITL simulation achieves stable hover for 60 seconds", "Baseline PID controller parameters documented and traced to requirements", "Stage 1 test report approved".

**Parallelization**: Should complete early; can run in parallel with other Phase 1 tasks.

---

### 1.4.2 Review Schedule

**Level**: 3
**Parent**: 1.4
**Duration**: 1 day
**Effort**: 4 person-hours

**Description**:
Create project schedule that identifies planned dates for each stage gate review, accounting for stage durations, dependencies, resource availability, and buffer time. Schedule should indicate review preparation periods (typically 1-2 weeks before review), review conduct (1-2 days), and decision/action periods (1 week after review). Coordinate review dates with stakeholder calendars to maximize attendance. Build in schedule contingency for potential gate failures requiring remediation and re-review. Integrate gate schedule with overall project timeline and publish to all stakeholders.

**Deliverables**:
- Stage gate review schedule with dates
- Review preparation period definitions
- Schedule buffer allocation rationale
- Published calendar events for reviews

**Prerequisites/Dependencies**:
- Overall project timeline estimate (derived from WBS duration estimates)

**Required Resources**:
- **Personnel**: Project manager
- **Hardware**: None
- **Software/Tools**: Project scheduling tool (MS Project, Primavera, or spreadsheet)
- **Budget**: None

**Success Criteria**:
- All four gate review dates scheduled
- Preparation periods allocated before each review
- Stakeholder calendars checked for major conflicts
- Schedule includes buffer for potential remediation

**Risk Factors**:
- **Unrealistic schedule** (High/High): Validate durations against WBS estimates; include contingency
- **Stakeholder conflicts** (Medium/Medium): Identify early and negotiate dates; prioritize critical stakeholders

**Notes**:
For a solo developer working part-time, estimate Stage 1: 8-10 weeks, Stage 2: 10-12 weeks, Stage 3: 6-8 weeks, Stage 4: 10-14 weeks (includes hardware lead times). Schedule gates accordingly.

**Parallelization**: Must follow overall project timeline estimation; can run in parallel with 1.4.3 and 1.4.4.

---

### 1.4.3 Stakeholder Identification

**Level**: 3
**Parent**: 1.4
**Duration**: 1 day
**Effort**: 2 person-hours

**Description**:
Identify all stakeholders who have decision authority, review responsibilities, or information interest in stage gate reviews. Stakeholders may include project sponsor, technical advisors, systems engineer, development team leads, test lead, and potential users. For each stakeholder, document their role in gate reviews (decision-maker, reviewer, observer), specific areas of expertise or concern, and availability constraints. Create stakeholder communication plan for gate preparation and follow-up. For academic or solo projects, stakeholders might include advisors, mentors, or peer reviewers.

**Deliverables**:
- Stakeholder identification matrix
- Stakeholder roles and responsibilities
- Stakeholder communication plan

**Prerequisites/Dependencies**:
- None

**Required Resources**:
- **Personnel**: Project manager
- **Hardware**: None
- **Software/Tools**: None
- **Budget**: None

**Success Criteria**:
- All relevant stakeholders identified with contact information
- Decision-making authority clearly assigned
- Communication preferences documented for each stakeholder

**Risk Factors**:
- **Missing key stakeholders** (Medium/High): Conduct thorough stakeholder analysis; review with project sponsor
- **Unclear decision authority** (Medium/High): Establish single final decision-maker per gate (or clear voting process)

**Notes**:
For solo/academic projects, consider inviting external reviewers (faculty, industry practitioners) to provide objective feedback at key gates.

**Parallelization**: Can run in parallel with all other Phase 1 tasks.

---

### 1.4.4 Review Procedures

**Level**: 3
**Parent**: 1.4
**Duration**: 1 day
**Effort**: 4 person-hours

**Description**:
Document detailed procedures for conducting stage gate reviews including pre-review preparation activities, review meeting structure and agenda, presentation materials required, evaluation and scoring methods, decision-making process, action item assignment, and post-review follow-up. Procedures should specify how criteria are verified, how evidence is presented, how reviewers provide feedback, and how go/no-go decisions are made. Include templates for review packages, review agendas, and review reports. Define escalation procedures for disagreements or borderline cases.

**Deliverables**:
- Stage gate review procedure document
- Review meeting agenda template
- Review package checklist
- Review report template
- Decision-making process flowchart

**Prerequisites/Dependencies**:
- 1.4.1: Stage Gate Criteria Definition

**Required Resources**:
- **Personnel**: Project manager, systems engineer
- **Hardware**: None
- **Software/Tools**: None specific
- **Budget**: None

**Success Criteria**:
- Procedures cover all review phases (preparation, conduct, follow-up)
- Templates exist for all review artifacts
- Decision-making process clear and objective
- Procedures approved by key stakeholders

**Risk Factors**:
- **Process too bureaucratic** (Medium/Medium): Tailor formality to project scale; avoid unnecessary overhead
- **Ambiguous decision criteria** (Medium/High): Define clear go/no-go thresholds; specify when conditional approval is acceptable

**Notes**:
Review procedures should be lightweight for small teams but rigorous enough to ensure thorough evaluation. Consider using scoring rubrics for objective criteria assessment.

**Parallelization**: Must follow 1.4.1; can run in parallel with 1.4.2 and 1.4.3.

---

## Phase Resource Requirements

### Personnel Skills Matrix

| Work Package | Systems Engineer | Software Dev | Config Manager | Tech Writer | Project Manager |
|--------------|-----------------|--------------|----------------|-------------|-----------------|
| 1.1 Requirements | **Lead** (32h) | - | - | Support (8h) | Review (4h) |
| 1.2 Version Control | Review (8h) | Support (16h) | **Lead** (32h) | - | Review (4h) |
| 1.3 Documentation | Support (8h) | Support (8h) | - | **Lead** (20h) | Review (4h) |
| 1.4 Stage Gates | **Co-lead** (8h) | - | - | - | **Co-lead** (12h) |

### Hardware and Software Requirements

**Development Workstations**: Standard developer laptops (Windows/Linux/Mac)
**CI/CD Server**:
- 8+ CPU cores
- 16+ GB RAM
- 100+ GB storage
- Can be cloud-based (AWS EC2 t3.xlarge or equivalent)

**Software Stack**:
- Version Control: Git + GitLab/GitHub/Bitbucket
- Requirements: DOORS Alternatives (Jama Connect, Azure DevOps, or Doorstop open-source)
- CI/CD: Jenkins/GitLab CI/CircleCI
- Documentation: Sphinx + MyST or MkDocs + Material theme
- Containerization: Docker Desktop or Docker Engine

### Budget Breakdown

| Item | Low Cost | High Cost | Notes |
|------|----------|-----------|-------|
| Requirements Tool | $0 | $500 | FOSS options available |
| CI/CD Hosting | $0 | $300 | GitHub Actions free tier usable |
| Documentation Hosting | $0 | $0 | ReadTheDocs/GitHub Pages free |
| Development Tools | $0 | $200 | Most tooling is open-source |
| **Phase 1 Total** | **$0** | **$1,000** | Can start with $0 using FOSS |

---

## Phase Risks and Mitigations

### High-Priority Risks

| Risk | Probability | Impact | Mitigation Strategy |
|------|------------|--------|-------------------|
| Incomplete requirements capture | High | High | Conduct thorough stakeholder interviews; iterate on requirements; accept that refinement is ongoing |
| CI/CD build environment complexity | Medium | High | Use containerization; document setup thoroughly; start simple and iterate |
| Documentation drift from code | High | High | Automate docs generation; link docs to PR reviews; make docs part of definition-of-done |
| Requirements ambiguity | Medium | High | Apply SMART criteria; conduct peer reviews; use examples and test cases |

### Medium-Priority Risks

| Risk | Probability | Impact | Mitigation Strategy |
|------|------------|--------|-------------------|
| Tool selection disagreements | Medium | Low | Build consensus early; evaluate tools against criteria; allow pilot testing |
| Learning curve for new tools | Medium | Medium | Allocate training time; pair experienced with novice users; use familiar tools where reasonable |
| Infrastructure scope creep | Medium | Medium | Define MVP feature set; defer nice-to-haves; time-box Phase 1 to 2 weeks max |
| Build environment not reproducible | Medium | High | Containerize all build environments; version-pin all dependencies; document from-scratch setup |

---

## Cross-References

### Prerequisites
- **None**: Phase 1 is the project entry point

### Outputs Used By
- **Phase 2 (SITL Baseline)**: Requires completed repository, CI/CD pipelines, and requirements database before significant development begins
- **All subsequent phases**: All phases depend on infrastructure established here

### Related Documents
- **Master Overview**: See [00_WBS_Master_Overview.md](./00_WBS_Master_Overview.md) for high-level timeline and resource allocation
- **Next Phase**: Proceed to [02_WBS_Phase2_SITL_Baseline.md](./02_WBS_Phase2_SITL_Baseline.md) after Phase 1 completion

---

## Document Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | February 7, 2026 | Ratan Lal Bunkar | Initial detailed WBS (part of monolithic document) |
| 2.0 | February 8, 2026 | Ratan Lal Bunkar | Extracted Phase 1 into standalone document with enhanced navigation |

---

**End of Phase 1 Document**

**Navigate to**: [Master Overview](./00_WBS_Master_Overview.md) | [Phase 2 →](./02_WBS_Phase2_SITL_Baseline.md)
