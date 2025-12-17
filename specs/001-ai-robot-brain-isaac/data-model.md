# Data Model: Additional Learning Resources

## Entities

### GlossaryTerm
- **term**: string (required) - The technical term being defined
- **definition**: string (required) - Clear, concise definition of the term
- **category**: string (required) - Classification (e.g., "ROS", "AI", "Simulation", "Hardware")
- **related_terms**: list of strings - Cross-references to related terms
- **examples**: list of strings - Practical examples of the term usage
- **source**: string (optional) - Reference source for the definition
- **last_updated**: datetime - Timestamp of last modification

### AssessmentQuestion
- **id**: string (required) - Unique identifier for the question
- **question_type**: string (required) - Type of question (e.g., "multiple_choice", "short_answer", "scenario", "practical")
- **prompt**: string (required) - The question text
- **category**: string (required) - Related topic or module
- **difficulty_level**: string (required) - "beginner", "intermediate", "advanced"
- **options**: list of strings - For multiple choice questions
- **correct_answer**: string or list (required) - The correct answer(s)
- **explanation**: string (required) - Explanation of the correct answer
- **learning_objectives**: list of strings - The learning objectives being assessed
- **estimated_duration**: integer - Time in minutes to complete the question

### TroubleshootingGuide
- **title**: string (required) - Descriptive title of the guide
- **problem_category**: string (required) - Classification (e.g., "installation", "simulation", "hardware", "network")
- **symptoms**: list of strings - Observable symptoms of the problem
- **possible_causes**: list of strings - Potential root causes
- **diagnostic_steps**: list of TroubleshootingStep - Systematic steps to diagnose the issue
- **solution_steps**: list of TroubleshootingStep - Steps to resolve the issue
- **verification_steps**: list of strings - How to confirm the problem is resolved
- **related_guides**: list of strings - Other related troubleshooting guides
- **last_updated**: datetime - Timestamp of last modification

### TroubleshootingStep
- **step_number**: integer (required) - Sequential number of the step
- **description**: string (required) - Description of the step
- **command**: string (optional) - Command to execute (if applicable)
- **expected_result**: string (required) - What should happen when step is executed
- **troubleshooting_tip**: string (optional) - Additional guidance for this step

### LearningResource
- **resource_type**: string (required) - "glossary", "assessment", or "troubleshooting"
- **title**: string (required) - Title of the resource
- **content**: string (required) - The main content of the resource
- **target_audience**: string (required) - "beginner", "intermediate", or "advanced"
- **related_modules**: list of strings - Modules related to this resource
- **estimated_time**: integer - Time in minutes to review or complete
- **created_date**: datetime - When the resource was created
- **last_reviewed**: datetime - Last review date
- **review_status**: string - "draft", "reviewed", "published", "deprecated"

## Relationships

- GlossaryTerm belongs to one category
- AssessmentQuestion belongs to one category and relates to multiple learning objectives
- TroubleshootingGuide has many TroubleshootingStep
- TroubleshootingGuide has many related guides
- LearningResource may reference multiple GlossaryTerms
- LearningResource may include multiple AssessmentQuestions
- LearningResource may link to multiple TroubleshootingGuides

## State Transitions

### ResourceState
- **DRAFT**: Initial state, content is being created
- **REVIEW**: Ready for review by domain experts
- **PUBLISHED**: Approved and available to learners
- **DEPRECATED**: No longer relevant or accurate
- **ARCHIVED**: Removed but kept for historical reference

### AssessmentState
- **PENDING**: Question created but not yet reviewed
- **VALIDATED**: Reviewed by expert and confirmed accurate
- **ACTIVE**: In use in assessments
- **RETIRED**: No longer used but kept for reference