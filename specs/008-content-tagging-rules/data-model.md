# Data Model: Content Tagging System

## Entities

### ContentTag
Represents metadata applied to content files for personalization

**Fields:**
- `difficulty` (string, required): Enum values ["Beginner", "Intermediate", "Advanced"]
- `category` (string, required): Enum values ["Foundations", "ROS2", "Simulation", "NVIDIA-Isaac", "Hardware", "VLA-AI"]
- `hardware_focus` (array of strings): Enum values ["RTX-GPU", "Jetson-Orin", "RealSense"]
- `software_focus` (array of strings): Enum values ["Python", "Ubuntu", "OpenAI-SDK"]

**Validation Rules:**
- difficulty must be one of the predefined values
- category must be one of the predefined values
- hardware_focus values must be from the predefined list
- software_focus values must be from the predefined list

### ContentFile
Represents a markdown file in the docs directory

**Fields:**
- `id` (string, required): Existing id from frontmatter (preserved)
- `title` (string, required): Existing title from frontmatter (preserved)
- `slug` (string, required): Existing slug from frontmatter (preserved)
- `filePath` (string, required): Path to the content file
- `tags` (ContentTag, required): New tags to be added to frontmatter

**State Transitions:**
- Initial state: File exists with id, title, slug in frontmatter
- After tagging: File has additional tags while preserving existing metadata

### TaggingRule
Represents the rules that determine how content should be categorized

**Fields:**
- `weekRange` (string, required): Week range for difficulty determination (e.g., "1-5", "6-10", "11-13")
- `difficulty` (string, required): Associated difficulty level
- `contentKeywords` (array of strings): Keywords that indicate specific categories
- `hardwareKeywords` (array of strings): Keywords that indicate hardware requirements
- `softwareKeywords` (array of strings): Keywords that indicate software requirements