# Quickstart Guide: Content Tagging System

## Overview
This guide will help you set up and use the content tagging system for your Physical AI & Humanoid Robotics book.

## Prerequisites
- Node.js (if using JavaScript implementation)
- Access to the content files in the `/docs` directory
- Understanding of the tagging rules defined in the specification

## Installation
The tagging system can be implemented as a command-line tool. Here's a basic implementation approach:

```bash
# Clone or access the tagging tool
git clone <repository-url> # if available as separate tool

# Or run directly from the project
cd d:/ai-book-pro/ai-robo-bk
```

## Usage

### 1. Analyze Content
First, analyze your content to see what tags would be applied:

```bash
# Example command structure
node tag-content.js analyze --path ./docs/
```

### 2. Apply Tags
Once you've verified the analysis, apply the tags to your content:

```bash
# Example command structure
node tag-content.js tag --path ./docs/ --backup
```

### 3. Verify Results
Check that tags were applied correctly and that existing metadata was preserved:

```bash
# Example command structure
node tag-content.js verify --path ./docs/
```

## Example Implementation

Here's a basic JavaScript implementation approach:

```javascript
// tag-content.js
const fs = require('fs');
const path = require('path');
const matter = require('gray-matter'); // for parsing frontmatter

function analyzeFile(filePath) {
  const fileContent = fs.readFileSync(filePath, 'utf8');
  const { data: frontmatter, content } = matter(fileContent);
  
  // Determine tags based on file path and content
  const tags = determineTags(filePath, content);
  
  return {
    filePath,
    ...tags,
    originalMetadata: frontmatter
  };
}

function tagFile(filePath) {
  const fileContent = fs.readFileSync(filePath, 'utf8');
  const { data: frontmatter, content } = matter(fileContent);
  
  // Determine tags based on file path and content
  const tags = determineTags(filePath, content);
  
  // Preserve original metadata and add new tags
  const updatedFrontmatter = {
    ...frontmatter,
    ...tags
  };
  
  // Write updated content back to file
  const updatedContent = matter.stringify(content, updatedFrontmatter);
  fs.writeFileSync(filePath, updatedContent);
  
  return {
    success: true,
    filePath,
    tags,
    originalMetadata: frontmatter
  };
}

function determineTags(filePath, content) {
  // Extract week number from file path
  const weekMatch = filePath.match(/w(\d+)/i);
  let difficulty = 'Beginner';
  
  if (weekMatch) {
    const week = parseInt(weekMatch[1]);
    if (week >= 1 && week <= 5) {
      difficulty = 'Beginner';
    } else if (week >= 6 && week <= 10) {
      difficulty = 'Intermediate';
    } else if (week >= 11 && week <= 13) {
      difficulty = 'Advanced';
    }
  }
  
  // Determine category based on content and file path
  let category = 'Foundations'; // default
  if (filePath.includes('ros2') || filePath.includes('rclpy')) {
    category = 'ROS2';
  } else if (filePath.includes('gazebo') || filePath.includes('simulation')) {
    category = 'Simulation';
  } else if (filePath.includes('isaac') || filePath.includes('nvidia')) {
    category = 'NVIDIA-Isaac';
  } else if (filePath.includes('humanoid') || filePath.includes('kinematics')) {
    category = 'Hardware';
  } else if (filePath.includes('vla') || filePath.includes('conversational')) {
    category = 'VLA-AI';
  }
  
  // Determine hardware focus based on content
  const hardwareFocus = [];
  if (content.includes('RTX') || content.includes('GPU')) {
    hardwareFocus.push('RTX-GPU');
  }
  if (content.includes('Jetson') || content.includes('Orin')) {
    hardwareFocus.push('Jetson-Orin');
  }
  if (content.includes('RealSense')) {
    hardwareFocus.push('RealSense');
  }
  
  // Determine software focus based on content
  const softwareFocus = [];
  if (content.includes('Python')) {
    softwareFocus.push('Python');
  }
  if (content.includes('Ubuntu')) {
    softwareFocus.push('Ubuntu');
  }
  if (content.includes('OpenAI') || content.includes('SDK')) {
    softwareFocus.push('OpenAI-SDK');
  }
  
  return {
    difficulty,
    category,
    hardware_focus: hardwareFocus,
    software_focus: softwareFocus
  };
}

// Command line interface
const args = process.argv.slice(2);
const command = args[0];
const filePath = args[1] || './docs';

if (command === 'analyze') {
  // Analyze all markdown files in the directory
  const files = getAllMarkdownFiles(filePath);
  files.forEach(file => {
    const result = analyzeFile(file);
    console.log(JSON.stringify(result, null, 2));
  });
} else if (command === 'tag') {
  // Tag all markdown files in the directory
  const files = getAllMarkdownFiles(filePath);
  files.forEach(file => {
    const result = tagFile(file);
    console.log(`Tagged: ${result.filePath}`);
  });
}

function getAllMarkdownFiles(dir) {
  const files = [];
  const items = fs.readdirSync(dir);
  
  items.forEach(item => {
    const fullPath = path.join(dir, item);
    const stat = fs.statSync(fullPath);
    
    if (stat.isDirectory()) {
      files.push(...getAllMarkdownFiles(fullPath));
    } else if (item.endsWith('.md')) {
      files.push(fullPath);
    }
  });
  
  return files;
}
```

## Validation
After tagging, verify that:
1. All files have the appropriate tags applied
2. Existing id, title, and slug values are preserved
3. Tags conform to the predefined lists
4. Content files remain valid and accessible

## Troubleshooting
- If a file doesn't get the expected tags, check if the file naming convention follows the expected pattern
- If existing metadata is lost, verify that the tagging process preserves the original frontmatter
- If tags don't conform to the predefined lists, check the tagging logic for validation