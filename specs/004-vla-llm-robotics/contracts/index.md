# Contracts: Vision-Language-Action (VLA) for LLM-Robotics Integration

## Overview

This document defines the contracts between components in Module 4: Vision-Language-Action (VLA) for LLM-Robotics Integration. These contracts specify the interfaces, data formats, and communication protocols between the various system components.

## 1. Voice Processing Contract

### Interface: VoiceInputProcessor
**Purpose**: Convert voice commands to text and handle audio processing

**Input**:
- **Method**: POST /process-voice
- **Content-Type**: audio/wav, audio/mp3, or audio/ogg
- **Headers**: 
  - `Accept-Language`: Language code (e.g., en-US, fr-FR)
  - `User-ID`: (Optional) Identifier of the user issuing the command

**Output**:
- **Status Codes**:
  - `200`: Successful processing
  - `400`: Invalid audio format
  - `422`: Audio quality too low for processing
  - `500`: Processing error

- **Response Body** (application/json):
```json
{
  "transcribed_text": "string",
  "confidence": "float (0-1)",
  "language": "string",
  "processing_time": "float (seconds)",
  "command_id": "string"
}
```

### Interface: VoiceCommandPublisher
**Purpose**: Publish voice commands to the ROS 2 system

**Message Type**: `std_msgs/String`
**Topic**: `/voice_commands`
**Message Format**:
```json
{
  "data": "transcribed voice command text",
  "timestamp": "ISO 8601 timestamp",
  "command_id": "unique identifier"
}
```

## 2. LLM Cognitive Planning Contract

### Interface: CognitivePlanner
**Purpose**: Translate natural language commands into ROS 2 action sequences

**Input**:
- **Method**: POST /plan
- **Content-Type**: application/json
- **Request Body**:
```json
{
  "command_text": "string",
  "robot_capabilities": {
    "navigation": "boolean",
    "manipulation": "boolean",
    "perception": "boolean",
    "supported_actions": ["string"]
  },
  "environment_context": {
    "object_positions": [
      {
        "name": "string",
        "class": "string",
        "position": {
          "x": "float",
          "y": "float",
          "z": "float"
        }
      }
    ],
    "robot_state": {
      "position": {
        "x": "float",
        "y": "float",
        "z": "float"
      },
      "orientation": {
        "x": "float",
        "y": "float", 
        "z": "float",
        "w": "float"
      }
    }
  }
}
```

**Output**:
- **Status Codes**:
  - `200`: Plan generated successfully
  - `400`: Invalid command format
  - `422`: Command cannot be executed with available capabilities
  - `500`: LLM service error

- **Response Body** (application/json):
```json
{
  "plan_id": "string",
  "action_sequence": [
    {
      "action_type": "string (navigation|manipulation|perception|other)",
      "parameters": {
        "target_location": {
          "x": "float",
          "y": "float",
          "z": "float"
        },
        "object_name": "string",
        "gripper_position": "float (0-1)"
      },
      "description": "string",
      "timeout": "float (seconds)"
    }
  ],
  "estimated_duration": "float (seconds)",
  "confidence": "float (0-1)"
}
```

## 3. Computer Vision Contract

### Interface: VisionProcessor
**Purpose**: Process visual input and detect objects in the environment

**Input**:
- **Method**: POST /process-vision
- **Content-Type**: image/jpeg, image/png, or multipart/form-data
- **Request Body** if multipart:
  - `image`: Image file
  - `depth_map` (optional): Associated depth map
  - `camera_info` (optional): Camera calibration data

**Output**:
- **Status Codes**:
  - `200`: Processing successful
  - `400`: Invalid image format
  - `422`: Image quality too low for processing
  - `500`: Processing error

- **Response Body** (application/json):
```json
{
  "detection_id": "string",
  "detected_objects": [
    {
      "class_name": "string",
      "confidence": "float (0-1)",
      "position": {
        "x": "float",
        "y": "float",
        "z": "float"
      },
      "orientation": {
        "x": "float",
        "y": "float",
        "z": "float",
        "w": "float"
      },
      "bounding_box": {
        "x": "float",
        "y": "float",
        "width": "float",
        "height": "float"
      },
      "properties": {
        "color": "string",
        "size": "string"
      }
    }
  ],
  "processing_time": "float (seconds)"
}
```

### Interface: VisionDataPublisher
**Purpose**: Publish vision data to the ROS 2 system

**Message Type**: `sensor_msgs/Image` and `vision_msgs/DetectionArray`
**Topics**: 
- `/camera/image_raw` for raw images
- `/vision/detections` for processed detections

**Message Format** (for detections):
```json
{
  "header": {
    "stamp": "time",
    "frame_id": "string"
  },
  "detections": [
    {
      "bbox": {
        "xmin": "float",
        "ymin": "float", 
        "xmax": "float",
        "ymax": "float"
      },
      "results": [
        {
          "hypothesis": {
            "class_id": "string",
            "score": "float"
          }
        }
      ]
    }
  ]
}
```

## 4. ROS 2 Action Execution Contract

### Interface: ActionExecutor
**Purpose**: Execute action sequences generated by the cognitive planner

**Action Type**: `action_msgs/GoalStatus`
**Action Name**: `/execute_action_sequence`

**Goal Message**:
```json
{
  "plan_id": "string",
  "action_sequence": [
    {
      "action_type": "string",
      "parameters": "object",
      "timeout": "float (seconds)"
    }
  ],
  "execution_context": "object"
}
```

**Result Message**:
```json
{
  "plan_id": "string",
  "execution_status": "string (success|partial_success|failure)",
  "executed_actions_count": "integer",
  "total_actions_count": "integer",
  "error_message": "string (if failure)",
  "result_data": "object"
}
```

## 5. Voice-to-Action Integration Contract

### Interface: VLAOrchestrator
**Purpose**: Coordinate the entire VLA pipeline from voice input to robot action

**Input**:
- **Method**: POST /execute-voice-command
- **Content-Type**: application/json
- **Request Body**:
```json
{
  "voice_command": "string",
  "audio_data": "string (base64 encoded audio)",
  "context": {
    "robot_state": "object",
    "environment_state": "object"
  }
}
```

**Output**:
- **Status Codes**:
  - `200`: Command processed successfully
  - `202`: Command accepted, processing in progress
  - `400`: Invalid command format
  - `422`: Command cannot be executed with current state
  - `500`: Processing error

- **Response Body** (application/json):
```json
{
  "execution_id": "string",
  "status": "string (received|processing|completed|failed)",
  "estimated_completion_time": "float (seconds)",
  "progress": [
    {
      "step": "string (voice_processing|planning|execution)",
      "status": "string (pending|in_progress|completed|failed)",
      "timestamp": "ISO 8601 timestamp"
    }
  ]
}
```

## 6. Error Handling Contract

### Standard Error Response Format
All service errors follow this format:

```json
{
  "error_code": "string",
  "message": "string",
  "details": "object",
  "timestamp": "ISO 8601 timestamp"
}
```

### Common Error Codes
- `VOICE_PROCESSING_ERROR`: Issue with voice transcription
- `PLANNING_ERROR`: LLM failed to generate plan
- `EXECUTION_ERROR`: Robot failed to execute action
- `VISION_ERROR`: Computer vision processing failed
- `VALIDATION_ERROR`: Input validation failed
- `RESOURCE_UNAVAILABLE`: Required service not available

## 7. Data Validation Contract

### Input Validation Requirements
All interfaces must validate:
- Required fields present
- Data types correct
- String lengths within limits
- Numerical values within valid ranges
- Format compliance (e.g., ISO 8601 timestamps)

### Output Validation Requirements
All interfaces must ensure:
- Required fields populated
- Data types correct
- No sensitive data leakage
- Consistent format with documentation

## 8. Performance Contract

### Response Time Requirements
- Voice Processing: < 2 seconds for standard audio
- Cognitive Planning: < 5 seconds for complex commands
- Vision Processing: < 1 second for image analysis
- Action Execution: < 0.1 seconds for command initiation

### Availability Requirements
- System uptime: 99.5% during operational hours
- Service availability: Each component must be available independently
- Fallback mechanisms: System must gracefully degrade when components are unavailable

## 9. Security Contract

### Authentication
- API keys required for external access
- Certificate-based authentication for ROS 2 communication
- Role-based access control for administrative functions

### Data Protection
- Encryption in transit for all communication
- Sensitive data (audio, video) retention limits
- API key rotation every 90 days

## 10. Monitoring Contract

### Required Metrics
- Request/response rates per interface
- Error rates and types
- Response time percentiles (p50, p95, p99)
- Component availability
- Resource utilization (CPU, memory, GPU)

### Logging Requirements
- Request/response logging with anonymized data
- Error logging with stack traces
- Audit trails for security-relevant events
- Performance monitoring logs