# Research Findings: Backend API for Physical AI Textbook Platform

## Identified Unknowns and Resolutions

### 1. How will the personalization engine work without user authentication?

**Decision**: The personalization engine will accept user preferences as parameters in API requests or through frontend configuration.

**Rationale**: Without authentication, personalization can still be achieved by allowing users to specify their preferences when making requests or by tracking preferences in client-side storage.

**Implementation**:
- Add user preference parameters to personalization API endpoints
- Allow frontend to pass user background information (software experience, hardware availability) with requests
- Implement default personalization for anonymous users

**Alternatives considered**:
- Storing preferences in local storage (implemented: as fallback)
- Cookie-based preferences (rejected: privacy concerns)

### 2. What specific data validation rules apply to the personalization parameters?

**Decision**: Personalization parameters will use strict validation with predefined enum values.

**Rationale**: Using predefined values ensures consistency and makes personalization rules easier to implement and maintain.

**Implementation**:
- `softwareBackground`: Must be one of ["beginner", "python_intermediate", "ros2_developer", "ai_robotics_expert"]
- `hardwareBackground`: Must be one of ["no_gpu", "rtx_laptop", "rtx_workstation", "jetson_kit", "cloud"]
- Backend will validate and reject any other values
- Default to "beginner" and "no_gpu" if not specified

**Alternatives considered**:
- Free text field with validation (rejected: would make personalization difficult)
- Numeric scale (rejected: doesn't capture the required specific categories)

### 3. What are the performance requirements for RAG operations?

**Decision**: RAG operations must complete within 2000ms (p95 percentile) with support for 1000 concurrent users.

**Rationale**: This requirement ensures a responsive user experience while handling the complexity of RAG operations. The 2s threshold accounts for API latency and processing time.

**Implementation**:
- Use connection pooling for database operations
- Implement caching for frequently accessed content
- Optimize embedding generation and retrieval
- Monitor performance metrics during testing

**Alternatives considered**:
- Higher performance threshold (rejected: too slow for good UX)
- Different concurrent user count (rejected: 1000 based on scalability requirements)

## Additional Research Findings

### RAG Implementation Approach

**Decision**: Use Cohere embeddings with Qdrant vector store for efficient content retrieval.

**Rationale**: This combination provides good performance and accuracy for retrieving relevant textbook content based on user queries.

**Implementation**:
- Use Cohere to generate embeddings for textbook content
- Store embeddings in Qdrant vector store
- Implement similarity search for context retrieval

### Security Considerations

**Decision**: Implement API key validation, rate limiting, and input validation as specified in the feature requirements.

**Rationale**: These are standard security practices for API services that help protect against abuse and ensure data integrity.

**Implementation**:
- Validate API keys for protected endpoints
- Implement rate limiting for all endpoints
- Use Pydantic models for input validation