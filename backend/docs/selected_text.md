# Selected Text Feature Documentation

## Overview
The selected text feature allows users to select specific text in the book and ask questions about that text to get more targeted information about specific concepts or sections. This enhances the relevance of responses by focusing the RAG (Retrieval-Augmented Generation) system on the specific content the user is interested in.

## How It Works
1. User selects text in the book interface
2. User asks a question related to the selected text
3. The system combines the original query with the selected text to create a more specific search query
4. The RAG system retrieves content that's relevant to both the query and the selected text
5. A more targeted response is generated based on the retrieved context

## API Integration
The feature is implemented through the `/chat/selected-text` endpoint:

### Request Format
```json
{
  "query": "Your question about the selected text",
  "selected_text": "The specific text that was selected in the book",
  "session_id": "optional session identifier"
}
```

### Response Format
```json
{
  "response": "Generated answer based on both the query and selected text",
  "relevance_score": 0.95,
  "retrieved_contexts": [
    {
      "content": "Relevant book content used to generate the response",
      "source_location": "Chapter 1, Section 2",
      "relevance_score": 0.89
    }
  ]
}
```

## Technical Implementation
### 1. Query Enhancement
- The system combines the user's query with the selected text to create a more targeted search query
- This helps retrieve more relevant content from the vector store

### 2. Context Prioritization
- Retrieved contexts are re-ranked based on their similarity to the selected text
- This ensures that responses are more focused on the specific content the user highlighted

### 3. Embedding Strategy
- Special embedding generation method (`generate_embedding_for_combined_query`) is used when selected text is provided
- This creates an embedding that captures both the query intent and the selected text context

## Best Practices for Frontend Integration
1. Capture selected text accurately, avoiding capturing too much extraneous content
2. Provide clear UI indicators showing that selected text will be used for the query
3. Allow users to modify or confirm the selected text before submitting the query
4. Handle cases where no text is selected by falling back to the regular chat endpoint

## Performance Considerations
- The selected text feature adds minimal overhead to the query processing time
- The system still maintains sub-second response times even with selected text processing
- Context prioritization happens efficiently as a post-processing step after retrieval

## Error Handling
- If the selected text is empty or very short, the system gracefully falls back to standard query processing
- Appropriate error messages are returned if the selected text is too long (exceeds character limits)
- Sessions are maintained properly even when using the selected text feature