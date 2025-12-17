import requests
import json
import uuid

# Load sample book content
with open('sample_book_content.json', 'r') as f:
    sample_content = json.load(f)

# Prepare the payload for indexing, but with UUIDs instead of string IDs
content_blocks = []
for block in sample_content:
    # Generate a new UUID for the document ID to be compatible with Qdrant
    new_block = {
        "id": str(uuid.uuid4()),  # Replace string ID with UUID
        "content": block["content"],
        "vector_embedding": [],  # Will be populated by the service
        "source_location": block["source_location"],
        "book_id": block["book_id"]
    }
    content_blocks.append(new_block)

# Prepare the payload for indexing
payload = {
    "book_id": "ai_robotics_book",
    "content_blocks": content_blocks
}

# Send request to index the book content
response = requests.post('http://127.0.0.1:8000/api/v1/book-content/index', json=payload)

print(f"Status Code: {response.status_code}")
print(f"Response: {response.json()}")