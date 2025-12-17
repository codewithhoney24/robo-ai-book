import requests
import json

# Load sample book content
with open('sample_book_content.json', 'r') as f:
    sample_content = json.load(f)

# Prepare the payload for indexing
payload = {
    "book_id": "ai_robotics_book",
    "content_blocks": sample_content
}

# Send request to index the book content
response = requests.post('http://127.0.0.1:8000/api/v1/book-content/index', json=payload)

print(f"Status Code: {response.status_code}")
print(f"Response: {response.json()}")