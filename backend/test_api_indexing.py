import requests
import json

# Load sample book content exactly as it would come from the API
with open('sample_book_content.json', 'r') as f:
    sample_content = json.load(f)

# Prepare the payload for indexing exactly as the API expects
payload = {
    "book_id": "ai_robotics_book",
    "content_blocks": sample_content
}

# Send request to index the book content via the API
response = requests.post('http://127.0.0.1:8000/api/v1/book-content/index', json=payload)

print(f"Status Code: {response.status_code}")
print(f"Response: {response.json()}")