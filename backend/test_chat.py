import requests

# Test the chat endpoint
payload = {
    "message": "What is AI and Robotics about?",
    "book_id": "ai_robotics_book"
}

response = requests.post('http://127.0.0.1:8000/api/v1/chat', json=payload)

print(f"Status Code: {response.status_code}")
print(f"Response: {response.json()}")