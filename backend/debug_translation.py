from fastapi.testclient import TestClient
from src.api.main import app

client = TestClient(app)

def debug_translation_endpoint():
    print("Testing translation endpoint with valid request...")
    
    # Test with valid data
    response = client.post(
        "/api/v1/translate",
        json={"text": "Hello world"}
    )
    
    print(f"Status Code: {response.status_code}")
    print(f"Response: {response.text}")
    
    print("\nTesting with both text and selected_text...")
    
    response2 = client.post(
        "/api/v1/translate",
        json={
            "text": "This is the main text",
            "selected_text": "This is the selected text"
        }
    )
    
    print(f"Status Code: {response2.status_code}")
    print(f"Response: {response2.text}")
    
    print("\nTesting with empty text...")
    
    response3 = client.post(
        "/api/v1/translate",
        json={"text": ""}
    )
    
    print(f"Status Code: {response3.status_code}")
    print(f"Response: {response3.text}")
    
    print("\nTesting with no text field...")
    
    response4 = client.post(
        "/api/v1/translate",
        json={}
    )
    
    print(f"Status Code: {response4.status_code}")
    print(f"Response: {response4.text}")

if __name__ == "__main__":
    debug_translation_endpoint()