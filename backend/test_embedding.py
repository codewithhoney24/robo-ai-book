import sys
import os
sys.path.insert(0, os.getcwd())

from src.services.embedding_service import embedding_service

# Test if embedding service works
try:
    test_text = "Hello, this is a test for embedding generation."
    embedding = embedding_service.generate_embedding(test_text)
    print(f"Embedding generated successfully. Length: {len(embedding)}")
    print(f"First 5 values: {embedding[:5]}")
except Exception as e:
    print(f"Error generating embedding: {e}")
    import traceback
    traceback.print_exc()