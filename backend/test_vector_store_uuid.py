import uuid
import sys
import os
sys.path.insert(0, os.getcwd())

from src.services.vector_store_service import vector_store_service
from src.services.embedding_service import embedding_service

# Test with a properly formatted UUID
try:
    # Generate a test embedding
    test_text = "Test document for vector store"
    test_embedding = embedding_service.generate_embedding(test_text)
    
    # Use a UUID for the document ID
    doc_id = str(uuid.uuid4())  # Generate a proper UUID
    
    # Try to add a document with UUID
    success = vector_store_service.add_document(
        doc_id=doc_id,
        content=test_text,
        embedding=test_embedding,
        source_location="Test location",
        book_id="test_book"
    )
    
    if success:
        print(f"Successfully added document to vector store with UUID: {doc_id}")
        
        # Try to search for the document
        search_results = vector_store_service.search(
            query_embedding=test_embedding,
            query_text="Test document for vector store"
        )
        
        print(f"Search returned {len(search_results)} results")
        if search_results:
            print(f"First result ID: {search_results[0].id}")
            print(f"First result content (truncated): {search_results[0].content[:50]}...")
    else:
        print("Failed to add document to vector store")
        
except Exception as e:
    print(f"Error with vector store service: {e}")
    import traceback
    traceback.print_exc()